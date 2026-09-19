/*
 * ESP32 Line Follower Robot
 * - 8 Sensor Line Position Detection
 * - Calibration via Button Toggle
 * - Closed-Loop Control using PID
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

static const char *TAG = "LINE_FOLLOWER";

/* =========================================================================
 * 1. HARDWARE PINS & SENSOR CONFIGURATION
 * ========================================================================= */
#define NUM_SENSORS        8
#define CALIB_BUTTON_GPIO  GPIO_NUM_4
#define INVERT_LINE_LOGIC  false

// ADC Channels assigned to sensors (6 on ADC1, 2 on ADC2)
static const adc_channel_t adc1_channels[6] = {
    ADC_CHANNEL_0, ADC_CHANNEL_3, ADC_CHANNEL_4,
    ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7
};
static const adc_channel_t adc2_channels[2] = {
    ADC_CHANNEL_8, ADC_CHANNEL_9
};

// Distance of each sensor from the center line (in mm or relative units)
static const float sensor_weight[NUM_SENSORS] = {
    -52.5f, -37.5f, -22.5f, -7.5f, 7.5f, 22.5f, 37.5f, 52.5f
};

// Global variables for calibration and system state
static adc_oneshot_unit_handle_t adc1_handle;
static adc_oneshot_unit_handle_t adc2_handle;

static int sensor_min[NUM_SENSORS];
static int sensor_max[NUM_SENSORS];

static volatile bool is_calibrating = false;
static volatile bool button_pressed  = false;

/* =========================================================================
 * 2. MOTOR CONTROL & PWM CONFIGURATION
 * ========================================================================= */
#define AIN1      GPIO_NUM_17
#define AIN2      GPIO_NUM_18
#define BIN1      GPIO_NUM_21
#define BIN2      GPIO_NUM_22
#define STBY      GPIO_NUM_23
#define PWMA_PIN  GPIO_NUM_16
#define PWMB_PIN  GPIO_NUM_19

// PWM Parameters for Speed Control
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT  // Resolution: 0 to 1023
#define LEDC_FREQUENCY  5000               // Frequency: 5000 Hz
#define CHANNEL_A       LEDC_CHANNEL_0
#define CHANNEL_B       LEDC_CHANNEL_1

#define MAX_DUTY        818                // ~80% max motor speed limit
#define BASE_SPEED      250                // Default forward speed

/* =========================================================================
 * 3. PID CONTROLLER STRUCT & STATE
 * ========================================================================= */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float integral_limit;
} pid_t;

static pid_t line_pid = {
    .kp             = 8.0f,
    .ki             = 0.0f,
    .kd             = 0.0f,
    .integral       = 0.0f,
    .prev_error     = 0.0f,
    .integral_limit = 50.0f
};

/* =========================================================================
 * 4. BUTTON & INTERRUPT FUNCTIONS
 * ========================================================================= */

// Interrupt Service Routine (ISR): executes instantly when button is clicked
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    button_pressed = true;
}

// Configures the calibration push button on GPIO 4
void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CALIB_BUTTON_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf);
    
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CALIB_BUTTON_GPIO, gpio_isr_handler, NULL);
}

/* =========================================================================
 * 5. SENSOR READ & CALIBRATION FUNCTIONS
 * ========================================================================= */

// Initializes ADC units for reading analog sensor values
void sensors_init(void)
{
    adc_oneshot_unit_init_cfg_t cfg1 = { .unit_id = ADC_UNIT_1, .clk_src = ADC_RTC_CLK_SRC_DEFAULT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg1, &adc1_handle));

    adc_oneshot_unit_init_cfg_t cfg2 = { .unit_id = ADC_UNIT_2, .clk_src = ADC_RTC_CLK_SRC_DEFAULT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg2, &adc2_handle));

    adc_oneshot_chan_cfg_t chan_cfg = { .bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_12 };
    
    for (int i = 0; i < 6; i++) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc1_channels[i], &chan_cfg));
    }
    for (int i = 0; i < 2; i++) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, adc2_channels[i], &chan_cfg));
    }

    // Initialize calibration bounds
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_min[i] = 4095;
        sensor_max[i] = 0;
    }
    
    ESP_LOGI(TAG, "Sensors initialized.");
}

// Reads raw ADC values (0 to 4095) from all 8 sensors
void read_sensors(int *raw)
{
    for (int i = 0; i < 6; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc1_channels[i], &raw[i]));
    }
    for (int i = 0; i < 2; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, adc2_channels[i], &raw[6 + i]));
    }
}

// Dynamically updates minimum and maximum sensor readings during calibration phase
void update_calibration(const int *raw)
{
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (raw[i] < sensor_min[i]) sensor_min[i] = raw[i];
        if (raw[i] > sensor_max[i]) sensor_max[i] = raw[i];
    }
}

// Maps raw analog values into a normalized 0–1000 range based on calibration
void get_calibrated_values(const int *raw, int *calibrated)
{
    for (int i = 0; i < NUM_SENSORS; i++) {
        int range = sensor_max[i] - sensor_min[i];
        if (range <= 0) {
            calibrated[i] = 0;
            continue;
        }

        int val = raw[i];
        if (val < sensor_min[i]) val = sensor_min[i];
        if (val > sensor_max[i]) val = sensor_max[i];

        int mapped = ((val - sensor_min[i]) * 1000) / range;
        calibrated[i] = INVERT_LINE_LOGIC ? (1000 - mapped) : mapped;
    }
}

// Computes the center position of the line relative to the sensor array
float get_weighted_position(const int *calibrated)
{
    float numerator = 0.0f;
    float denominator = 0.0f;

    for (int i = 0; i < NUM_SENSORS; i++) {
        numerator += (float)calibrated[i] * sensor_weight[i];
        denominator += (float)calibrated[i];
    }

    if (denominator == 0.0f) return 0.0f;
    return numerator / denominator;
}

/* =========================================================================
 * 6. PID COMPUTATION FUNCTION
 * ========================================================================= */

// Calculates steering correction based on position error
float pid_compute(pid_t *pid, float error, float dt)
{
    // Accumulate integral term with anti-windup clamping
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit)  pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    // Calculate derivative term (rate of change of error)
    float derivative = (dt > 0.0f) ? (error - pid->prev_error) / dt : 0.0f;
    pid->prev_error = error;

    // Total PID Output Formula
    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}

/* =========================================================================
 * 7. MOTOR DRIVER CONTROL FUNCTIONS
 * ========================================================================= */

// Configures directional GPIO pins for TB6612FNG motor driver
void motor_gpio_init(void)
{
    uint64_t pin_mask = (1ULL << AIN1) | (1ULL << AIN2) | 
                        (1ULL << BIN1) | (1ULL << BIN2) | 
                        (1ULL << STBY);

    gpio_config_t io_conf = {
        .pin_bit_mask = pin_mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Default state: disable standby and set direction pins LOW
    gpio_set_level(STBY, 0);
    gpio_set_level(AIN1, 0); 
    gpio_set_level(AIN2, 0);
    gpio_set_level(BIN1, 0); 
    gpio_set_level(BIN2, 0);
}

// Initializes ESP32 LEDC peripheral for motor PWM speed control
void motor_pwm_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = LEDC_FREQUENCY,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t chan_a = {
        .gpio_num   = PWMA_PIN,
        .speed_mode = LEDC_MODE,
        .channel    = CHANNEL_A,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&chan_a);

    ledc_channel_config_t chan_b = {
        .gpio_num   = PWMB_PIN,
        .speed_mode = LEDC_MODE,
        .channel    = CHANNEL_B,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&chan_b);
}

// Clamps duty cycle between safe bounds and applies it to PWM channel
void set_motor_speed(ledc_channel_t channel, int duty)
{
    if (duty > MAX_DUTY) duty = MAX_DUTY;
    if (duty < 0) duty = 0;

    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);
}

// Commands both left and right motors to drive forward at specified speeds
void drive(int left_speed, int right_speed)
{
    // Set both motor channels to forward direction
    gpio_set_level(AIN1, 1); 
    gpio_set_level(AIN2, 0);
    gpio_set_level(BIN1, 1); 
    gpio_set_level(BIN2, 0);

    set_motor_speed(CHANNEL_A, left_speed);
    set_motor_speed(CHANNEL_B, right_speed);
}

/* =========================================================================
 * 8. MAIN EXECUTION LOOP
 * ========================================================================= */
void app_main(void)
{
    // Initialize all hardware peripherals
    sensors_init();
    button_init();
    motor_gpio_init();
    motor_pwm_init();

    int raw[NUM_SENSORS];
    int calibrated[NUM_SENSORS];

    ESP_LOGI(TAG, "Press button to start calibration.");

    while (1) {
        // --- BUTTON HANDLING & DEBOUNCE ---
        if (button_pressed) {
            vTaskDelay(pdMS_TO_TICKS(50)); // 50ms software debounce
            
            if (gpio_get_level(CALIB_BUTTON_GPIO) == 0) {
                is_calibrating = !is_calibrating;
                
                gpio_set_level(STBY, 0); // Always disable motors when toggling states

                if (is_calibrating) {
                    ESP_LOGW(TAG, ">>> CALIBRATION STARTED <<<");
                    for (int i = 0; i < NUM_SENSORS; i++) {
                        sensor_min[i] = 4095;
                        sensor_max[i] = 0;
                    }
                } else {
                    ESP_LOGW(TAG, ">>> CALIBRATION DONE -- driving now <<<");
                    line_pid.integral   = 0.0f;
                    line_pid.prev_error = 0.0f;
                    gpio_set_level(STBY, 1); // Enable motor driver standby pin
                }
            }
            button_pressed = false;
        }

        // --- READ HARDWARE SENSORS ---
        read_sensors(raw);

        // --- CALIBRATION ROUTINE ---
        if (is_calibrating) {
            update_calibration(raw);
            vTaskDelay(pdMS_TO_TICKS(100)); // Sample every 100ms during calibration
            continue;
        }

        // --- MAIN DRIVE & PID ROUTINE ---
        get_calibrated_values(raw, calibrated);
        float position = get_weighted_position(calibrated);
        
        // Compute steering correction using 20ms fixed timestep (0.02s)
        float correction = pid_compute(&line_pid, position, 0.02f);

        int left_speed  = BASE_SPEED - (int)correction;
        int right_speed = BASE_SPEED + (int)correction;

        drive(left_speed, right_speed);

        ESP_LOGI(TAG, "pos: %6.2f  corr: %6.2f  L:%d R:%d", position, correction, left_speed, right_speed);

        vTaskDelay(pdMS_TO_TICKS(20)); // Fixed 20ms loop interval
    }
}