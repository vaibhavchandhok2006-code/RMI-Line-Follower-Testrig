/*
 * ESP32 Line Follower Robot
 * Features:
 * - 8 Sensor Line Position Detection
 * - Single-Button Control State Machine (GPIO 4)
 * - Closed-Loop PID Control
 * - Unconditional Straight Probe Window with Flexible Straight-Line Classifier
 */

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

static const char *TAG = "LINE_FOLLOWER";

/* =========================================================================
 * 1. HARDWARE PINS & SENSOR CONFIGURATION
 * ========================================================================= */
#define NUM_SENSORS        8
#define INVERT_LINE_LOGIC  false

// Single Control Button (GPIO 4)
#define BTN_SINGLE_GPIO    GPIO_NUM_4
#define DEBOUNCE_TIME_MS   300

// Status LEDs
#define LED_CALIB_GPIO     GPIO_NUM_2
#define LED_RUN_GPIO       GPIO_NUM_0

// ADC Channels assigned to sensors (6 on ADC1, 2 on ADC2)
static const adc_channel_t adc1_channels[6] = {
    ADC_CHANNEL_0, ADC_CHANNEL_3, ADC_CHANNEL_6,
    ADC_CHANNEL_7, ADC_CHANNEL_4, ADC_CHANNEL_5
};
static const adc_channel_t adc2_channels[2] = {
    ADC_CHANNEL_8, ADC_CHANNEL_9
};

// Distance of each sensor from the center line
static const float sensor_weight[NUM_SENSORS] = {
    -52.5f, -37.5f, -22.5f, -7.5f, 7.5f, 22.5f, 37.5f, 52.5f
};

static adc_oneshot_unit_handle_t adc1_handle;
static adc_oneshot_unit_handle_t adc2_handle;

static int sensor_min[NUM_SENSORS];
static int sensor_max[NUM_SENSORS];
static bool is_calibrated = false;

/* =========================================================================
 * 2. ROBOT STATE MACHINE
 * ========================================================================= */
typedef enum {
    ROBOT_STATE_IDLE = 0,
    ROBOT_STATE_CALIBRATING,
    ROBOT_STATE_RUNNING
} robot_state_t;

static volatile robot_state_t robot_state = ROBOT_STATE_IDLE;
static volatile bool btn_pressed = false;

/* =========================================================================
 * 3. MOTOR CONTROL & PWM CONFIGURATION
 * ========================================================================= */
#define AIN1      GPIO_NUM_17
#define AIN2      GPIO_NUM_18
#define BIN1      GPIO_NUM_13
#define BIN2      GPIO_NUM_22
#define STBY      GPIO_NUM_23
#define PWMA_PIN  GPIO_NUM_16
#define PWMB_PIN  GPIO_NUM_19

#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY  5000
#define CHANNEL_A       LEDC_CHANNEL_0
#define CHANNEL_B       LEDC_CHANNEL_1

#define MAX_DUTY        1023
#define BASE_SPEED      1000

/* =========================================================================
 * 4. JUNCTION PROBE & CLASSIFICATION CONFIGURATION
 * ========================================================================= */
#define JUNCTION_STRAIGHT_SPEED      500    // PWM speed during probe
#define JUNCTION_STRAIGHT_DRIVE_MS   500    // Time (ms) to drive straight past junction stem
#define JUNCTION_COOLDOWN_MS         1000   // Cooldown between triggers

#define CENTER_REGION_MASK           0x3C   // Middle 4 sensors: S5, S4, S3, S2 (0b00111100)

static bool junction_probing_active = false;
static int64_t junction_probe_start_us = 0;
static int64_t last_junction_time_us = 0;

/*
 * Returns true if current binary pattern matches known Left Junction triggers.
 */
bool is_left_junction_pattern(uint8_t binary)
{
    switch (binary)
    {
        case 0b11111100:
        case 0b11111000:
        case 0b11110000:
        case 0b11111110:
        case 0b11111001:
        case 0b01011000:
            return true;

        default:
            return false;
    }
}

/*
 * Multi-case + Count/Mask hybrid check for Straight Line Presence
 */
bool check_straight_line_exists(uint8_t binary, int count)
{
    // Specific pattern matches for standard/shifting straight lines
    switch (binary)
    {
        case 0b00011000: // Ideal center (S3, S4)
        case 0b00010000: // S4 single
        case 0b00001000: // S3 single
        case 0b00111000: // S3, S4, S5 (3-sensor wide)
        case 0b00011100: // S2, S3, S4 (3-sensor wide)
        case 0b00110000: // S4, S5
        case 0b00001100: // S2, S3
            return true;

        default:
            break;
    }

    // Fallback: Check if ANY of the middle 4 sensors are active with a valid line count
    if ((binary & CENTER_REGION_MASK) != 0 && count >= 1) {
        return true;
    }

    return false;
}

/* =========================================================================
 * 5. PID CONTROLLER STRUCT & STATE
 * ========================================================================= */
#define DERIVATIVE_FILTER_ALPHA 0.2f

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float integral_limit;
    float filtered_derivative;
    bool  first_sample;
} pid_t;

static pid_t line_pid = {
    .kp                  = 55.0f,
    .ki                  = 0.0f,
    .kd                  = 1.5f,
    .integral            = 0.0f,
    .prev_error          = 0.0f,
    .integral_limit      = 50.0f,
    .filtered_derivative  = 0.0f,
    .first_sample        = true
};

/* =========================================================================
 * 6. INTERRUPT SERVICE ROUTINES & DEBOUNCED SINGLE BUTTON
 * ========================================================================= */
static volatile uint64_t last_btn_time_ms = 0;

static void IRAM_ATTR button_isr_handler(void* arg)
{
    uint64_t now_ms = esp_timer_get_time() / 1000;

    if ((now_ms - last_btn_time_ms) > DEBOUNCE_TIME_MS) {
        btn_pressed = true;
        last_btn_time_ms = now_ms;
    }
}

void buttons_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_SINGLE_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_SINGLE_GPIO, button_isr_handler, (void*) BTN_SINGLE_GPIO);
}

/* =========================================================================
 * 7. LED STATUS INDICATORS
 * ========================================================================= */
void leds_init(void)
{
    uint64_t mask = (1ULL << LED_CALIB_GPIO) | (1ULL << LED_RUN_GPIO);
    gpio_config_t io_conf = {
        .pin_bit_mask = mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    gpio_set_level(LED_CALIB_GPIO, 0);
    gpio_set_level(LED_RUN_GPIO, 0);
}

void update_leds(void)
{
    static uint32_t blink_counter = 0;

    switch (robot_state) {
        case ROBOT_STATE_IDLE:
            gpio_set_level(LED_CALIB_GPIO, is_calibrated ? 1 : 0);
            gpio_set_level(LED_RUN_GPIO, 0);
            break;

        case ROBOT_STATE_CALIBRATING:
            blink_counter++;
            gpio_set_level(LED_CALIB_GPIO, (blink_counter / 12) % 2);
            gpio_set_level(LED_RUN_GPIO, 0);
            break;

        case ROBOT_STATE_RUNNING:
            gpio_set_level(LED_CALIB_GPIO, 1);
            gpio_set_level(LED_RUN_GPIO, 1);
            break;
    }
}

/* =========================================================================
 * 8. SENSOR PROCESSING FUNCTIONS
 * ========================================================================= */
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

    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_min[i] = 4095;
        sensor_max[i] = 0;
    }

    ESP_LOGI(TAG, "Sensors initialized.");
}

void read_sensors(int *raw)
{
    for (int i = 0; i < 6; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc1_channels[i], &raw[i]));
    }
    for (int i = 0; i < 2; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, adc2_channels[i], &raw[6 + i]));
    }
}

void update_calibration(const int *raw)
{
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (raw[i] < sensor_min[i]) sensor_min[i] = raw[i];
        if (raw[i] > sensor_max[i]) sensor_max[i] = raw[i];
    }
}

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

uint8_t get_sensors_binary(const int *calibrated)
{
    uint8_t binary = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (calibrated[i] > 500) {
            binary |= (1 << i);
        }
    }
    return binary;
}

/* =========================================================================
 * 9. PID COMPUTATION FUNCTION
 * ========================================================================= */
void pid_reset(pid_t *pid)
{
    pid->integral            = 0.0f;
    pid->prev_error          = 0.0f;
    pid->filtered_derivative = 0.0f;
    pid->first_sample        = true;
}

float pid_compute(pid_t *pid, float error, float dt)
{
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit)  pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    float raw_derivative = 0.0f;
    if (!pid->first_sample && dt > 0.0f) {
        raw_derivative = (error - pid->prev_error) / dt;
    }
    pid->first_sample = false;
    pid->prev_error   = error;

    pid->filtered_derivative = (DERIVATIVE_FILTER_ALPHA * raw_derivative)
                             + ((1.0f - DERIVATIVE_FILTER_ALPHA) * pid->filtered_derivative);

    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * pid->filtered_derivative);
}

/* =========================================================================
 * 10. MOTOR DRIVER CONTROL FUNCTIONS
 * ========================================================================= */
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

    gpio_set_level(STBY, 0);
    gpio_set_level(AIN1, 0);
    gpio_set_level(AIN2, 0);
    gpio_set_level(BIN1, 0);
    gpio_set_level(BIN2, 0);
}

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

int clamp_duty(int duty)
{
    if (duty > MAX_DUTY) duty = MAX_DUTY;
    if (duty < 0) duty = 0;
    return duty;
}

void set_motor_speed(ledc_channel_t channel, int duty)
{
    duty = clamp_duty(duty);

    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);
}

void drive(int left_speed, int right_speed)
{
    gpio_set_level(AIN1, 1);
    gpio_set_level(AIN2, 0);
    gpio_set_level(BIN1, 1);
    gpio_set_level(BIN2, 0);

    set_motor_speed(CHANNEL_A, left_speed);
    set_motor_speed(CHANNEL_B, right_speed);
}

/* =========================================================================
 * 11. SINGLE-BUTTON STATE MACHINE HANDLER
 * ========================================================================= */
void handle_single_button(void)
{
    if (btn_pressed) {
        btn_pressed = false;

        if (gpio_get_level(BTN_SINGLE_GPIO) == 0) {

            if (robot_state == ROBOT_STATE_IDLE) {
                robot_state = ROBOT_STATE_CALIBRATING;
                gpio_set_level(STBY, 0);

                for (int i = 0; i < NUM_SENSORS; i++) {
                    sensor_min[i] = 4095;
                    sensor_max[i] = 0;
                }
                ESP_LOGW(TAG, ">>> STATE: CALIBRATING (Sweep sensors across line) <<<");
            }
            else if (robot_state == ROBOT_STATE_CALIBRATING) {
                robot_state = ROBOT_STATE_IDLE;
                is_calibrated = true;
                ESP_LOGW(TAG, ">>> STATE: IDLE (Calibration Saved! Press again to RUN) <<<");
            }
            else if (robot_state == ROBOT_STATE_RUNNING) {
                robot_state = ROBOT_STATE_IDLE;
                junction_probing_active = false;
                gpio_set_level(STBY, 0);
                ESP_LOGW(TAG, ">>> STATE: IDLE (Robot Stopped) <<<");
            }
            else {
                robot_state = ROBOT_STATE_IDLE;
                gpio_set_level(STBY, 0);
            }
        }
    }
}

void handle_run_trigger_from_idle(void)
{
    if (btn_pressed && robot_state == ROBOT_STATE_IDLE && is_calibrated) {
        btn_pressed = false;
        if (gpio_get_level(BTN_SINGLE_GPIO) == 0) {
            robot_state = ROBOT_STATE_RUNNING;
            junction_probing_active = false;
            pid_reset(&line_pid);
            gpio_set_level(STBY, 1);
            ESP_LOGW(TAG, ">>> STATE: RUNNING STARTED <<<");
        }
    }
}

void app_main(void)
{
    sensors_init();
    buttons_init();
    leds_init();
    motor_gpio_init();
    motor_pwm_init();

    int raw[NUM_SENSORS];
    int calibrated[NUM_SENSORS];
    uint32_t log_counter = 0;

    ESP_LOGI(TAG, "System Ready! Press BTN (GPIO 4) to Start Calibration.");

    while (1) {
        if (robot_state == ROBOT_STATE_IDLE && is_calibrated) {
            handle_run_trigger_from_idle();
        } else {
            handle_single_button();
        }

        update_leds();
        read_sensors(raw);

        // --- STATE 1: CALIBRATING ---
        if (robot_state == ROBOT_STATE_CALIBRATING) {
            update_calibration(raw);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // --- STATE 2: RUNNING ---
        if (robot_state == ROBOT_STATE_RUNNING) {
            get_calibrated_values(raw, calibrated);
            float position = get_weighted_position(calibrated);
            uint8_t binary = get_sensors_binary(calibrated);

            int binary_count = 0;
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (binary & (1 << i)) binary_count++;
            }

            int64_t now_us = esp_timer_get_time();

            /* -------------------------------------------------
             * TRIGGER CHECK FOR JUNCTION
             * ------------------------------------------------- */
            if (!junction_probing_active &&
                ((now_us - last_junction_time_us) > ((int64_t)JUNCTION_COOLDOWN_MS * 1000))) {

                if (is_left_junction_pattern(binary)) {
                    junction_probing_active = true;
                    junction_probe_start_us = now_us;
                    last_junction_time_us   = now_us;

                    ESP_LOGW(TAG, ">>> JUNCTION CANDIDATE DETECTED -> STARTING STRAIGHT PROBE <<<");
                }
            }

            /* -------------------------------------------------
             * MODE A: UNCONDITIONAL STRAIGHT DRIVE PROBE (PID OFF)
             * ------------------------------------------------- */
            if (junction_probing_active) {

                // Drive strictly straight past the junction stem
                drive(JUNCTION_STRAIGHT_SPEED, JUNCTION_STRAIGHT_SPEED);

                int64_t elapsed_ms = (now_us - junction_probe_start_us) / 1000;

                ESP_LOGI(TAG, "PROBE t=%lld ms | bin=%c%c%c%c%c%c%c%c | cnt=%d",
                         elapsed_ms,
                         (binary & (1 << 7)) ? '1' : '0',
                         (binary & (1 << 6)) ? '1' : '0',
                         (binary & (1 << 5)) ? '1' : '0',
                         (binary & (1 << 4)) ? '1' : '0',
                         (binary & (1 << 3)) ? '1' : '0',
                         (binary & (1 << 2)) ? '1' : '0',
                         (binary & (1 << 1)) ? '1' : '0',
                         (binary & (1 << 0)) ? '1' : '0',
                         binary_count);

                // Check if straight probe duration has completed
                if (elapsed_ms >= JUNCTION_STRAIGHT_DRIVE_MS) {

                    // Evaluate straight line using pattern matching + count + center mask
                    bool straight_line_present = check_straight_line_exists(binary, binary_count);

                    if (straight_line_present) {
                        ESP_LOGW(TAG, ">>> CLASSIFICATION: T-JUNCTION / CROSS (Straight Line Exists) <<<");
                    } else {
                        ESP_LOGW(TAG, ">>> CLASSIFICATION: L-JUNCTION (Dead End Straight -> Mandatory Left Turn) <<<");
                    }

                    // Reset probe state and re-enable PID line following
                    junction_probing_active = false;
                    pid_reset(&line_pid);
                    ESP_LOGI(TAG, ">>> PID ON - REJOINING LINE <<<");
                }
            }
            /* -------------------------------------------------
             * MODE B: NORMAL CLOSED-LOOP PID LINE FOLLOWING
             * ------------------------------------------------- */
            else {
                float correction = pid_compute(&line_pid, position, 0.02f);

                int left_speed  = BASE_SPEED - (int)correction;
                int right_speed = BASE_SPEED + (int)correction;

                drive(left_speed, right_speed);

                if (++log_counter >= 5) {
                    ESP_LOGI(TAG, "pos: %6.2f | corr: %6.2f | L:%4d R:%4d | bin: %c%c%c%c%c%c%c%c | cnt: %d",
                             position, correction, clamp_duty(left_speed), clamp_duty(right_speed),
                             (binary & (1 << 7)) ? '1' : '0',
                             (binary & (1 << 6)) ? '1' : '0',
                             (binary & (1 << 5)) ? '1' : '0',
                             (binary & (1 << 4)) ? '1' : '0',
                             (binary & (1 << 3)) ? '1' : '0',
                             (binary & (1 << 2)) ? '1' : '0',
                             (binary & (1 << 1)) ? '1' : '0',
                             (binary & (1 << 0)) ? '1' : '0',
                             binary_count);
                    log_counter = 0;
                }
            }
        } 
        else {
            gpio_set_level(STBY, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}