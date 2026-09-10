#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "MOTOR_TEST";

// Pin Definitions
#define PWMA GPIO_NUM_16 
#define AIN1 GPIO_NUM_17 
#define AIN2 GPIO_NUM_18

#define STBY GPIO_NUM_23   

#define PWMB GPIO_NUM_19  
#define BIN1 GPIO_NUM_21 
#define BIN2 GPIO_NUM_22   

// PWM Configuration Definitions
#define LEDC_MODE             LEDC_LOW_SPEED_MODE
#define LEDC_TIMER            LEDC_TIMER_0
#define LEDC_DUTY_RES         LEDC_TIMER_10_BIT // 0 to 1023 duty cycle
#define LEDC_FREQUENCY        5000              // 5 kHz frequency
#define LEDC_CHANNEL_MOTOR_A  LEDC_CHANNEL_0
#define LEDC_CHANNEL_MOTOR_B  LEDC_CHANNEL_1

// Safe helper to set Motor A speed with boundary protection
void set_motor_a_speed(uint32_t duty)
{
    if (duty > 1023) duty = 1023; // Clamp value to bit-resolution maximum
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A);
}

// Safe helper to set Motor B speed with boundary protection
void set_motor_b_speed(uint32_t duty)
{
    if (duty > 1023) duty = 1023;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B);
}

// Function to initialize directional GPIO pins securely
void gpio_init(void)
{
    uint64_t pin_mask = (1ULL << AIN1) | (1ULL << AIN2) | 
                        (1ULL << BIN1) | (1ULL << BIN2) | 
                        (1ULL << STBY);

    gpio_config_t io_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Keep driver disabled (STBY = 0) and motor pins LOW on startup
    gpio_set_level(STBY, 0);
    gpio_set_level(AIN1, 0);
    gpio_set_level(AIN2, 0);
    gpio_set_level(BIN1, 0);
    gpio_set_level(BIN2, 0);
}

// Function to initialize LEDC PWM module
void pwm_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t pwm_a_channel = {
        .gpio_num   = PWMA,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL_MOTOR_A,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&pwm_a_channel);

    ledc_channel_config_t pwm_b_channel = {
        .gpio_num   = PWMB,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL_MOTOR_B,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&pwm_b_channel);
}

// Master initialization routine
void motor_system_init(void)
{
    gpio_init();
    pwm_init();
    
    // Enable motor driver only after all PWM & GPIO channels are fully active
    gpio_set_level(STBY, 1);
    ESP_LOGI(TAG, "Motor system initialized successfully.");
}

void app_main(void)
{
    // Single clean call initializes all hardware safely
    motor_system_init();

    while (1) {
        // Forward at 50% Speed (~512 duty)
        ESP_LOGI(TAG, "Forward - 50%% Speed");
        gpio_set_level(AIN1, 1);
        gpio_set_level(AIN2, 0);
        gpio_set_level(BIN1, 1);
        gpio_set_level(BIN2, 0);

        set_motor_a_speed(512);
        set_motor_b_speed(512);
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Forward at Full Speed (1023 duty)
        ESP_LOGI(TAG, "Forward - 100%% Speed");
        set_motor_a_speed(1023);
        set_motor_b_speed(1023);
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Stop
        ESP_LOGI(TAG, "Stop");
        set_motor_a_speed(0);
        set_motor_b_speed(0);
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Reverse Speed Ramping (0% to 100%)
        ESP_LOGI(TAG, "Reverse - Speed Ramping Up");
        gpio_set_level(AIN1, 0);
        gpio_set_level(AIN2, 1);
        gpio_set_level(BIN1, 0);
        gpio_set_level(BIN2, 1);

        for (int speed = 0; speed <= 1023; speed += 100) {
            set_motor_a_speed(speed);
            set_motor_b_speed(speed);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        // Stop
        ESP_LOGI(TAG, "Stop");
        set_motor_a_speed(0);
        set_motor_b_speed(0);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}