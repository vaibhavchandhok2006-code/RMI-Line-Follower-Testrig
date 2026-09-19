/*
 * IR Sensor Array Test -- 8 channels with Button Calibration
 * 6 sensors on ADC1, 2 on ADC2
 * Button toggles Calibration Mode -> Normal Mode
 * Maps sensor range to 0-1000 for weighted position calculation.
 *
 * CMakeLists.txt needs: REQUIRES esp_adc driver
 */

#include <stdio.h>
#include <limits.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "SENSORS";

#define NUM_SENSORS 8
#define CALIB_BUTTON_GPIO GPIO_NUM_4   // Change to your actual button GPIO

// High value (1000) = ON the line, Low (0) = OFF the line
// Set to true if the line produces LOWER raw ADC values (e.g., black line on white surface)
#define INVERT_LINE_LOGIC false 

static const adc_channel_t adc1_channels[6] = {
    ADC_CHANNEL_0, // GPIO36 - S1
    ADC_CHANNEL_3, // GPIO39 - S2
    ADC_CHANNEL_4, // GPIO32 - S3
    ADC_CHANNEL_5, // GPIO33 - S4
    ADC_CHANNEL_6, // GPIO34 - S5
    ADC_CHANNEL_7  // GPIO35 - S6
};

static const adc_channel_t adc2_channels[2] = {
    ADC_CHANNEL_8, // GPIO25 - S7
    ADC_CHANNEL_9  // GPIO26 - S8
};

static const float sensor_weight[NUM_SENSORS] = {
    -52.5f, -37.5f, -22.5f, -7.5f, 7.5f, 22.5f, 37.5f, 52.5f
};

static adc_oneshot_unit_handle_t adc1_handle;
static adc_oneshot_unit_handle_t adc2_handle;

// Calibration tracking variables
static int sensor_min[NUM_SENSORS];
static int sensor_max[NUM_SENSORS];
static volatile bool is_calibrating = false;
static volatile bool button_pressed = false;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    button_pressed = true;
}

void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CALIB_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE   // Trigger on press (active low)
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(CALIB_BUTTON_GPIO, gpio_isr_handler, NULL);
}

void sensors_init(void)
{
    adc_oneshot_unit_init_cfg_t cfg1 = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg1, &adc1_handle));

    adc_oneshot_unit_init_cfg_t cfg2 = {
        .unit_id = ADC_UNIT_2,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg2, &adc2_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,   // 0 - 4095
        .atten    = ADC_ATTEN_DB_12,   // ~0 - 3.3V range
    };

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

    ESP_LOGI(TAG, "Sensor array & ADC units initialized.");
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

// Maps raw reading into range 0..1000 based on individual min/max bounds
void get_calibrated_values(const int *raw, int *calibrated)
{
    for (int i = 0; i < NUM_SENSORS; i++) {
        int range = sensor_max[i] - sensor_min[i];
        if (range <= 0) {
            calibrated[i] = 0;
            continue;
        }

        // Clamp values to min/max window
        int val = raw[i];
        if (val < sensor_min[i]) val = sensor_min[i];
        if (val > sensor_max[i]) val = sensor_max[i];

        // Map to 0-1000
        int mapped = ((val - sensor_min[i]) * 1000) / range;

        if (INVERT_LINE_LOGIC) {
            calibrated[i] = 1000 - mapped;
        } else {
            calibrated[i] = mapped;
        }
    }
}

float get_weighted_position(const int *calibrated)
{
    float num = 0.0f;
    float den = 0.0f;

    for (int i = 0; i < NUM_SENSORS; i++) {
        num += (float)calibrated[i] * sensor_weight[i];
        den += (float)calibrated[i];
    }

    if (den == 0.0f) return 0.0f; 
    return num / den;
}

void app_main(void)
{
    sensors_init();
    button_init();

    int raw[NUM_SENSORS];
    int calibrated[NUM_SENSORS];

    ESP_LOGI(TAG, "Press button to start Calibration Mode.");

    while (1) {
        // Simple software debouncing & toggle check
        if (button_pressed) {
            vTaskDelay(pdMS_TO_TICKS(50)); // Debounce delay
            if (gpio_get_level(CALIB_BUTTON_GPIO) == 0) {
                is_calibrating = !is_calibrating;
                if (is_calibrating) {
                    ESP_LOGW(TAG, ">>> CALIBRATION STARTED: Sweep sensor array across the line... <<<");
                    // Reset limits when starting fresh calibration run
                    for (int i = 0; i < NUM_SENSORS; i++) {
                        sensor_min[i] = 4095;
                        sensor_max[i] = 0;
                    }
                } else {
                    ESP_LOGW(TAG, ">>> CALIBRATION COMPLETED & SAVED <<<");
                    for (int i = 0; i < NUM_SENSORS; i++) {
                        ESP_LOGI(TAG, "Sensor %d: Min = %4d | Max = %4d", i + 1, sensor_min[i], sensor_max[i]);
                    }
                }
            }
            button_pressed = false;
        }

        read_sensors(raw);

        if (is_calibrating) {
            update_calibration(raw);
            ESP_LOGI(TAG, "[CALIBRATING] Raw -> S1:%4d S2:%4d S3:%4d S4:%4d S5:%4d S6:%4d S7:%4d S8:%4d",
                     raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6], raw[7]);
        } else {
            get_calibrated_values(raw, calibrated);
            float pos = get_weighted_position(calibrated);

            ESP_LOGI(TAG, "Calib -> S1:%4d S2:%4d S3:%4d S4:%4d S5:%4d S6:%4d S7:%4d S8:%4d | Pos: %7.2f",
                     calibrated[0], calibrated[1], calibrated[2], calibrated[3],
                     calibrated[4], calibrated[5], calibrated[6], calibrated[7], pos);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}