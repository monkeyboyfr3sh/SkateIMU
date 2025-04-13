/**
 *  MIT License
 *  Copyright (c) 2021 Christian Castaneda <github.com/christianjc>
 * 
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 * 
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 * 
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 * 
 * 
 *          https://www.bosch-sensortec.com/bst/products/all_products/bno055
 *          Reference Datasheet: BST_BNO055_DS000_14 (consulted in January 2018)
 * 
*/

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>
#include "esp_bno055.h"

static const char *TAG = "MAIN";

/** Task Handles **/
static TaskHandle_t xUpdateVectorTask = NULL;
//static TaskHandle_t xUpdateFrameTask = NULL;

/** Queue Handles **/
static QueueHandle_t xVectorQueue = NULL;

static void apply_deadband(double *vec, double threshold) {
    for (int i = 0; i < 3; ++i) {
        if (fabs(vec[i]) < threshold) {
            vec[i] = 0.0;
        }
    }
}

static void get_vector_callback(void *arg)
{
    double accel_raw[3];
    double gravity[3];
    double accel[3];
    double quat[4];

    double velocity[3] = {0};
    double position[3] = {0};
    int64_t prev_time = 0;
    int64_t last_print_time = 0;

    const int print_interval_ms = 50;
    const double accel_deadband = 0.2;
    const double velocity_deadband = 0.05;
    const double velocity_decay = 0.80;

    while (1)
    {
        get_vector(VECTOR_ACCELEROMETER, accel_raw);
        get_vector(VECTOR_GRAVITY, gravity);
        get_quat(quat);
        int64_t timestamp = esp_timer_get_time();

        for (int i = 0; i < 3; ++i) {
            accel[i] = accel_raw[i] - gravity[i];
        }

        apply_deadband(accel, accel_deadband);

        if (prev_time == 0) {
            prev_time = timestamp;
            last_print_time = timestamp;
            continue;
        }

        double dt = (timestamp - prev_time) / 1000000.0;
        prev_time = timestamp;

        for (int i = 0; i < 3; ++i) {
            velocity[i] += accel[i] * dt;
            velocity[i] *= velocity_decay;
        }

        double velocity_filtered[3] = { velocity[0], velocity[1], velocity[2] };
        apply_deadband(velocity_filtered, velocity_deadband);

        for (int i = 0; i < 3; ++i) {
            position[i] += velocity_filtered[i] * dt;
        }

        if ((timestamp - last_print_time) >= (print_interval_ms * 1000)) {
            ESP_LOGI(TAG, "PROC,%lld,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                timestamp,
                accel[0], accel[1], accel[2],
                velocity_filtered[0], velocity_filtered[1], velocity_filtered[2],
                position[0], position[1], position[2],
                quat[0], quat[1], quat[2], quat[3]);

            last_print_time = timestamp;
        }
    }
}

void app_main(void)
{
    printf("Hello World!! \n");

    static const gpio_num_t OutputPin = GPIO_NUM_7;
    static const gpio_num_t ButtonPin = GPIO_NUM_0;

    // Set GPIO 7 high
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << OutputPin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(OutputPin, 1);

    // Configure GPIO0 as input with pull-up
    gpio_config_t btn_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << ButtonPin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&btn_conf);

    vTaskDelay(pdMS_TO_TICKS(100));

    // Create vector queue
    xVectorQueue = xQueueCreate(10, (sizeof(double) * 3));

    esp_err_t err = bno055_begin_i2c(OPERATION_MODE_IMUPLUS);
    if (err != ESP_OK) {
        printf("ERROR: could not initiate i2c communication\n");
        return;
    }

    // Start the sensor reading task
    xTaskCreate(get_vector_callback, "get_vector", 2048 * 4, NULL, 11, &xUpdateVectorTask);

    // Main loop: check for button press
    while (1) {
        if (gpio_get_level(ButtonPin) == 0) {
            // Button press detected
            int64_t press_start = esp_timer_get_time(); // microseconds

            // Wait until button is released
            while (gpio_get_level(ButtonPin) == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            int64_t press_duration = esp_timer_get_time() - press_start; // microseconds

            printf("Button press duration: %lld ms\n", press_duration / 1000);

            // Pause task
            if (xUpdateVectorTask) {
                vTaskSuspend(xUpdateVectorTask);
            }

            // Decide calibration type based on press duration
            if (press_duration > 1500000) { // 1.5 seconds threshold for long press
                printf("Long press detected. Calibrating sensor from scratch...\n");
                err = calibrate_sensor(true);
            } else {
                printf("Short press detected. Loading calibration profile...\n");
                err = calibrate_sensor_from_saved_profile();
            }

            if (err == ESP_OK) {
                printf("Calibration successful.\n");
            } else {
                printf("Calibration failed with error: %d\n", err);
            }

            // Resume task
            if (xUpdateVectorTask) {
                vTaskResume(xUpdateVectorTask);
            }

            // Debounce delay
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Polling interval
    }
}