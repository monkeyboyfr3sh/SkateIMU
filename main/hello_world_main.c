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

#include "esp_bno055.h"

/** Task Handles **/
static TaskHandle_t xUpdateVectorTask = NULL;
//static TaskHandle_t xUpdateFrameTask = NULL;

/** Queue Handles **/
static QueueHandle_t xVectorQueue = NULL;

static void get_vector_callback(void *arg)
{
    double accel[3];
    double quat[4];
    //memset(vec, 0, 3);
    //esp_err_t err = ESP_OK;

    // VECTOR_ACCELEROMETER
    // VECTOR_MAGNETOMETER
    // VECTOR_GYROSCOPE
    // VECTOR_EULER
    // VECTOR_LINEARACCEL
    // VECTOR_GRAVITY
    for (;;)
    {
        // get_vector(VECTOR_EULER, vec);
        // print_vector(VECTOR_EULER, vec);
        // get_vector(VECTOR_GYROSCOPE, vec);
        // print_vector(VECTOR_GYROSCOPE, vec);
        // get_vector(VECTOR_LINEARACCEL, vec);
        // print_vector(VECTOR_LINEARACCEL, vec);
        // printf("LINEARACCEL,%.3f,%.3f,%.3f\n", vec[0], vec[1], vec[2]);  // heading, roll, pitch
        // get_vector(VECTOR_ACCELEROMETER, vec);
        // print_vector(VECTOR_ACCELEROMETER, vec);


        // Get linear acceleration (movement only)
        get_vector(VECTOR_LINEARACCEL, accel);

        // Get sensor fusion quaternion (orientation)
        get_quat(quat);

        // Format: LINEARACCEL,x,y,z,quat_w,quat_x,quat_y,quat_z
        printf("LINEARACCEL,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f\n",
            accel[0], accel[1], accel[2],
            quat[0], quat[1], quat[2], quat[3]);

        vTaskDelay(pdMS_TO_TICKS(50));
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
            printf("Button pressed: pausing and calibrating...\n");

            // Pause task
            if (xUpdateVectorTask) {
                vTaskSuspend(xUpdateVectorTask);
            }

            // Perform calibration
            err = calibrate_sensor(true);
            if (err == ESP_OK) {
                printf("Calibration complete.\n");
            } else {
                printf("Calibration failed with error: %d\n", err);
            }

            // Resume task
            if (xUpdateVectorTask) {
                vTaskResume(xUpdateVectorTask);
            }

            // Simple debounce / prevent repeated trigger
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Polling interval
    }
}