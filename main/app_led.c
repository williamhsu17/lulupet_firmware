#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "include/app_led.h"
#include "include/board_driver.h"
#include "include/util.h"

#include <stdio.h>
#include <string.h>

#define TAG "led_task"

#define LED_TASK_PERIOD_MS 100
#define LED_CMD_QUEUE_CREATE_RETRY 3

// Message Queue for LED task
QueueHandle_t led_cmd_que = NULL;

static void led_task(void *pvParameter) {
    int queue_retry = 0;
    unsigned int led_cmd_get = 0;
    unsigned int led_cmd_run = 0;
    int repeat_count = 0;
    int cmd_update = 0;
    BaseType_t xStatus;

    led_cmd_que = xQueueCreate(20, sizeof(unsigned int));
    while ((led_cmd_que = xQueueCreate(20, sizeof(unsigned int))) == NULL) {
        vTaskDelay(1000 / portTICK_PERIOD_MS); // wait for a second
        ESP_LOGW(TAG, "led_cmd_que re-create");
        if (queue_retry++ > LED_CMD_QUEUE_CREATE_RETRY) {
            ESP_LOGE(TAG, "led task creation failed");
            return;
        }
    }

    for (;;) {
        xStatus = xQueueReceive(led_cmd_que, &led_cmd_get, 0);
        if (xStatus == pdPASS) {
            ESP_LOGI(TAG, "LED CMD: receive %d", led_cmd_get);
            cmd_update = 1;
            led_cmd_run = led_cmd_get;
            repeat_count = 0;
        } else {
            cmd_update = 0;
        }
        switch (led_cmd_run) {
        case LED_ALL_OFF:
            if (cmd_update) {
                // LED ALL OFF
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x03,
                                     0x0); // set brightness 20mA (R)
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x04,
                                     0x0); // set brightness 20mA (B)
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x05,
                                     0x0); // set brightness 20mA (G)
            }
            break;
        case LED_WHITE_CNT:
            if (cmd_update) {
                // LED White
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x03,
                                     0xC8); // set brightness 20mA (R)
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x04,
                                     0xC8); // set brightness 20mA (B)
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x05,
                                     0xC8); // set brightness 20mA (G)
            }
            break;
        case LED_RED_CNT:
            if (cmd_update) {
                // LED Green
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x03,
                                     0xC8); // set brightness 20mA (R)
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x04,
                                     0x0); // set brightness 20mA (B)
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x05,
                                     0x0); // set brightness 20mA (G)
            }
            break;
        case LED_BLUE_CNT:
            if (cmd_update) {
                // LED Green
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x03,
                                     0x0); // set brightness 20mA (R)
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x04,
                                     0xC8); // set brightness 20mA (B)
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x05,
                                     0x0); // set brightness 20mA (G)
            }
            break;
        case LED_GREEN_CNT:
            if (cmd_update) {
                // LED Green
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x03,
                                     0x0); // set brightness 20mA (R)
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x04,
                                     0x0); // set brightness 20mA (B)
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x05,
                                     0xC8); // set brightness 20mA (G)
            }
            break;
        case LED_BLUE_2HZ:
            // ON cycle
            if (repeat_count % 10 == 0) {
                repeat_count = 0;
                // LED Blue ON
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x03,
                                     0x0); // set brightness 20mA (R)
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x04,
                                     0xC8); // set brightness 20mA (B)
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x05,
                                     0x0); // set brightness 20mA (G)
            }
            // OFF cycle
            else if (repeat_count % 5 == 0) {
                // LED Blue OFF
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x03,
                                     0x0); // set brightness 20mA (R)
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x04,
                                     0x0); // set brightness 20mA (B)
                i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x05,
                                     0x0); // set brightness 20mA (G)
            }
            repeat_count++;
            break;
        default:
            // Nothing...
            break;
        }

        vTaskDelay(LED_TASK_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

void app_led_main(void) {
    ESP_LOGI(TAG, "app_led_main start");

    xTaskCreate(&led_task, "led_task", 4096, NULL, 4, NULL);
}
