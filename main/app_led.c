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

static char *led_cmd_type[] = {
    "off", "w_solid", "r_solid", "b_solid", "g_solid", "b_1Hz", "r_1Hz",
};

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
            ESP_LOGI(TAG, "LED CMD: receive %s[%d]", led_cmd_type[led_cmd_get],
                     led_cmd_get);
            cmd_update = 1;
            if (led_cmd_run != led_cmd_get) {
                board_set_rgb_led(false, false, false);
                led_cmd_run = led_cmd_get;
            }

            repeat_count = 0;
        } else {
            cmd_update = 0;
        }
        switch (led_cmd_run) {
        case LED_ALL_OFF:
            if (cmd_update) {
                board_set_rgb_led(false, false, false);
            }
            break;
        case LED_WHITE_SOLID:
            if (cmd_update) {
                board_set_rgb_led(true, true, true);
            }
            break;
        case LED_RED_SOLID:
            if (cmd_update) {
                board_set_rgb_led(true, false, false);
            }
            break;
        case LED_BLUE_SOLID:
            if (cmd_update) {
                board_set_rgb_led(false, false, true);
            }
            break;
        case LED_GREEN_SOLID:
            if (cmd_update) {
                board_set_rgb_led(false, true, false);
            }
            break;
        case LED_BLUE_1HZ:
            if (repeat_count % 10 == 0) {
                repeat_count = 0;
                board_set_rgb_led(false, false, true);
            } else if (repeat_count % 5 == 0) {
                board_set_rgb_led(false, false, false);
            }
            repeat_count++;
            break;
        case LED_RED_1HZ:
            if (repeat_count % 10 == 0) {
                repeat_count = 0;
                board_set_rgb_led(true, false, false);
            } else if (repeat_count % 5 == 0) {
                board_set_rgb_led(false, false, false);
            }
            repeat_count++;
            break;
        case LED_GREEN_1HZ:
            if (repeat_count % 10 == 0) {
                repeat_count = 0;
                board_set_rgb_led(false, true, false);
            } else if (repeat_count % 5 == 0) {
                board_set_rgb_led(false, false, false);
            }
            repeat_count++;
            break;
        default:
            break;
        }

        vTaskDelay(LED_TASK_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

void app_led_main(void) {
    ESP_LOGI(TAG, "app_led_main start");

    xTaskCreate(&led_task, "led_task", 4096, NULL, 4, NULL);
}
