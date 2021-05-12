#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
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
        vTaskDelay(LED_TASK_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

void app_led_main(void) {
    ESP_LOGI(TAG, "app_led_main start");

    xTaskCreate(&led_task, "led_task", 4096, NULL, 4, NULL);
}
