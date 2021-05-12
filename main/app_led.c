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

static void led_task(void *pvParameter) {

    for (;;) {
        vTaskDelay(LED_TASK_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

void app_led_main(void) {
    ESP_LOGI(TAG, "app_led_main start");

    xTaskCreate(&led_task, "led_task", 4096, NULL, 4, NULL);
}
