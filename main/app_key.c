#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "include/app_key.h"
#include "include/board_driver.h"
#include "include/event.h"
#include "include/timer_tick.h"
#include "include/util.h"

#define TAG "key_task"

#define KEY_TASK_PERIOD_MS 10
#define KEY_5000_MS 5000
#define KEY_3000_MS 3000
#define KEY_COOL_DOWN_MS 10000

typedef struct {
    esp_event_loop_handle_t evt_loop;
} key_task_config_t;

static key_task_config_t task_conf;

static char *key_event_name[] = {
    "press_over_5_sec",
    "press_2_times_within_3_sec",
};

static void key_post_evnet(esp_event_loop_handle_t event_loop,
                           key_event_type_e type) {

    key_loop_event_t event;
    event.key_event_type = type;

    ESP_LOGW(TAG, "key event post to: %s", key_event_name[type]);

    esp_event_post_to(event_loop, LULUPET_EVENT_BASE, LULUPET_EVENT_KEY, &event,
                      sizeof(event), pdMS_TO_TICKS(100));
}

static void key_task(void *pvParameter) {
    key_task_config_t *conf = (key_task_config_t *)pvParameter;
    bool key_press = false;
    uint8_t press_cnt = 0;
    uint32_t now_tick;
    uint32_t press_tick = 0;
    uint32_t release_tick = 0;
    uint32_t event_send_tick;
    uint32_t press_conti_ms;
    uint32_t press_interval_ms;
    uint32_t event_cool_down_ms = 0;

    for (;;) {
        key_press = board_get_key_status();
        now_tick = timer_tick_ms();

        if (event_cool_down_ms) {
            if (timer_tick_diff(event_send_tick, now_tick) >=
                event_cool_down_ms) {
                ESP_LOGW(TAG, "key event cool down stop");
                event_cool_down_ms = 0;
            } else {
                continue;
            }
        }

        if (key_press) {

            ESP_LOGD(TAG, "now_tick: %d", now_tick);

            // check key press 2 times within 3 seconds
            if (press_cnt != 0 && release_tick != 0) {
                ESP_LOGD(TAG, "latest release_tick: %d", release_tick);
                press_interval_ms = timer_tick_diff(release_tick, now_tick);
                ESP_LOGD(TAG, "press cnt: %u press_interval_ms: %u", press_cnt,
                         press_interval_ms);
                if (press_interval_ms < KEY_3000_MS) {
                    key_post_evnet(conf->evt_loop,
                                   KEY_EVENT_PRESS_2_TIMES_WITHIN_3_SEC);
                    // set event cool down time
                    event_cool_down_ms = KEY_3000_MS;
                    event_send_tick = now_tick;
                }
                press_tick = 0;
                release_tick = 0;
                press_cnt = 0;
            }

            if (press_tick == 0) {
                // record press_tick when first in
                press_tick = now_tick;
                ++press_cnt;
            } else {
                // check key press over 5 seconds
                press_conti_ms = timer_tick_diff(press_tick, now_tick);
                ESP_LOGD(TAG, "press_during_ms: %u ms", press_conti_ms);
                if (press_conti_ms > KEY_5000_MS) {
                    key_post_evnet(conf->evt_loop, KEY_EVENT_PRESS_OVER_5_SEC);
                    // set event cool down time
                    event_cool_down_ms = KEY_5000_MS;
                    event_send_tick = now_tick;
                }
            }
        } else {
            if (press_tick != 0) {
                // record release tick
                ESP_LOGD(TAG, "release_tick: %d", release_tick);
                release_tick = now_tick;
                press_tick = 0;
            }
        }

        vTaskDelay(KEY_TASK_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

void app_key_main(esp_event_loop_handle_t event_loop) {
    ESP_LOGI(TAG, "app_key_main start");

    task_conf.evt_loop = event_loop;

    xTaskCreate(&key_task, "key_task", 4096, (void *)&task_conf, 4, NULL);
}
