#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "driver/rtc_io.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"

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
#define SYS_DET_GOTO_DEEP_SLEEP_MS 1000

typedef struct {
    esp_event_loop_handle_t evt_loop;
} key_task_config_t;

static key_task_config_t task_conf;

RTC_DATA_ATTR struct timeval sleep_enter_time;

static char *key_event_name[] = {
    "none",
    "press_over_5_sec",
    "press_2_times_within_3_sec",
};

static void key_post_evnet(esp_event_loop_handle_t event_loop,
                           key_event_type_e type) {

    key_loop_event_t event;
    event.key_event_type = type;

    ESP_LOGW(TAG, "key event post to: %s", app_key_event_type_str(type));

    esp_event_post_to(event_loop, LULUPET_EVENT_BASE, LULUPET_EVENT_KEY, &event,
                      sizeof(event), pdMS_TO_TICKS(100));
}

static void key_task(void *pvParameter) {
    key_task_config_t *conf = (key_task_config_t *)pvParameter;
    bool key_press = false;
    uint8_t press_cnt = 0;
    uint8_t sys_det_cnt = 0;
    uint32_t now_tick;
    uint32_t press_tick = 0;
    uint32_t release_tick = 0;
    uint32_t event_send_tick;
    uint32_t press_conti_ms;
    uint32_t press_interval_ms;
    uint32_t event_cool_down_ms = 0;

    // set SYS_DET pin
    gpio_config_t io_conf;
    // disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    // bit mask of the pins that you want to set
    io_conf.pin_bit_mask = (1ULL << SYS_DET_PIN);
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);

    if (!rtc_gpio_is_valid_gpio(SYS_DET_PIN)) {
        ESP_LOGE(TAG, "GPIO %d is not an RTC IO", SYS_DET_PIN);
    }

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

        if (!gpio_get_level(SYS_DET_PIN)) {
            ++sys_det_cnt;
            if (sys_det_cnt ==
                (SYS_DET_GOTO_DEEP_SLEEP_MS / KEY_TASK_PERIOD_MS)) {
                // goto deep sleep mode
                ESP_LOGW(TAG,
                         "external power off %d msec, goto deep sleep mode",
                         SYS_DET_GOTO_DEEP_SLEEP_MS);
                const int ext_wakeup_pin_1 = SYS_DET_PIN;
                const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
                ESP_LOGW(TAG, "Enabling EXT1 wakeup on pins GPIO%d",
                         ext_wakeup_pin_1);
                esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask,
                                             ESP_EXT1_WAKEUP_ANY_HIGH);
                esp_deep_sleep_start();
            }
        } else {
            sys_det_cnt = 0;
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

char *app_key_event_type_str(key_event_type_e type) {
    return key_event_name[type];
}

void app_key_main(esp_event_loop_handle_t event_loop) {
    ESP_LOGI(TAG, "app_key_main start");

    task_conf.evt_loop = event_loop;

    xTaskCreate(&key_task, "key_task", 4096, (void *)&task_conf, 4, NULL);
}

void key_check_wakeup(void) {
    struct timeval now;
    struct tm timeinfo;
    char strftime_buf[64];

    // Set timezone to CST-8
    setenv("TZ", "CST-8", 1);
    tzset();

    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 +
                        (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    localtime_r((const time_t *)&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time is: %s", strftime_buf);

    switch (esp_sleep_get_wakeup_cause()) {
    case ESP_SLEEP_WAKEUP_EXT1: {
        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
        if (wakeup_pin_mask != 0) {
            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
            ESP_LOGW(TAG, "Wake up from GPIO %d", pin);
        } else {
            ESP_LOGW(TAG, "Wake up from GPIO");
        }
        break;
    }
    case ESP_SLEEP_WAKEUP_TIMER: {
        ESP_LOGW(TAG, "Wake up from timer. Time spent in deep sleep: %d ms",
                 sleep_time_ms);
        break;
    }
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
        ESP_LOGW(TAG, "Not a deep sleep reset");
    }
}
