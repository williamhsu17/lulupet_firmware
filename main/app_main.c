/* ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in
 * which case, it is free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the "Software"), to deal in the
 * Software without restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "esp_event.h"
#include "esp_log.h"

//#include "include/app_camera.h"
#include "include/app_cmd.h"
#include "include/app_key.h"
#include "include/app_led.h"
#include "include/app_weight.h"
#include "include/app_wifi.h"
#include "include/board_driver.h"
#include "include/nvs_op.h"
#include "include/task_fs.h"
#include "include/timer_tick.h"
#include "include/util.h"

#define TAG "app_main"

static esp_event_loop_args_t service_event_loop_args = {
    .queue_size = 64,
    .task_name = "srv-evnloop",
    .task_priority = 4,
    .task_stack_size = 4096,
    .task_core_id = 0,
};

static esp_event_loop_handle_t service_event_loop;

static esp_err_t event_loop_init(void) {
    esp_err_t err;
    err = esp_event_loop_create(&service_event_loop_args, &service_event_loop);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

void app_main() {
    esp_err_t esp_err;
    bool init_pass = true;

    esp_reset_reason_t reset_reason = esp_reset_reason();

    if (reset_reason == ESP_RST_BROWNOUT) {
        ESP_LOGW(TAG, "reset_brownout");
        check_sys_det_low();
    }

    key_check_wakeup();

    if ((esp_err = board_init()) != ESP_OK) {
        ESP_LOGE(TAG, "err: %s L%d", esp_err_to_name(esp_err), __LINE__);
        init_pass = false;
    }

    if ((esp_err = esp_event_loop_create_default()) != ESP_OK) {
        ESP_LOGE(TAG, "err: %s L%d", esp_err_to_name(esp_err), __LINE__);
        init_pass = false;
    }

    if ((esp_err = event_loop_init()) != ESP_OK) {
        ESP_LOGE(TAG, "err: %s L%d", esp_err_to_name(esp_err), __LINE__);
        init_pass = false;
    }

    if ((esp_err = nvs_init()) != ESP_OK) {
        ESP_LOGE(TAG, "err: %s L%d", esp_err_to_name(esp_err), __LINE__);
        init_pass = false;
    }

    if ((esp_err = nvs_cali_init()) != ESP_OK) {
        ESP_LOGE(TAG, "err: %s L%d", esp_err_to_name(esp_err), __LINE__);
        init_pass = false;
    }

    if ((esp_err = timer_tick_init()) != ESP_OK) {
        ESP_LOGE(TAG, "err: %s L%d", esp_err_to_name(esp_err), __LINE__);
        init_pass = false;
    }

#if (FUNC_CMD_TASK)
    app_cmd_main(service_event_loop); // Init command line interface
#endif

    fs_task_start(service_event_loop); // Init fs task
    app_key_main(service_event_loop);
    app_led_main(); // Init led task
#if (FUNC_CMD_TASK)
    app_weight_main(service_event_loop); // Init weight task
#endif

    if (init_pass) {
        app_wifi_main(service_event_loop); // Init connect/httpc task
    } else {
        ESP_LOGE(TAG, "init failed L%d", __LINE__);
        board_set_rgb_led(true, false, false);
        while (1) {
            vTaskDelay(100);
        }
    }
}
