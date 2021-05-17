#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "include/app_weight.h"
#include "include/board_driver.h"
#include "include/util.h"

#include <stdio.h>
#include <string.h>

#define TAG "weight_task"

#define WEIGHT_JUMP_PAUSE_DEFAULT 5

static char *weight_fsm_name[] = {"start", "standby", "jump", "bigjump",
                                  "postevent"};

weight_task_cb w_task_cb;

// static function prototype
static void weight_get(void);
static void weight_fsm_check_start(void);
static void weight_fsm_check_standby(void);
static void weight_fsm_goto_standby(void);
static void weight_fsm_check_jump(void);
static void weight_fsm_goto_jump(void);
static void weight_fsm_check_bigjump(void);
static void weight_fsm_goto_bigjump(void);
static void weight_fsm_check_postevent(void);
static void weight_fsm_goto_postevent(void);

static void weight_get(void) {
    if (w_task_cb.weight_pause_times) {
        ESP_LOGI(TAG, "weight pause[%d]", w_task_cb.weight_pause_times);
        --w_task_cb.weight_pause_times;
        return;
    }

#if (!FUNC_WEIGHT_FAKE)
    unsigned int tmp_adc;
    if (i2c_mcp3221_readADC(I2C_MASTER_NUM, &tmp_adc) == ESP_OK) {
        // get latest adc
        w_task_cb.latest_adc = 1.0 * tmp_adc;
        w_task_cb.now_weight =
            weight_calculate(w_task_cb.latest_adc, w_task_cb.weight_coefficent);

        // get reference adc
        if (w_task_cb.ref_adc_exec) {
            w_task_cb.ref_adc_sum += tmp_adc;

            if (w_task_cb.ring_buffer_loop) {
                w_task_cb.ref_adc_sum -=
                    w_task_cb.ring_buffer
                        [w_task_cb.ring_buffer_idx]; // minus orignal value
                w_task_cb.ring_buffer[w_task_cb.ring_buffer_idx] =
                    tmp_adc; // save adc data into buffer
                w_task_cb.ref_adc =
                    1.0 * w_task_cb.ref_adc_sum /
                    (1.0 *
                     WEIGHT_TASK_BUFFER_SIZE); // calculate reference weight
            } else {
                w_task_cb.ring_buffer[w_task_cb.ring_buffer_idx] =
                    tmp_adc; // save adc data into buffer
                w_task_cb.ref_adc = 1.0 * w_task_cb.ref_adc_sum /
                                    (1.0 * (w_task_cb.ring_buffer_idx +
                                            1)); // calculate reference weight
            }

            ++w_task_cb.ring_buffer_idx;
            if (w_task_cb.ring_buffer_idx == WEIGHT_TASK_BUFFER_SIZE) {
                w_task_cb.ring_buffer_idx = 0;
                w_task_cb.ring_buffer_loop = true;
            }
            w_task_cb.ref_weight = weight_calculate(
                w_task_cb.ref_adc, w_task_cb.weight_coefficent);
        }

        ESP_LOGD(TAG, "now adc [%.2f] ref [%.2f]", w_task_cb.latest_adc,
                 w_task_cb.ref_adc);
    } else
        ESP_LOGE(TAG, "can not read weight adc");
#endif
    ESP_LOGD(TAG, "now [%.2f g] ref [%.2f g]", w_task_cb.now_weight,
             w_task_cb.ref_weight);
}

static void weight_fsm_check_start(void) { weight_fsm_goto_standby(); }

static void weight_fsm_check_standby(void) {
#if (!FUNC_WEIGHT_FAKE)
    if (w_task_cb.ring_buffer_loop) {
#endif
        if ((w_task_cb.now_weight - w_task_cb.ref_weight) >
            w_task_cb.active_weight) {
            weight_fsm_goto_jump();
        }
#if (!FUNC_WEIGHT_FAKE)
    }
#endif
}

static void weight_fsm_goto_standby(void) {
    // action
    board_led_ctrl(LED_TYPE_BD_W, false); // turn off W led
    board_led_ctrl(LED_TYPE_IR, false);   // turn off IR led
    w_task_cb.weight_pause_times = 0;
    w_task_cb.period_ms = w_task_cb.standby_period_ms;
    w_task_cb.ref_adc_exec = true;
    w_task_cb.ring_buffer_loop = false;
    w_task_cb.ring_buffer_idx = 0;
    w_task_cb.ref_adc_sum = 0;
    w_task_cb.jump_to_bigjump_cnt = 0;
    w_task_cb.jump_to_standby_cnt = 0;
    // fsm change
    w_task_cb.now_stat = WEIGHT_TASK_STAT_STANDBY;
}

static void weight_fsm_check_jump(void) {
    if (!w_task_cb.weight_pause_times) {
#if (!FUNC_WEIGHT_FAKE)
        w_task_cb.pir_level = gpio_get_level(GPIO_INPUT_PIR);
#endif

        ++w_task_cb.jump_cnt;

        if ((w_task_cb.now_weight - w_task_cb.ref_weight) <
                (0.5 * w_task_cb.cat_weight) &&
            !w_task_cb.pir_level) {
            w_task_cb.jump_to_bigjump_cnt = 0;
            ++w_task_cb.jump_to_standby_cnt;
            ESP_LOGI(TAG, "jump_to_standby_cnt: %d",
                     w_task_cb.jump_to_standby_cnt);
            if (w_task_cb.jump_to_standby_cnt == w_task_cb.jump_to_standby_num)
                weight_fsm_goto_standby();
        } else if ((w_task_cb.now_weight - w_task_cb.ref_weight) >=
                       (0.5 * w_task_cb.cat_weight) &&
                   w_task_cb.pir_level) {
            w_task_cb.jump_to_standby_cnt = 0;
            ++w_task_cb.jump_to_bigjump_cnt;
            ESP_LOGI(TAG, "jump_to_bigjump_cnt: %d",
                     w_task_cb.jump_to_bigjump_cnt);
            if (w_task_cb.jump_to_bigjump_cnt == w_task_cb.jump_to_bigjump_num)
                weight_fsm_goto_bigjump();
        } else {
            w_task_cb.jump_to_bigjump_cnt = 0;
            w_task_cb.jump_to_standby_cnt = 0;
        }

        ESP_LOGI(TAG, "jump_cnt: %d", w_task_cb.jump_cnt);
        if (w_task_cb.jump_cnt == w_task_cb.jump_num)
            weight_fsm_goto_standby();
    }
}

static void weight_fsm_goto_jump(void) {
    // action
    w_task_cb.weight_pause_times = w_task_cb.jump_pause_times;
    w_task_cb.period_ms = w_task_cb.jump_period_ms;
    w_task_cb.ref_adc_exec = false;
    w_task_cb.jump_to_standby_num = w_task_cb.jump_to_standby_chk;
    w_task_cb.jump_to_standby_cnt = 0;
    w_task_cb.jump_to_bigjump_num = w_task_cb.jump_to_bigjump_chk;
    w_task_cb.jump_to_bigjump_cnt = 0;
    w_task_cb.jump_num = w_task_cb.jump_chk;
    w_task_cb.jump_cnt = 0;
    // fsm change
    w_task_cb.now_stat = WEIGHT_TASK_STAT_JUMP;
}

static void weight_fsm_check_bigjump(void) {
    if ((w_task_cb.now_weight - w_task_cb.ref_weight) <
        w_task_cb.active_weight) {
        weight_fsm_goto_postevent();
    }
    ++w_task_cb.period_cnt;
}

static void weight_fsm_goto_bigjump(void) {
    // action
    board_led_ctrl(LED_TYPE_IR, true);         // turn on IR led
    ESP_LOGI(TAG, "TODO: take bigjump photo"); // send event to photo task
    w_task_cb.period_ms = w_task_cb.bugjump_period_ms;
    w_task_cb.period_cnt = 0;

    // fsm change
    w_task_cb.now_stat = WEIGHT_TASK_STAT_BIGJUMP;
}

static void weight_fsm_check_postevent(void) {
    ++w_task_cb.postevnet_cnt;
    ESP_LOGI(TAG, "postevnet_cnt: %d", w_task_cb.postevnet_cnt);
    if (w_task_cb.postevnet_chk == w_task_cb.postevnet_num) {
        ESP_LOGI(TAG, "TODO: take postevent photo"); // send event to photo task
        ESP_LOGI(TAG, "cat weight diff: %.3f g",
                 w_task_cb.now_weight -
                     w_task_cb.ref_weight); // record adc_weight - ref_weight
        ESP_LOGI(TAG, "cat druing time: %d ms",
                 w_task_cb.period_cnt *
                     w_task_cb.bugjump_period_ms); // record cat during time
        weight_fsm_goto_standby();
    }
}

static void weight_fsm_goto_postevent(void) {
    // action
    board_led_ctrl(LED_TYPE_BD_W, true); // turn on W led
    w_task_cb.postevnet_num = w_task_cb.postevnet_chk;
    w_task_cb.postevnet_cnt = 0;
    w_task_cb.period_ms = w_task_cb.postevent_period_ms;
    // fsm change
    w_task_cb.now_stat = WEIGHT_TASK_STAT_POSTEVENT;
}

static void weight_task(void *pvParameter) {
    // inital cb
    w_task_cb.now_stat = w_task_cb.pre_stat = WEIGHT_TASK_STAT_START;
    w_task_cb.ring_buffer_idx = 0;
    w_task_cb.ring_buffer_loop = false;
    w_task_cb.ref_adc_sum = 0;
    w_task_cb.weight_coefficent = WEIGHT_COEFFICIENT;
    w_task_cb.standby_period_ms = WEIGHT_STANDBY_PERIOD;
    w_task_cb.jump_period_ms = WEIGHT_JUMP_PERIOD;
    w_task_cb.bugjump_period_ms = WEIGHT_BIGJUMP_PERIOD;
    w_task_cb.postevent_period_ms = WEIGHT_POSTEVENT_PERIOD;
    w_task_cb.period_ms = w_task_cb.standby_period_ms;
    w_task_cb.jump_pause_times = WEIGHT_JUMP_PAUSE_DEFAULT;
    w_task_cb.weight_pause_times = 0;
    w_task_cb.ref_adc_exec = true;
    w_task_cb.cat_weight = WEIGHT_CAT_VAL;
    w_task_cb.active_weight = WEIGHT_ACTIVE_VAL;
    w_task_cb.jump_to_standby_chk = WEIGHT_JUMP_TO_STANDBY_CHECK_TIMES;
    w_task_cb.jump_to_bigjump_chk = WEIGHT_JUMP_TO_BIGJUMP_CHECK_TIMES;
    w_task_cb.jump_chk = WEIGHT_JUMP_CHECK_TIMES;
    w_task_cb.postevnet_chk = WEIGHT_POSTEVENT_CHECK_TIMES;

    for (;;) {
        // measure weight
        weight_get();

        // state machine
        switch (w_task_cb.now_stat) {
        case WEIGHT_TASK_STAT_START:
        default:
            weight_fsm_check_start();
            break;

        case WEIGHT_TASK_STAT_STANDBY:
            weight_fsm_check_standby();
            break;

        case WEIGHT_TASK_STAT_JUMP:
            weight_fsm_check_jump();
            break;

        case WEIGHT_TASK_STAT_BIGJUMP:
            weight_fsm_check_bigjump();
            break;

        case WEIGHT_TASK_STAT_POSTEVENT:
            weight_fsm_check_postevent();
        }

        if (w_task_cb.now_stat != w_task_cb.pre_stat) {
            ESP_LOGW(TAG, "fsm: [%s]->[%s]",
                     weight_fsm_name[w_task_cb.pre_stat],
                     weight_fsm_name[w_task_cb.now_stat]);
            w_task_cb.pre_stat = w_task_cb.now_stat;
        }

        vTaskDelay(w_task_cb.period_ms / portTICK_PERIOD_MS);
    }
}

float weight_calculate(float adc, float weight_coefficeint) {
    // TODO: use calibration data to calculate
    return adc * weight_coefficeint;
}

int weight_get_latest(void) { return (int)w_task_cb.now_weight; }

void app_weight_main(void) {
    ESP_LOGD(TAG, "app_weight_main start");

    xTaskCreate(&weight_task, "weight_task", 4096, NULL, 4, NULL);
}
