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
#include "include/event.h"
#include "include/nvs_op.h"
#include "include/util.h"

#include <stdio.h>
#include <string.h>

#define TAG "weight_task"

#define WEIGHT_JUMP_PAUSE_DEFAULT 5

typedef struct {
    // weight state machine
    enum weight_task_fsm now_fsm;
    enum weight_task_fsm pre_fsm;

    SemaphoreHandle_t cali_data_mutex;
} weight_task_data_t;

static weight_task_data_t task_data;

static char *weight_fsm_name[] = {"start", "standby", "jump", "bigjump",
                                  "postevent"};

weight_task_cb w_task_cb;

// static function prototype
static void weight_post_evnet(esp_event_loop_handle_t event_loop, float weight,
                              rawdata_eventid eventid);
static float weight_calculate_internal(float adc, float weight_coefficeint);
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

static void weight_post_evnet(esp_event_loop_handle_t event_loop, float weight,
                              rawdata_eventid eventid) {

    weight_take_photo_event_t evt;
    evt.eventid = eventid;
    evt.weight_g = (int)weight;
    evt.pir_val = w_task_cb.pir_level;

    ESP_LOGW(TAG,
             "weight take photo event post weight[%d g] pir[%d] eventid[%d]",
             evt.weight_g, evt.pir_val, evt.eventid);
    esp_event_post_to(event_loop, LULUPET_EVENT_BASE, LULUPET_EVENT_TAKE_PHOTO,
                      &evt, sizeof(evt), pdMS_TO_TICKS(1000));
}

static float weight_calculate_internal(float adc, float weight_coefficeint) {
    float weight = weight_calculate(adc, weight_coefficeint);

    xSemaphoreTake(task_data.cali_data_mutex, portMAX_DELAY);
    weight = adc * weight_coefficeint;
    if (w_task_cb.cali_cb.cali_val_num != 0) {
        for (uint8_t i = 0; i < w_task_cb.cali_cb.cali_val_num; ++i) {
            weight_cali_val *cali_val = &w_task_cb.cali_cb.cali_val[i];
            if (weight > cali_val->range_floor &&
                weight <= cali_val->range_ceiling) {
                // y = a * x + b;
                weight = cali_val->slope * weight + cali_val->offset;
            }
        }
    }
    xSemaphoreGive(task_data.cali_data_mutex);

    return weight;
}

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
        w_task_cb.now_weight = weight_calculate_internal(
            w_task_cb.latest_adc, w_task_cb.weight_coefficent);

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
            w_task_cb.ref_weight = weight_calculate_internal(
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
    task_data.now_fsm = WEIGHT_TASK_STAT_STANDBY;
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
    task_data.now_fsm = WEIGHT_TASK_STAT_JUMP;
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
    board_led_ctrl(LED_TYPE_IR, true); // turn on IR led
    w_task_cb.period_ms = w_task_cb.bugjump_period_ms;
    w_task_cb.period_cnt = 0;
    weight_post_evnet(w_task_cb.evt_loop, w_task_cb.now_weight,
                      RAWDATA_EVENTID_CAT_IN);

    // fsm change
    task_data.now_fsm = WEIGHT_TASK_STAT_BIGJUMP;
}

static void weight_fsm_check_postevent(void) {
    ++w_task_cb.postevnet_cnt;
    ESP_LOGI(TAG, "postevnet_cnt: %d", w_task_cb.postevnet_cnt);
    if (w_task_cb.postevnet_chk == w_task_cb.postevnet_num) {
        ESP_LOGI(TAG, "cat weight diff: %.3f g",
                 w_task_cb.now_weight -
                     w_task_cb.ref_weight); // record adc_weight - ref_weight
        ESP_LOGI(TAG, "cat druing time: %d ms",
                 w_task_cb.period_cnt *
                     w_task_cb.bugjump_period_ms); // record cat during time
        weight_post_evnet(w_task_cb.evt_loop,
                          (w_task_cb.now_weight - w_task_cb.ref_weight),
                          RAWDATA_EVENTID_CAT_OUT);
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
    task_data.now_fsm = WEIGHT_TASK_STAT_POSTEVENT;
}

static void weight_task(void *pvParameter) {
    // inital cb
    task_data.now_fsm = task_data.pre_fsm = WEIGHT_TASK_STAT_START;
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
    task_data.cali_data_mutex = xSemaphoreCreateMutex();
    weight_load_nvs_cali_val();

    for (;;) {
        // measure weight
        weight_get();

        // state machine
        switch (task_data.now_fsm) {
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

        if (task_data.now_fsm != task_data.pre_fsm) {
            ESP_LOGW(TAG, "fsm: [%s]->[%s]",
                     weight_fsm_name[task_data.pre_fsm],
                     weight_fsm_name[task_data.now_fsm]);
            task_data.pre_fsm = task_data.now_fsm;
        }

        vTaskDelay(w_task_cb.period_ms / portTICK_PERIOD_MS);
    }
}

float weight_calculate(float adc, float weight_coefficeint) {
    return adc * weight_coefficeint;
}

int weight_get_now_weight_int(void) { return (int)w_task_cb.now_weight; }

float weight_get_ref_weight(void) {

    float weight;

    weight = w_task_cb.ref_weight;

    ESP_LOGI(TAG, "ref_adc: %.3f", w_task_cb.ref_adc);
    ESP_LOGI(TAG, "ref_weight: %.3f g", weight);

    return weight;
}

float weight_get_now_weight(void) {
    float weight;

    weight = w_task_cb.now_weight;

    ESP_LOGI(TAG, "ref_adc: %.3f", w_task_cb.latest_adc);
    ESP_LOGI(TAG, "ref_weight: %.3f g", weight);

    return weight;
}

void weight_set_cali_val(uint32_t range_floor, uint32_t range_ceiling,
                         float slope, float offset) {
    ESP_LOGD(TAG, "range_floor: %d", range_floor);
    ESP_LOGD(TAG, "range_ceilling: %d", range_ceiling);
    ESP_LOGD(TAG, "slope: %.3f", slope);
    ESP_LOGD(TAG, "offset: %.3f", offset);
    xSemaphoreTake(task_data.cali_data_mutex, portMAX_DELAY);

    if (w_task_cb.cali_cb.cali_val_num >= WEIGHT_CALI_DATA_BUF) {
        ESP_LOGE(TAG, "over calibartion data buffer: %d", WEIGHT_CALI_DATA_BUF);
        xSemaphoreGive(task_data.cali_data_mutex);
        return;
    }

    weight_cali_val *cali_val =
        &w_task_cb.cali_cb.cali_val[w_task_cb.cali_cb.cali_val_num++];
    cali_val->range_floor = range_floor;
    cali_val->range_ceiling = range_ceiling;
    cali_val->slope = slope;
    cali_val->offset = offset;
    xSemaphoreGive(task_data.cali_data_mutex);
    weight_list_cali_val_ram();
}

void weight_list_cali_val_ram(void) {

    xSemaphoreTake(task_data.cali_data_mutex, portMAX_DELAY);
    weight_list_cali_val(&w_task_cb.cali_cb);
    xSemaphoreGive(task_data.cali_data_mutex);
}

void weight_list_cali_val(weight_cali_cb *cb) {
    char range_str[32];
    char formula_str[32];

    if (cb->cali_val_num == 0) {
        ESP_LOGW(TAG, "weight calibration data is empty");
        return;
    }

    for (uint8_t i = 0; i < cb->cali_val_num; ++i) {
        weight_cali_val *cali_val = &cb->cali_val[i];
        snprintf(range_str, sizeof(range_str), "%d <= x <= %d",
                 cali_val->range_floor, cali_val->range_ceiling);
        snprintf(formula_str, sizeof(formula_str), "\"y= %.3f * x + (%.3f)\"",
                 cali_val->slope, cali_val->offset);

        // printf("weight_cali[%d]: %32s %32s\n", i, range_str, formula_str);
        ESP_LOGI(TAG, "weight_cali[%d]: %32s %32s", i, range_str, formula_str);
    }
}

esp_err_t weight_load_nvs_cali_val(void) {
    esp_err_t esp_err;

    xSemaphoreTake(task_data.cali_data_mutex, portMAX_DELAY);
    esp_err = nvs_cali_read_weight_clai_cb(&w_task_cb.cali_cb);
    xSemaphoreGive(task_data.cali_data_mutex);

    return esp_err;
}

esp_err_t weight_clear_nvs_cali_val(void) {
    esp_err_t esp_err;

    xSemaphoreTake(task_data.cali_data_mutex, portMAX_DELAY);
    w_task_cb.cali_cb.cali_val_num = 0;
    esp_err = nvs_cali_reset_weight_clai_cb();
    xSemaphoreGive(task_data.cali_data_mutex);

    return esp_err;
}

esp_err_t weight_save_nvs_cali_val(void) {
    esp_err_t esp_err;

    xSemaphoreTake(task_data.cali_data_mutex, portMAX_DELAY);
    esp_err = nvs_cali_write_weight_clai_cb(&w_task_cb.cali_cb);
    xSemaphoreGive(task_data.cali_data_mutex);

    return esp_err;
}

void app_weight_main(esp_event_loop_handle_t event_loop) {
    ESP_LOGD(TAG, "app_weight_main start");

    w_task_cb.evt_loop = event_loop;

    xTaskCreate(&weight_task, "weight_task", 4096, NULL, 4, NULL);
}
