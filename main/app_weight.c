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
#define WEIGHT_CONF_VERSION 1
#define WEIGHT_TASK_BUFFER_SIZE 255
#define WEIGHT_ACTIVE_VAL 200.0      // unit:g
#define WEIGHT_CAT_VAL 2000.0        // unit:g
#define WEIGHT_STANDBY_PERIOD 10     // ms
#define WEIGHT_JUMP_PERIOD 1000      // ms
#define WEIGHT_BIGJUMP_PERIOD 1000   // ms
#define WEIGHT_POSTEVENT_PERIOD 1000 // ms
#define WEIGHT_JUMP_TO_STANDBY_CHECK_TIMES 4
#define WEIGHT_JUMP_TO_BIGJUMP_CHECK_TIMES 4
#define WEIGHT_JUMP_CHECK_TIMES 25
#define WEIGHT_POSTEVENT_CHECK_TIMES 1

enum weight_task_fsm {
    WEIGHT_TASK_STAT_START = 0,
    WEIGHT_TASK_STAT_STANDBY,
    WEIGHT_TASK_STAT_JUMP,
    WEIGHT_TASK_STAT_BIGJUMP,
    WEIGHT_TASK_STAT_POSTEVENT,
};

typedef struct {
    // weight state machine
    enum weight_task_fsm now_fsm;
    enum weight_task_fsm pre_fsm;

    unsigned int ring_buffer[WEIGHT_TASK_BUFFER_SIZE]; // TODO: the buffer just
                                                       // for verifying
    bool ref_adc_exec;
    unsigned int ref_adc_sum;
    bool ring_buffer_loop;
    uint32_t ring_buffer_idx;
    float weight_coefficent;

    uint32_t weight_pause_times;
    uint8_t jump_num;
    uint8_t jump_cnt;
    uint8_t jump_to_standby_num;
    uint8_t jump_to_standby_cnt;
    uint8_t jump_to_bigjump_num;
    uint8_t jump_to_bigjump_cnt;
    uint8_t postevnet_num;
    uint8_t postevnet_cnt;

    uint32_t period_ms;
    uint32_t period_cnt;
    uint32_t big_jump_period_cnt;

    esp_event_loop_handle_t evt_loop;
    SemaphoreHandle_t cali_data_mutex;
} weight_task_data_t;

static weight_task_data_t task_data;

static char *weight_fsm_name[] = {"start", "standby", "jump", "bigjump",
                                  "postevent"};

weight_task_cb w_task_cb;

// static function prototype
static void weight_post_event(esp_event_loop_handle_t event_loop, float weight,
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
static void weight_data_init(weight_task_data_t *data);
static void weight_conf_init_v1(weight_conf_ver1_t *conf);
static esp_err_t weight_conf_init(weight_task_cb *task);
static void weight_task(void *pvParameter);

#if 0 // unused
static esp_err_t esp_err_print(esp_err_t esp_err, const char *file,
                               uint32_t line) {
    ESP_LOGE(TAG, "err: %s %s():L%d", esp_err_to_name(esp_err), file, line);
    return esp_err;
}
#endif

static void weight_post_event(esp_event_loop_handle_t event_loop, float weight,
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
    if (task_data.weight_pause_times) {
        ESP_LOGI(TAG, "weight pause[%d]", task_data.weight_pause_times);
        --task_data.weight_pause_times;
        return;
    }

#if (!FUNC_WEIGHT_FAKE)
    unsigned int tmp_adc;
    if (i2c_mcp3221_readADC(I2C_MASTER_NUM, &tmp_adc) == ESP_OK) {
        // get latest adc
        w_task_cb.latest_adc = 1.0 * tmp_adc;
        w_task_cb.now_weight_g = weight_calculate_internal(
            w_task_cb.latest_adc, task_data.weight_coefficent);

        // get reference adc
        if (task_data.ref_adc_exec) {
            task_data.ref_adc_sum += tmp_adc;

            if (task_data.ring_buffer_loop) {
                task_data.ref_adc_sum -=
                    task_data.ring_buffer
                        [task_data.ring_buffer_idx]; // minus orignal value
                task_data.ring_buffer[task_data.ring_buffer_idx] =
                    tmp_adc; // save adc data into buffer
                w_task_cb.ref_adc =
                    1.0 * task_data.ref_adc_sum /
                    (1.0 *
                     WEIGHT_TASK_BUFFER_SIZE); // calculate reference weight
            } else {
                task_data.ring_buffer[task_data.ring_buffer_idx] =
                    tmp_adc; // save adc data into buffer
                w_task_cb.ref_adc = 1.0 * task_data.ref_adc_sum /
                                    (1.0 * (task_data.ring_buffer_idx +
                                            1)); // calculate reference weight
            }

            ++task_data.ring_buffer_idx;
            if (task_data.ring_buffer_idx == WEIGHT_TASK_BUFFER_SIZE) {
                task_data.ring_buffer_idx = 0;
                task_data.ring_buffer_loop = true;
            }
            w_task_cb.ref_weight_g = weight_calculate_internal(
                w_task_cb.ref_adc, task_data.weight_coefficent);
        }

        ESP_LOGD(TAG, "now adc [%.2f] ref [%.2f]", w_task_cb.latest_adc,
                 w_task_cb.ref_adc);
    } else
        ESP_LOGE(TAG, "can not read weight adc");
#endif
    ESP_LOGD(TAG, "now [%.2f g] ref [%.2f g]", w_task_cb.now_weight_g,
             w_task_cb.ref_weight_g);
}

static void weight_fsm_check_start(void) {
#if (FUNC_TESTING_FW)
    return;
#else
    weight_fsm_goto_standby();
#endif
}

static void weight_fsm_check_standby(void) {
#if (!FUNC_WEIGHT_FAKE)
    if (task_data.ring_buffer_loop) {
#endif
        if ((w_task_cb.now_weight_g - w_task_cb.ref_weight_g) >
            w_task_cb.conf.standby_active_weight_g) {
            weight_fsm_goto_jump();
        }
#if (!FUNC_WEIGHT_FAKE)
    }
#endif
}

static void weight_fsm_goto_standby(void) {
    // action
    task_data.weight_pause_times = 0;
    task_data.period_ms = w_task_cb.conf.standby_period_ms;
    task_data.ref_adc_exec = true;
    task_data.ref_adc_sum = 0;
    task_data.ring_buffer_loop = false;
    task_data.ring_buffer_idx = 0;

    // fsm change
    task_data.now_fsm = WEIGHT_TASK_STAT_STANDBY;
}

static void weight_fsm_check_jump(void) {
    if (!task_data.weight_pause_times) {
#if (!FUNC_WEIGHT_FAKE)
        w_task_cb.pir_level = gpio_get_level(GPIO_INPUT_PIR);
#endif

        ++task_data.jump_cnt;

        if ((w_task_cb.now_weight_g - w_task_cb.ref_weight_g) <
                (0.5 * w_task_cb.conf.jump_cat_weight_g)
#if (FUNC_WEIGHT_JUMP_CHECK_PIR)
            && !w_task_cb.pir_level
#endif
        ) {
            task_data.jump_to_bigjump_cnt = 0;
            ++task_data.jump_to_standby_cnt;
            ESP_LOGI(TAG, "jump_to_standby_cnt: %d",
                     task_data.jump_to_standby_cnt);
            if (task_data.jump_to_standby_cnt == task_data.jump_to_standby_num)
                weight_fsm_goto_standby();
        } else if ((w_task_cb.now_weight_g - w_task_cb.ref_weight_g) >=
                       (0.5 * w_task_cb.conf.jump_cat_weight_g)
#if (FUNC_WEIGHT_JUMP_CHECK_PIR)
                   && w_task_cb.pir_level
#endif
        ) {
            task_data.jump_to_standby_cnt = 0;
            ++task_data.jump_to_bigjump_cnt;
            ESP_LOGI(TAG, "jump_to_bigjump_cnt: %d",
                     task_data.jump_to_bigjump_cnt);
            if (task_data.jump_to_bigjump_cnt == task_data.jump_to_bigjump_num)
                weight_fsm_goto_bigjump();
        } else {
            task_data.jump_to_bigjump_cnt = 0;
            task_data.jump_to_standby_cnt = 0;
        }

        ESP_LOGI(TAG, "jump_cnt: %d", task_data.jump_cnt);
        if (task_data.jump_cnt == task_data.jump_num)
            weight_fsm_goto_standby();
    }
}

static void weight_fsm_goto_jump(void) {
    // action
    task_data.weight_pause_times = w_task_cb.conf.jump_pause_times;
    task_data.period_ms = w_task_cb.conf.jump_period_ms;
    task_data.ref_adc_exec = false;
    task_data.jump_to_standby_num = w_task_cb.conf.jump_to_standby_chk;
    task_data.jump_to_standby_cnt = 0;
    task_data.jump_to_bigjump_num = w_task_cb.conf.jump_to_bigjump_chk;
    task_data.jump_to_bigjump_cnt = 0;
    task_data.jump_num = w_task_cb.conf.jump_chk;
    task_data.jump_cnt = 0;
    // fsm change
    task_data.now_fsm = WEIGHT_TASK_STAT_JUMP;
}

static void weight_fsm_check_bigjump(void) {
    if ((w_task_cb.now_weight_g - w_task_cb.ref_weight_g) <
        w_task_cb.conf.standby_active_weight_g) {
        weight_fsm_goto_postevent();
    }
    ++task_data.period_cnt;

    // every FUNC_WEIGHT_BIGJUMP_SEND_PHOTO_PERIOD_MS to send photo to backend
    ++task_data.big_jump_period_cnt;
    if ((task_data.big_jump_period_cnt * w_task_cb.conf.bigjump_period_ms) >
        FUNC_WEIGHT_BIGJUMP_SEND_PHOTO_PERIOD_MS) {
        task_data.big_jump_period_cnt = 0;
        weight_post_event(task_data.evt_loop, w_task_cb.now_weight_g,
                          RAWDATA_EVENTID_CAT_IN);
    }
}

static void weight_fsm_goto_bigjump(void) {
    // action
    task_data.period_ms = w_task_cb.conf.bigjump_period_ms;
    task_data.period_cnt = 0;
    task_data.big_jump_period_cnt = 0;
    weight_post_event(task_data.evt_loop, w_task_cb.now_weight_g,
                      RAWDATA_EVENTID_CAT_IN);

    // fsm change
    task_data.now_fsm = WEIGHT_TASK_STAT_BIGJUMP;
}

static void weight_fsm_check_postevent(void) {
    ++task_data.postevnet_cnt;
    ESP_LOGI(TAG, "postevnet_cnt: %d", task_data.postevnet_cnt);
    if (task_data.postevnet_cnt == task_data.postevnet_num) {
        ESP_LOGI(
            TAG, "cat weight diff: %.3f g",
            w_task_cb.now_weight_g -
                w_task_cb.ref_weight_g); // record adc_weight - ref_weight_g
        ESP_LOGI(
            TAG, "cat druing time: %d ms",
            task_data.period_cnt *
                w_task_cb.conf.bigjump_period_ms); // record cat during time
        weight_post_event(task_data.evt_loop,
                          (w_task_cb.now_weight_g - w_task_cb.ref_weight_g),
                          RAWDATA_EVENTID_CAT_OUT);
        weight_fsm_goto_standby();
    }
}

static void weight_fsm_goto_postevent(void) {
    // action
    task_data.postevnet_num = w_task_cb.conf.postevnet_chk;
    task_data.postevnet_cnt = 0;
    task_data.period_ms = w_task_cb.conf.postevent_period_ms;
    // fsm change
    task_data.now_fsm = WEIGHT_TASK_STAT_POSTEVENT;
}

static void weight_data_init(weight_task_data_t *data) {
    data->now_fsm = task_data.pre_fsm = WEIGHT_TASK_STAT_START;
    data->weight_coefficent = WEIGHT_COEFFICIENT;
    data->cali_data_mutex = xSemaphoreCreateMutex();
}

static void weight_conf_init_v1(weight_conf_ver1_t *conf) {
    conf->version = 1;
    conf->standby_period_ms = WEIGHT_STANDBY_PERIOD;
    conf->standby_active_weight_g = WEIGHT_ACTIVE_VAL;

    conf->jump_period_ms = WEIGHT_JUMP_PERIOD;
    conf->jump_pause_times = WEIGHT_JUMP_PAUSE_DEFAULT;
    conf->jump_to_standby_chk = WEIGHT_JUMP_TO_STANDBY_CHECK_TIMES;
    conf->jump_to_bigjump_chk = WEIGHT_JUMP_TO_BIGJUMP_CHECK_TIMES;
    conf->jump_chk = WEIGHT_JUMP_CHECK_TIMES;
    conf->jump_cat_weight_g = WEIGHT_CAT_VAL;

    conf->bigjump_period_ms = WEIGHT_BIGJUMP_PERIOD;

    conf->postevent_period_ms = WEIGHT_POSTEVENT_PERIOD;
    conf->postevnet_chk = WEIGHT_POSTEVENT_CHECK_TIMES;
}

static esp_err_t weight_conf_init(weight_task_cb *task) {
    // if conf OTA changing, that should be migrated into new config version
    if (WEIGHT_CONF_VERSION == 1) {
        if (nvs_read_weight_conf((void *)&task->conf, 1) != ESP_OK) {
            weight_conf_init_v1(&task->conf);
            nvs_write_weight_conf((void *)&task->conf, 1);
        }
    } else {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void weight_task(void *pvParameter) {
    // init task_data
    weight_data_init(&task_data);
    // init conf data.
    ESP_ERROR_CHECK(weight_conf_init(&w_task_cb));
    weight_load_nvs_cali_val();

    for (;;) {
        // measure weight
        weight_get();

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
            ESP_LOGW(TAG, "fsm: [%s]->[%s]", weight_fsm_name[task_data.pre_fsm],
                     weight_fsm_name[task_data.now_fsm]);
            task_data.pre_fsm = task_data.now_fsm;
        }

        vTaskDelay(task_data.period_ms / portTICK_PERIOD_MS);
    }
}

float weight_calculate(float adc, float weight_coefficeint) {
    return adc * weight_coefficeint;
}

int weight_get_now_weight_int(void) { return (int)w_task_cb.now_weight_g; }

float weight_get_ref_weight(void) {
    ESP_LOGI(TAG, "ref_adc: %.3f", w_task_cb.ref_adc);
    ESP_LOGI(TAG, "ref_weight_g: %.3f g", w_task_cb.ref_weight_g);
    return w_task_cb.ref_weight_g;
}

float weight_get_now_weight(void) {
    float weight;

    weight = w_task_cb.now_weight_g;

    ESP_LOGI(TAG, "ref_adc: %.3f", w_task_cb.latest_adc);
    ESP_LOGI(TAG, "now_weight_g: %.3f g", weight);

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

void weight_update_weight_conf_v1(weight_conf_ver1_t *cfg_v1) {
    memcpy(&w_task_cb.conf, cfg_v1, sizeof(weight_conf_ver1_t));
    nvs_write_weight_conf((void *)&w_task_cb.conf, 1);
    weight_dump_weight_conf_v1(cfg_v1);
}

void weight_dump_weight_conf_v1(weight_conf_ver1_t *cfg_v1) {
    if (cfg_v1 == NULL)
        return;
    ESP_LOGI(TAG, "weight_cfg:");
    ESP_LOGI(TAG, "\t version: %u", cfg_v1->version);
    ESP_LOGI(TAG, "\t standby_period_ms: %u", cfg_v1->standby_period_ms);
    ESP_LOGI(TAG, "\t standby_active_weight_g: %.3f",
             cfg_v1->standby_active_weight_g);
    ESP_LOGI(TAG, "\t jump_period_ms: %u", cfg_v1->jump_period_ms);
    ESP_LOGI(TAG, "\t jump_pause_times: %u", cfg_v1->jump_pause_times);
    ESP_LOGI(TAG, "\t jump_chk: %u", cfg_v1->jump_chk);
    ESP_LOGI(TAG, "\t jump_to_standby_chk: %u", cfg_v1->jump_to_standby_chk);
    ESP_LOGI(TAG, "\t jump_to_bigjump_chk: %u", cfg_v1->jump_to_bigjump_chk);
    ESP_LOGI(TAG, "\t jump_cat_weight_g: %.3f", cfg_v1->jump_cat_weight_g);
    ESP_LOGI(TAG, "\t bigjump_period_ms: %u", cfg_v1->bigjump_period_ms);
    ESP_LOGI(TAG, "\t postevent_period_ms: %u", cfg_v1->postevent_period_ms);
    ESP_LOGI(TAG, "\t postevnet_chk: %u", cfg_v1->postevnet_chk);
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

    task_data.evt_loop = event_loop;

    xTaskCreate(&weight_task, "weight_task", 4096, NULL, 4, NULL);
}
