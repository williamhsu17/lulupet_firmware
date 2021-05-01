#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "include/board_driver.h"
#include "include/util.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define TAG "weight_task"

#define WEIGHT_TASK_BUFFER_SIZE 100
#define WEIGHT_ACTIVE_VAL 200.0 // unit:g
#define WEIGHT_CAT_VAL 2000.0   // unit:g
#define WEIGHT_COEFFICIENT 20000.0 / 4096.0
#define WEIGHT_STANDBY_PERIOD 10   // ms
#define WEIGHT_JUMP_PERIOD 1000    // ms
#define WEIGHT_BIGJUMP_PERIOD 1000 // ms
#define WEIGHT_JUMP_TO_STANDBY_CHECK_TIMES 25
#define WEIGHT_JUMP_TO_BIGJUMP_CHECK_TIMES 4

#define WEIGHT_JUMP_PAUSE_DEFAULT 5

enum weight_task_fsm {
    WEIGHT_TASK_STAT_STANDBY = 0,
    WEIGHT_TASK_STAT_JUMP,
    WEIGHT_TASK_STAT_BIGJUMP,
};

static char *weight_fsm_name[] = {
    "standby",
    "jump",
    "bigjump",
};

typedef struct {
    // weight state machine
    enum weight_task_fsm now_stat;
    enum weight_task_fsm pre_stat;

    // weight data
    unsigned int ring_buffer[WEIGHT_TASK_BUFFER_SIZE]; // TODO: the buffer just
                                                       // for verifying
    uint32_t weight_pause_times;
    unsigned int ref_adc_sum;
    bool ring_buffer_loop;
    uint32_t ring_buffer_idx;
    float ref_adc;
    float latest_adc;
    float weight_coefficent;

    float ref_weight; // unit:g
    float now_weight; // unit:g

    bool ref_adc_exec;

    float active_weight; // param can be set by user
    float cat_weight;    // param can be set by user

    uint8_t jump_to_standby_num;
    uint8_t jump_to_standby_cnt;
    uint8_t jump_to_standby_chk; // param can be set by user
    uint8_t jump_to_bigjump_num;
    uint8_t jump_to_bigjump_cnt;
    uint8_t jump_to_bigjump_chk; // param can be set by user

    uint32_t jump_pause_times;  // param can be set by user
    uint32_t standby_period_ms; // param can be set by user
    uint32_t jump_period_ms;    // param can be set by user
    uint32_t bugjump_period_ms; // param can be set by user

    uint32_t period_ms;
} weight_task_cb;

static weight_task_cb w_task_cb;

// static function prototype
static float weight_calculate(float adc);
static void weight_get(void);
static void weight_fsm_check_standby(void);
static void weight_fsm_goto_standby(void);
static void weight_fsm_check_jump(void);
static void weight_fsm_goto_jump(void);
static void weight_fsm_check_bigjump(void);
static void weight_fsm_goto_bigjump(void);

static float weight_calculate(float adc) {
    // TODO: use calibration data to calculate
    return adc * w_task_cb.weight_coefficent;
}

static void weight_get(void) {
    unsigned int tmp_adc;

    if (w_task_cb.weight_pause_times) {
        ESP_LOGW(TAG, "weight pause[%d]", w_task_cb.weight_pause_times);
        --w_task_cb.weight_pause_times;
        return;
    }

    if (i2c_mcp3221_readADC(I2C_MASTER_NUM, &tmp_adc) == ESP_OK) {
        // get latest adc
        w_task_cb.latest_adc = 1.0 * tmp_adc;
        w_task_cb.now_weight = weight_calculate(w_task_cb.latest_adc);

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
            w_task_cb.ref_weight = weight_calculate(w_task_cb.ref_adc);
        }

        ESP_LOGI(TAG, "now [%.2f] ref [%.2f]", w_task_cb.now_weight,
                 w_task_cb.ref_weight);
    }
}

static void weight_fsm_check_standby(void) {
    if (w_task_cb.ring_buffer_loop) {
        if ((w_task_cb.now_weight - w_task_cb.ref_weight) >
            w_task_cb.active_weight) {
            weight_fsm_goto_jump();
        }
    }
}

static void weight_fsm_goto_standby(void) {
    // action
    w_task_cb.weight_pause_times = 0;
    w_task_cb.period_ms = w_task_cb.standby_period_ms;
    w_task_cb.ref_adc_exec = true;
    w_task_cb.ring_buffer_loop = false;
    w_task_cb.ring_buffer_idx = 0;
    // fsm change
    w_task_cb.now_stat = WEIGHT_TASK_STAT_STANDBY;
}

static void weight_fsm_check_jump(void) {
    if (!w_task_cb.weight_pause_times) {
        int sensor_pir = gpio_get_level(GPIO_INPUT_PIR);

        if ((w_task_cb.now_weight) < (0.5 * w_task_cb.cat_weight) &&
            !sensor_pir) {
            w_task_cb.jump_to_bigjump_cnt = 0;
            ++w_task_cb.jump_to_standby_cnt;
            ESP_LOGI(TAG, "jump_to_standby_cnt: %d",
                     w_task_cb.jump_to_standby_cnt);
            if (w_task_cb.jump_to_standby_cnt == w_task_cb.jump_to_standby_num)
                weight_fsm_goto_standby();
        } else if ((w_task_cb.now_weight) >= (0.5 * w_task_cb.cat_weight) &&
                   sensor_pir) {
            w_task_cb.jump_to_standby_cnt = 0;
            ++w_task_cb.jump_to_bigjump_cnt;
            ESP_LOGI(TAG, "jump_to_bigjump_cnt: %d",
                     w_task_cb.jump_to_bigjump_cnt);
            if (w_task_cb.jump_to_bigjump_cnt == w_task_cb.jump_to_bigjump_num)
                weight_fsm_goto_bigjump();
        }
    }
}

static void weight_fsm_goto_jump(void) {
    // action
    w_task_cb.weight_pause_times = w_task_cb.jump_pause_times;
    w_task_cb.period_ms = w_task_cb.jump_period_ms;
    w_task_cb.ref_adc_exec = false;
    w_task_cb.jump_to_standby_num = w_task_cb.jump_to_standby_chk;
    w_task_cb.jump_to_bigjump_num = w_task_cb.jump_to_bigjump_chk;
    // fsm change
    w_task_cb.now_stat = WEIGHT_TASK_STAT_JUMP;
}

static void weight_fsm_check_bigjump(void) {
    // TODO
}

static void weight_fsm_goto_bigjump(void) {
    // TODO
    // action
    ESP_LOGI(TAG, "TODO: take photo"); // send event to photo task

    // fsm change
    w_task_cb.now_stat = WEIGHT_TASK_STAT_BIGJUMP;
}

static void weight_task(void *pvParameter) {
    // inital cb
    w_task_cb.now_stat = w_task_cb.pre_stat = WEIGHT_TASK_STAT_STANDBY;
    w_task_cb.ring_buffer_idx = 0;
    w_task_cb.ring_buffer_loop = false;
    w_task_cb.ref_adc_sum = 0;
    w_task_cb.weight_coefficent = WEIGHT_COEFFICIENT;
    w_task_cb.standby_period_ms = WEIGHT_STANDBY_PERIOD;
    w_task_cb.jump_period_ms = WEIGHT_JUMP_PERIOD;
    w_task_cb.bugjump_period_ms = WEIGHT_BIGJUMP_PERIOD;
    w_task_cb.period_ms = w_task_cb.standby_period_ms;
    w_task_cb.jump_pause_times = WEIGHT_JUMP_PAUSE_DEFAULT;
    w_task_cb.weight_pause_times = 0;
    w_task_cb.ref_adc_exec = true;
    w_task_cb.cat_weight = WEIGHT_CAT_VAL;
    w_task_cb.active_weight = WEIGHT_ACTIVE_VAL;
    w_task_cb.jump_to_standby_chk = WEIGHT_JUMP_TO_STANDBY_CHECK_TIMES;
    w_task_cb.jump_to_bigjump_chk = WEIGHT_JUMP_TO_BIGJUMP_CHECK_TIMES;

    for (;;) {
        // measure weight
        weight_get();

        // state machine
        switch (w_task_cb.now_stat) {
        case WEIGHT_TASK_STAT_STANDBY:
        default:
            weight_fsm_check_standby();
            break;

        case WEIGHT_TASK_STAT_JUMP:
            weight_fsm_check_jump();
            break;

        case WEIGHT_TASK_STAT_BIGJUMP:
            weight_fsm_check_bigjump();
            break;
        }

        if (w_task_cb.now_stat != w_task_cb.pre_stat) {
            ESP_LOGW(TAG, "state [%s]->[%s]",
                     weight_fsm_name[w_task_cb.pre_stat],
                     weight_fsm_name[w_task_cb.now_stat]);
            w_task_cb.pre_stat = w_task_cb.now_stat;
        }

        vTaskDelay(w_task_cb.period_ms / portTICK_PERIOD_MS);
    }
}

void app_weight_main(void) {
    ESP_LOGD(TAG, "app_weight_main start");

    xTaskCreate(&weight_task, "weight_task", 4096, NULL, 4, NULL);
}
