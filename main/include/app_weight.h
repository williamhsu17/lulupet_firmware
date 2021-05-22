#ifndef _APP_WEIGHT_H_
#define _APP_WEIGHT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_event.h"
#include "util.h"
#include <stdbool.h>

#define WEIGHT_TASK_BUFFER_SIZE 100
#define WEIGHT_ACTIVE_VAL 200.0 // unit:g
#define WEIGHT_CAT_VAL 2000.0   // unit:g
#define WEIGHT_COEFFICIENT 20000.0 / 4096.0
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

typedef struct { // range_floor <= range < range_ceiling
    uint32_t range_floor;
    uint32_t range_ceiling;
    float slope;
    float offset;
} weight_cali_val;

typedef struct { // range_floor <= range < range_ceiling
    uint8_t cali_val_num;
    weight_cali_val cali_val[10];
} weight_cali_cb;

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

    uint8_t jump_num;
    uint8_t jump_cnt;
    uint8_t jump_chk; // param can be set by user

    uint8_t postevnet_num;
    uint8_t postevnet_cnt;
    uint8_t postevnet_chk; // param can be set by user

    uint32_t jump_pause_times;    // param can be set by user
    uint32_t standby_period_ms;   // param can be set by user
    uint32_t jump_period_ms;      // param can be set by user
    uint32_t bugjump_period_ms;   // param can be set by user
    uint32_t postevent_period_ms; // param can be set by user

    uint32_t period_ms;
    uint32_t period_cnt;

    int pir_level;

    SemaphoreHandle_t data_mutex;
    weight_cali_cb cali_cb;

    esp_event_loop_handle_t evt_loop;
} weight_task_cb;

typedef struct {
    rawdata_eventid eventid;
    int weight_g;
    int pir_val;
} weight_take_photo_event_t;

void app_weight_main(esp_event_loop_handle_t event_loop);

/**
 * @brief Get calculated weight. weight = adc * weight_coefficeint. unit: g.
 *
 * @param adc[in] adc value.
 * @param weight_coefficeint[in] coefficeint.
 *
 * @retval weight
 */
float weight_calculate(float adc, float weight_coefficeint);

/**
 * @brief Get the latest weight. unit: g.
 *
 * @retval weight
 */
int weight_get_latest(void);

/**
 * @brief Set the calibrated value.
 *
 * @param range_floor[in] calculated range of floor.
 * @param range_ceiling[in] calculated range of ceiling.
 * @param slope[in] calculated slope.
 * @param offset[in] calculated offset.
 *
 * @retval none
 */
void weight_set_cali_val(uint32_t range_floor, uint32_t range_ceiling,
                         float slope, float offset);

/**
 * @brief List the weight calibated value.
 *
 * @retval none
 */
void weight_list_cali_val(void);

int weight_clear_cali_val(void);

int weight_save_cali_val(void);

#ifdef __cplusplus
}
#endif

#endif // end of _APP_WEIGHT_H_
