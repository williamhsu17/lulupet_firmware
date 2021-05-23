#ifndef _APP_WEIGHT_H_
#define _APP_WEIGHT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_event.h"
#include "util.h"
#include <stdbool.h>

#define WEIGHT_TASK_BUFFER_SIZE 255
#define WEIGHT_ACTIVE_VAL 200.0 // unit:g
#define WEIGHT_CAT_VAL 2000.0   // unit:g
#define WEIGHT_COEFFICIENT 30000.0 / 4096.0
#define WEIGHT_STANDBY_PERIOD 10     // ms
#define WEIGHT_JUMP_PERIOD 1000      // ms
#define WEIGHT_BIGJUMP_PERIOD 1000   // ms
#define WEIGHT_POSTEVENT_PERIOD 1000 // ms
#define WEIGHT_JUMP_TO_STANDBY_CHECK_TIMES 4
#define WEIGHT_JUMP_TO_BIGJUMP_CHECK_TIMES 4
#define WEIGHT_JUMP_CHECK_TIMES 25
#define WEIGHT_POSTEVENT_CHECK_TIMES 1
#define WEIGHT_CALI_DATA_BUF 10

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
    weight_cali_val cali_val[WEIGHT_CALI_DATA_BUF];
} weight_cali_cb;

typedef struct {
    // weight data
    volatile float ref_adc;
    volatile float latest_adc;
    volatile float ref_weight; // unit:g
    volatile float now_weight; // unit:g
    int pir_level;

    uint32_t weight_pause_times; // param can be set by user
    float active_weight;         // param can be set by user
    float cat_weight;            // param can be set by user

    uint8_t jump_to_standby_chk; // param can be set by user
    uint8_t jump_to_bigjump_chk; // param can be set by user
    uint8_t jump_chk;            // param can be set by user
    uint8_t postevnet_chk;       // param can be set by user

    uint32_t jump_pause_times;    // param can be set by user
    uint32_t standby_period_ms;   // param can be set by user
    uint32_t jump_period_ms;      // param can be set by user
    uint32_t bugjump_period_ms;   // param can be set by user
    uint32_t postevent_period_ms; // param can be set by user

    weight_cali_cb cali_cb; // param can be set by nvs
} weight_task_cb;

typedef struct {
    rawdata_eventid eventid;
    int weight_g;
    int pir_val;
} weight_take_photo_event_t;

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
 * @brief Output ref_weight weight. unit: g.
 *
 * @retval ref_weight
 */
float weight_get_ref_weight(void);

/**
 * @brief Output now_weight. unit: g.
 *
 * @retval now_weight
 */
float weight_get_now_weight(void);

/**
 * @brief Get the latest weight. unit: g.
 *
 * @retval now_weight
 */
int weight_get_now_weight_int(void);

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
 * @brief List the weight calibated value in the ram.
 *
 * @retval none
 */
void weight_list_cali_val_ram(void);

/**
 * @brief List the weight calibated value in the ram.
 *
 * @param range_floor[in] Would like to display structure of weight_cali_cb.
 *
 * @retval none
 */
void weight_list_cali_val(weight_cali_cb *cb);

/**
 * @brief Load weight calibration data from nvs into ram
 *
 * @retval esp_err_t
 */
esp_err_t weight_load_nvs_cali_val(void);

/**
 * @brief Clear weight calibration data at nvs and ram
 *
 * @retval esp_err_t
 */
esp_err_t weight_clear_nvs_cali_val(void);

/**
 * @brief Save weight calibration data from ram into nvs
 *
 * @retval esp_err_t
 */
esp_err_t weight_save_nvs_cali_val(void);

/**
 * @brief Start weight_task service
 *
 * @param event_loop[in] esp32 event loop handler.
 *
 * @retval none
 */
void app_weight_main(esp_event_loop_handle_t event_loop);

#ifdef __cplusplus
}
#endif

#endif // end of _APP_WEIGHT_H_
