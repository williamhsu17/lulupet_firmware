#ifndef _APP_WEIGHT_H_
#define _APP_WEIGHT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_event.h"
#include "util.h"
#include <stdbool.h>

#define WEIGHT_COEFFICIENT 30000.0 / 4096.0
#define WEIGHT_CALI_DATA_BUF 10

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
    uint8_t version;
    // standby
    uint32_t standby_period_ms;
    float standby_active_weight_g;
    // jump
    uint32_t jump_period_ms;
    uint32_t jump_pause_times;
    uint8_t jump_chk;
    uint8_t jump_to_standby_chk;
    uint8_t jump_to_bigjump_chk;
    float jump_cat_weight_g;
    // bigjump
    uint32_t bigjump_period_ms;
    // postevent
    uint32_t postevent_period_ms;
    uint8_t postevnet_chk;
} weight_conf_ver1_t;

typedef struct {
    // weight data
    volatile float ref_adc;
    volatile float latest_adc;
    volatile float ref_weight_g; // unit:g
    volatile float now_weight_g; // unit:g
    int pir_level;

    weight_conf_ver1_t *conf;

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
 * @retval ref_weight_g
 */
float weight_get_ref_weight(void);

/**
 * @brief Output now_weight_g. unit: g.
 *
 * @retval now_weight_g
 */
float weight_get_now_weight(void);

/**
 * @brief Get the latest weight. unit: g.
 *
 * @retval now_weight_g
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
