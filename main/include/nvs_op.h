#ifndef _NVS_OP_H_
#define _NVS_OP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "app_weight.h"
#include "esp_err.h"

esp_err_t nvs_cali_init(void);
esp_err_t nvs_cali_read_weight_clai_cb(weight_cali_cb *cali);
esp_err_t nvs_cali_write_weight_clai_cb(weight_cali_cb *cali);
esp_err_t nvs_cali_reset_weight_clai_cb(void);

esp_err_t nvs_init(void);
int32_t nvs_read_wifichecked(void);
esp_err_t nvs_read_wifi_config(wifi_config_t *sta_config);
esp_err_t nvs_write_wifi_val(int32_t set_value, wifi_config_t *wifi_config);
esp_err_t nvs_reset_wifi_val(void);

esp_err_t nvs_read_lid_token(char *lid, uint32_t lid_len, char *token,
                             uint32_t token_len);
esp_err_t nvs_write_lid_token(char *lid, uint32_t lid_len, char *token,
                              uint32_t token_len);
esp_err_t nvs_reset_lid_token(void);

esp_err_t nvs_read_weight_conf(void *conf, int version);
esp_err_t nvs_write_weight_conf(void *conf, int version);
esp_err_t nvs_reset_weight_conf(void);

esp_err_t nvs_read_rtc_timeval(struct timeval *time_val);
esp_err_t nvs_write_rtc_timeval(struct timeval time_val);
esp_err_t nvs_reset_rtc_timeval(void);

esp_err_t nvs_reset(void);

#ifdef __cplusplus
}
#endif

#endif // _NVS_OP_H_