#ifndef _NVS_OP_H_
#define _NVS_OP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

esp_err_t nvs_init(void);
int32_t nvs_read_wifichecked(void);
esp_err_t nvs_reset_wifi_val(void);
esp_err_t nvs_read_lid_token(char *lid, uint32_t lid_len, char *token,
                             uint32_t token_len);
esp_err_t nvs_read_wifi_config(wifi_config_t *sta_config);
esp_err_t nvs_write_wifi_val(int32_t set_value, wifi_config_t *wifi_config);
esp_err_t nvs_write_lid_token(char *lid, uint32_t lid_len, char *token,
                              uint32_t token_len);

#ifdef __cplusplus
}
#endif

#endif // _NVS_OP_H_