#ifndef _BLUFI_H_
#define _BLUFI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include "stdint.h"

#define BLUFI_TAG "BluFi"
#define BLUFI_INFO(fmt, ...) ESP_LOGI(BLUFI_TAG, fmt, ##__VA_ARGS__)
#define BLUFI_DEBUG(fmt, ...) ESP_LOGD(BLUFI_TAG, fmt, ##__VA_ARGS__)
#define BLUFI_WARNING(fmt, ...) ESP_LOGW(BLUFI_TAG, fmt, ##__VA_ARGS__)
#define BLUFI_ERROR(fmt, ...) ESP_LOGE(BLUFI_TAG, fmt, ##__VA_ARGS__)

void blufi_dh_negotiate_data_handler(uint8_t *data, int len,
                                     uint8_t **output_data, int *output_len,
                                     bool *need_free);
int blufi_aes_encrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len);
int blufi_aes_decrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len);
uint16_t blufi_crc_checksum(uint8_t iv8, uint8_t *data, int len);
int blufi_security_init(void);
void blufi_security_deinit(void);

#endif  // _BLUFI_H_
