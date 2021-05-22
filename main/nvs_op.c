#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <string.h>

#include "include/nvs_op.h"

#define TAG "nvs_op"

/* NVS storage define */
#define STORAGE_NAMESPACE "storage"
#define NVSWIFISETTING "wificonfig"
#define NVSWIFICHECK "wifichecked"
#define NVSAPPLID "applid"
#define NVSAPPTOKEN "apptoken"

static esp_err_t esp_err_print(esp_err_t err, const char *file, uint32_t line);

static esp_err_t esp_err_print(esp_err_t err, const char *file, uint32_t line) {
    ESP_LOGE(TAG, "err: %s %s:L%d", esp_err_to_name(err), file, line);
    return err;
}

esp_err_t nvs_init(void) {
    esp_err_t err;

#if (FUNC_ERASE_NVS_BOOTUP)
    ESP_ERROR_CHECK(nvs_flash_erase());
#endif
    err = nvs_flash_init();
    ESP_LOGI(TAG, "nvs init");
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "nvs error, erase");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
        if (err != ESP_OK)
            esp_err_print(err, __func__, __LINE__);
    }
    ESP_LOGI(TAG, "nvs ready");
    return err;
}

int32_t nvs_read_wifichecked(void) {
    nvs_handle_t handle;
    esp_err_t err;
    int32_t wifichecked_value = 0;
    int32_t set_value = 0;
    wifi_config_t wifi_config = {};

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        esp_err_print(err, __FUNCTION__, __LINE__);
        nvs_init();
        nvs_reset_wifi_val();
        return wifichecked_value;
    }

    // read nvs NVSWIFICHECK
    err = nvs_get_i32(handle, NVSWIFICHECK, &wifichecked_value);
    if (err != ESP_OK) {
        // if can not read nvs NVSWIFICHECK, write the default value into nvs
        ESP_LOGW(TAG, "nvs wifichecked not found");
        ESP_LOGW(TAG, "nvs write wifichecked/wificonfig");
        // Write
        ESP_ERROR_CHECK(nvs_set_i32(handle, NVSWIFICHECK, set_value));
        ESP_ERROR_CHECK(nvs_set_blob(handle, NVSWIFISETTING, &wifi_config,
                                     sizeof(wifi_config)));
        nvs_get_i32(handle, NVSWIFICHECK, &wifichecked_value);
    }
    ESP_LOGI(TAG, "nvs read wifichecked : %d", wifichecked_value);

    // Close
    nvs_close(handle);
    return wifichecked_value;
}

esp_err_t nvs_reset_wifi_val(void) {
    ESP_LOGI(TAG, "%s:L%d", __func__, __LINE__);
    wifi_config_t wifi_config = {};
    return nvs_write_wifi_val(0, &wifi_config);
}

esp_err_t nvs_read_lid_token(char *lid, uint32_t lid_len, char *token,
                             uint32_t token_len) {
    nvs_handle_t handle;
    esp_err_t err;

    if (lid == NULL || token == NULL || lid_len == 0 || token_len == 0) {
        return esp_err_print(ESP_ERR_INVALID_ARG, __func__, __LINE__);
    }

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }

    // Read
    err = nvs_get_blob(handle, NVSAPPLID, lid, &lid_len);
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }
    err = nvs_get_blob(handle, NVSAPPTOKEN, token, &token_len);
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }
    ESP_LOGI(TAG, "nvs lid   : %s", lid);
    ESP_LOGI(TAG, "nvs token : %s", token);

    // Close
    nvs_close(handle);
    return ESP_OK;
}

esp_err_t nvs_read_wifi_config(wifi_config_t *sta_config) {
    nvs_handle_t handle;
    esp_err_t err;
    memset(sta_config, 0x0, sizeof(wifi_config_t));
    uint32_t len = sizeof(wifi_config_t);

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }

    // Read
    err = nvs_get_blob(handle, NVSWIFISETTING, sta_config, &len);
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }

    ESP_LOGI(TAG, "nvs read WiFi configure ssid:%s passwd:%s",
             sta_config->sta.ssid, sta_config->sta.password);
    // Close
    nvs_close(handle);
    return ESP_OK;
}

esp_err_t nvs_write_wifi_val(int32_t set_value, wifi_config_t *wifi_config) {
    nvs_handle_t handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }

    // Write
    ESP_LOGI(TAG, "save set_value: %d into nvs", set_value);
    err = nvs_set_i32(handle, NVSWIFICHECK, set_value);
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }
    ESP_LOGI(TAG, "save wifi_config into nvs");
    err = nvs_set_blob(handle, NVSWIFISETTING, wifi_config,
                       sizeof(wifi_config_t));
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }

    // Commit
    err = nvs_commit(handle);
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }

    // Close
    nvs_close(handle);
    return ESP_OK;
}

esp_err_t nvs_write_lid_token(char *lid, uint32_t lid_len, char *token,
                              uint32_t token_len) {
    nvs_handle_t handle;
    esp_err_t err;

    if (lid == NULL || token == NULL) {
        esp_err_print(ESP_ERR_INVALID_ARG, __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }

    // Write
    ESP_LOGI(TAG, "save lid: %s into nvs", lid);
    err = nvs_set_blob(handle, NVSAPPLID, lid, lid_len);
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }
    ESP_LOGI(TAG, "save token: %s into nvs", token);
    err = nvs_set_blob(handle, NVSAPPTOKEN, token, token_len);
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }
    // Commit
    err = nvs_commit(handle);
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }

    // Close
    nvs_close(handle);
    return ESP_OK;
}
