#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <string.h>

#include "include/nvs_op.h"

#define TAG "nvs_op"

/* nvs storage define */
#define STORAGE_NAMESPACE "storage"
#define NVS_WIFI_SETTING "wificonfig"
#define NVS_WIFI_CHECK "wifichecked"
#define NVS_APP_LID "applid"
#define NVS_APP_TOKEN "apptoken"
#define NVS_WEIGHT_CONF_V1 "weight_cfg_v1"

/* nvs_cali nvs_cali_weight define */
#define NVS_CALI_PARTITION "nvs_cali"
#define NVA_WEIGHT_CALI_NAMESPACE "nvs_cali_weight"
#define NVS_WEIGHT_CALI_VAL "weight_cali_cb"

static esp_err_t esp_err_print(esp_err_t esp_err, const char *file,
                               uint32_t line);

static esp_err_t esp_err_print(esp_err_t esp_err, const char *file,
                               uint32_t line) {
    ESP_LOGE(TAG, "err: %s %s():L%d", esp_err_to_name(esp_err), file, line);
    return esp_err;
}

esp_err_t nvs_cali_init(void) {
    ESP_LOGI(TAG, "%s init", NVS_CALI_PARTITION);
    esp_err_t esp_err = nvs_flash_init_partition(NVS_CALI_PARTITION);
    if (esp_err != ESP_OK) {
        ESP_LOGW(TAG, "%s erase", NVS_CALI_PARTITION);
        nvs_flash_erase_partition(NVS_CALI_PARTITION);
        if ((esp_err = nvs_flash_init_partition(NVS_CALI_PARTITION)) !=
            ESP_OK) {
            return esp_err_print(esp_err, __func__, __LINE__);
        }
    }
    ESP_LOGW(TAG, "%s init ok", NVS_CALI_PARTITION);
    return esp_err;
}

esp_err_t nvs_cali_read_weight_clai_cb(weight_cali_cb *cali) {
    if (cali == NULL) {
        return esp_err_print(ESP_ERR_INVALID_ARG, __func__, __LINE__);
    }

    esp_err_t esp_err;
    nvs_handle_t h;
    esp_err = nvs_open_from_partition(
        NVS_CALI_PARTITION, NVA_WEIGHT_CALI_NAMESPACE, NVS_READONLY, &h);
    if (esp_err != ESP_OK) {
        return esp_err_print(esp_err, __func__, __LINE__);
    }

    uint32_t len = sizeof(weight_cali_cb);
    esp_err = nvs_get_blob(h, NVS_WEIGHT_CALI_VAL, cali, &len);
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }

    weight_list_cali_val(cali);

_end:
    nvs_close(h);
    return esp_err;
}

esp_err_t nvs_cali_write_weight_clai_cb(weight_cali_cb *cali) {
    if (cali == NULL) {
        return esp_err_print(ESP_ERR_INVALID_ARG, __func__, __LINE__);
    }

    esp_err_t esp_err;
    nvs_handle_t h;
    esp_err = nvs_open_from_partition(
        NVS_CALI_PARTITION, NVA_WEIGHT_CALI_NAMESPACE, NVS_READWRITE, &h);
    if (esp_err != ESP_OK) {
        return esp_err_print(esp_err, __func__, __LINE__);
    }

    esp_err =
        nvs_set_blob(h, NVS_WEIGHT_CALI_VAL, cali, sizeof(weight_cali_cb));
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }

    esp_err = nvs_commit(h);
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }

_end:
    nvs_close(h);
    return esp_err;
}

esp_err_t nvs_cali_reset_weight_clai_cb(void) {
    weight_cali_cb cali;

    memset(&cali, 0x00, sizeof(weight_cali_cb));

    return nvs_cali_write_weight_clai_cb(&cali);
}

esp_err_t nvs_init(void) {
    esp_err_t esp_err;

#if (FUNC_ERASE_NVS_BOOTUP)
    ESP_ERROR_CHECK(nvs_flash_erase());
#endif
    esp_err = nvs_flash_init();
    ESP_LOGI(TAG, "nvs init");
    if (esp_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        esp_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "nvs erase");
        ESP_ERROR_CHECK(nvs_flash_erase());
        esp_err = nvs_flash_init();
        if (esp_err != ESP_OK)
            esp_err_print(esp_err, __func__, __LINE__);
    }
    ESP_LOGW(TAG, "nvs init ok");
    return esp_err;
}

int32_t nvs_read_wifichecked(void) {
    nvs_handle_t handle;
    esp_err_t esp_err;
    int32_t wifichecked_value = 0;
    int32_t set_value = 0;
    wifi_config_t wifi_config = {};

    esp_err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __FUNCTION__, __LINE__);
        nvs_init();
        nvs_reset_wifi_val();
        return wifichecked_value;
    }

    esp_err = nvs_get_i32(handle, NVS_WIFI_CHECK, &wifichecked_value);
    if (esp_err != ESP_OK) {
        // if can not read nvs NVS_WIFI_CHECK, write the default value into nvs
        ESP_LOGW(TAG, "nvs wifichecked not found");
        ESP_LOGW(TAG, "nvs write wifichecked/wificonfig");
        ESP_ERROR_CHECK(nvs_set_i32(handle, NVS_WIFI_CHECK, set_value));
        ESP_ERROR_CHECK(nvs_set_blob(handle, NVS_WIFI_SETTING, &wifi_config,
                                     sizeof(wifi_config)));
        nvs_get_i32(handle, NVS_WIFI_CHECK, &wifichecked_value);
    }
    ESP_LOGI(TAG, "nvs read wifichecked : %d", wifichecked_value);

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
    esp_err_t esp_err;

    if (lid == NULL || token == NULL || lid_len == 0 || token_len == 0) {
        return esp_err_print(ESP_ERR_INVALID_ARG, __func__, __LINE__);
    }

    esp_err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (esp_err != ESP_OK) {
        return esp_err_print(esp_err, __func__, __LINE__);
    }

    esp_err = nvs_get_blob(handle, NVS_APP_LID, lid, &lid_len);
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }
    esp_err = nvs_get_blob(handle, NVS_APP_TOKEN, token, &token_len);
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }
    ESP_LOGI(TAG, "nvs lid   : %s", lid);
    ESP_LOGI(TAG, "nvs token : %s", token);

_end:
    nvs_close(handle);
    return esp_err;
}

esp_err_t nvs_read_wifi_config(wifi_config_t *sta_config) {
    nvs_handle_t handle;
    esp_err_t esp_err;
    memset(sta_config, 0x0, sizeof(wifi_config_t));

    esp_err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (esp_err != ESP_OK) {
        return esp_err_print(esp_err, __func__, __LINE__);
    }

    uint32_t len = sizeof(wifi_config_t);
    esp_err = nvs_get_blob(handle, NVS_WIFI_SETTING, sta_config, &len);
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }

    ESP_LOGI(TAG, "nvs read WiFi configure ssid:%s passwd:%s",
             sta_config->sta.ssid, sta_config->sta.password);

_end:
    nvs_close(handle);
    return esp_err;
}

esp_err_t nvs_write_wifi_val(int32_t set_value, wifi_config_t *wifi_config) {
    nvs_handle_t handle;
    esp_err_t esp_err;

    esp_err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (esp_err != ESP_OK) {
        return esp_err_print(esp_err, __func__, __LINE__);
    }

    ESP_LOGI(TAG, "save set_value: %d into nvs", set_value);
    esp_err = nvs_set_i32(handle, NVS_WIFI_CHECK, set_value);
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }
    ESP_LOGI(TAG, "save wifi_config into nvs");
    esp_err = nvs_set_blob(handle, NVS_WIFI_SETTING, wifi_config,
                           sizeof(wifi_config_t));
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }

    esp_err = nvs_commit(handle);
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }

_end:
    nvs_close(handle);
    return esp_err;
}

esp_err_t nvs_write_lid_token(char *lid, uint32_t lid_len, char *token,
                              uint32_t token_len) {
    nvs_handle_t handle;
    esp_err_t esp_err;

    if (lid == NULL || token == NULL) {
        esp_err_print(ESP_ERR_INVALID_ARG, __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (esp_err != ESP_OK) {
        return esp_err_print(esp_err, __func__, __LINE__);
    }

    ESP_LOGI(TAG, "save lid: %s into nvs", lid);
    esp_err = nvs_set_blob(handle, NVS_APP_LID, lid, lid_len);
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }
    ESP_LOGI(TAG, "save token: %s into nvs", token);
    esp_err = nvs_set_blob(handle, NVS_APP_TOKEN, token, token_len);
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }

    esp_err = nvs_commit(handle);
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }

_end:
    nvs_close(handle);
    return esp_err;
}

esp_err_t nvs_read_weight_conf(void *conf, int version) {
    nvs_handle_t handle;
    esp_err_t esp_err;
    weight_conf_ver1_t *weight_cfg = NULL;

    if (conf == NULL) {
        return esp_err_print(ESP_ERR_INVALID_ARG, __func__, __LINE__);
    }

    if (version == 1) {
        weight_cfg = (weight_conf_ver1_t *)conf;
    } else {
        return esp_err_print(ESP_ERR_INVALID_ARG, __func__, __LINE__);
    }

    esp_err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }

    if (version == 1) {
    }
    uint32_t len = sizeof(weight_conf_ver1_t);
    esp_err = nvs_get_blob(handle, NVS_WEIGHT_CONF_V1, weight_cfg, &len);
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }

    ESP_LOGI(TAG, "%s ok", __func__);

    weight_dump_weight_conf_v1(weight_cfg);

_end:
    nvs_close(handle);
    return esp_err;
}

esp_err_t nvs_write_weight_conf(void *conf, int version) {
    nvs_handle_t handle;
    esp_err_t esp_err;
    weight_conf_ver1_t *weight_cfg = NULL;

    if (conf == NULL) {
        return esp_err_print(ESP_ERR_INVALID_ARG, __func__, __LINE__);
    }

    esp_err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (esp_err != ESP_OK) {
        return esp_err_print(esp_err, __func__, __LINE__);
    }

    if (version == 1) {
        weight_cfg = (weight_conf_ver1_t *)conf;
    } else {
        return esp_err_print(ESP_ERR_INVALID_ARG, __func__, __LINE__);
    }

    esp_err = nvs_set_blob(handle, NVS_WEIGHT_CONF_V1, weight_cfg,
                           sizeof(weight_conf_ver1_t));
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }

    esp_err = nvs_commit(handle);
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }

    ESP_LOGI(TAG, "%s ok", __func__);

_end:
    nvs_close(handle);
    return esp_err;
}

esp_err_t nvs_reset_weight_conf(void) {
    nvs_handle_t handle;
    esp_err_t esp_err;

    esp_err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (esp_err != ESP_OK) {
        return esp_err_print(esp_err, __func__, __LINE__);
    }

    esp_err = nvs_erase_key(handle, NVS_WEIGHT_CONF_V1);
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }

    esp_err = nvs_commit(handle);
    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }

    ESP_LOGI(TAG, "%s ok", __func__);

_end:
    nvs_close(handle);
    return esp_err;
}
