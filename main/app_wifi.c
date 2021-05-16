#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "lwip/apps/sntp.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "include/app_key.h"
#include "include/app_led.h"
#include "include/app_wifi.h"
#include "include/blufi.h"
#include "include/board_driver.h"
#include "include/event.h"
#include "include/task_httpc.h"
#include "include/util.h"

#define TAG "app_wifi"

/* Constants that aren't configurable in menuconfig */
#define DUMMY_SENSOR 1

/* NVS storage define */
#define STORAGE_NAMESPACE "storage"
#define NVSWIFISETTING "wificonfig"
#define NVSWIFICHECK "wifichecked"
#define NVSAPPLID "applid"
#define NVSAPPTOKEN "apptoken"

/* Test WiFi STA configuration */
#define WIFI_CONN_CHK_MS 10000 // TODO: can be set by command/config file
#define WIFI_CONN_RETRY 4

typedef struct {
    key_loop_event_t key_event;
} task_connect_cb_t;

static wifi_config_t sta_config;
static task_connect_cb_t task_conn_cb;
/* FreeRTOS event group to signal when we are connected & ready to make a
 * request */
static EventGroupHandle_t wifi_event_group;
/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
static const int WIFI_CONNECTED_BIT = BIT0;
/* store the station info for send back to phone */
/* lulupet API id and token */
static char lulupet_lid_get[NVS_LULUPET_LID_LEN];
static char lulupet_token_get[NVS_LULUPET_TOKEN_LEN];

static esp_err_t esp_err_print(esp_err_t err, const char *func, uint32_t line);
static void service_connect_event_handler(void *arg, esp_event_base_t base,
                                          int32_t event_id, void *event_data);
static void wifi_event_init(void);
static void wifi_init_from_nvs(void);
static void wifi_check_connect(uint32_t wait_ms, uint8_t retry);
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event);
static void set_led_cmd(unsigned int led_cmd_load);
static void sntp_obtain_time(void);
static void sntp_check(void);
static void snpt_time_check(void);

static esp_err_t nvs_init(void);
static int32_t nvs_read_wifichecked(void);
static esp_err_t nvs_reset_wifi_val(void);
static esp_err_t nvs_read_lid_token(void);
static esp_err_t nvs_read_wifi_config(void);

static esp_err_t esp_err_print(esp_err_t err, const char *func, uint32_t line) {
    ESP_LOGE(TAG, "err:%s %s:L%d", esp_err_to_name(err), func, line);
    return err;
}

static void set_led_cmd(unsigned int led_cmd_load) {
    xQueueSend(led_cmd_que, (void *)&led_cmd_load, (TickType_t)0);
}

static esp_err_t nvs_init(void) {
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

static esp_err_t nvs_reset_wifi_val(void) {
    ESP_LOGI(TAG, "%s:L%d", __func__, __LINE__);
    wifi_config_t wifi_config = {};
    return nvs_write_wifi_val(0, &wifi_config);
}

static esp_err_t nvs_read_lid_token(void) {
    nvs_handle_t handle;
    esp_err_t err;
    uint32_t len;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }

    // Read
    len = sizeof(lulupet_lid_get);
    err = nvs_get_blob(handle, NVSAPPLID, &lulupet_lid_get[0], &len);
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }
    len = sizeof(lulupet_token_get);
    err = nvs_get_blob(handle, NVSAPPTOKEN, &lulupet_token_get, &len);
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }
    ESP_LOGI(TAG, "nvs lid   : %s", lulupet_lid_get);
    ESP_LOGI(TAG, "nvs token : %s", lulupet_token_get);

    // Close
    nvs_close(handle);
    return ESP_OK;
}

static esp_err_t nvs_read_wifi_config(void) {
    nvs_handle_t handle;
    esp_err_t err;
    memset(&sta_config, 0x0, sizeof(sta_config));
    uint32_t len = sizeof(sta_config);

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }

    // Read
    err = nvs_get_blob(handle, NVSWIFISETTING, &sta_config, &len);
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }

    ESP_LOGI(TAG, "nvs read WiFi configure ssid:%s passwd:%s",
             sta_config.sta.ssid, sta_config.sta.password);
    // Close
    nvs_close(handle);
    return ESP_OK;
}

static int32_t nvs_read_wifichecked(void) {
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

static void wifi_init_from_nvs(void) {
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {};
    strcpy((char *)wifi_config.sta.ssid, (char *)sta_config.sta.ssid);
    strcpy((char *)wifi_config.sta.password, (char *)sta_config.sta.password);
    ESP_LOGI(TAG, "setting WiFi configuration SSID %s", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void wifi_check_connect(uint32_t wait_ms, uint8_t retry) {

    if (retry == 0) {
        retry = 1;
    }

    for (uint8_t i = 0; i < retry; ++i) {
        ESP_LOGI(TAG, "try to connect to WiFi %d time", i + 1);
        if (app_wifi_check_connect(wait_ms) == true) {
            return;
        }
        ESP_LOGI(TAG, "can't connecte to AP[SSID/PWD:%s/%s]",
                 sta_config.sta.ssid, sta_config.sta.password);
    }

    while (1) {
        set_led_cmd(LED_RED_1HZ);
        // TODO: Wait key event
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void wifi_event_init(void) {
    wifi_event_group = xEventGroupCreate();
    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
}

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event) {
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void snpt_time_check(void) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2020 - 1900)) {
        ESP_LOGI(TAG,
                 "Time is not set yet. Connecting to WiFi and getting time "
                 "over NTP.");
        sntp_obtain_time();
        // update 'now' variable with current time
        time(&now);
    }
    char strftime_buf[64];

    // Set timezone to China Standard Time
    setenv("TZ", "CST-8", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Taipei is: %s", strftime_buf);

    time_t seconds;
    seconds = time(NULL);
    ESP_LOGI(TAG, "Seconds since January 1, 1970 = %ld", seconds);
}

static void sntp_obtain_time(void) {
    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int retry_count = 10;

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true,
                        portMAX_DELAY);
    sntp_check();

    // wait for time to be set
    while (timeinfo.tm_year < (2020 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "waiting for system time to be set... (%d/%d)", retry,
                 retry_count);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
}

static void sntp_check(void) {
    ESP_LOGI(TAG, "init SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

static void service_connect_event_handler(void *arg, esp_event_base_t base,
                                          int32_t event_id, void *event_data) {
    switch (event_id) {
    case LULUPET_EVENT_KEY:
        memcpy(&task_conn_cb.key_event, (key_loop_event_t *)event_data,
               sizeof(key_loop_event_t));
        ESP_LOGW(TAG, "key event: %s",
                 app_key_event_type_str(task_conn_cb.key_event.key_event_type));
        break;
    default:
        break;
    }
}

static void wifi_start(esp_event_loop_handle_t event_loop) {

    esp_event_handler_register_with(event_loop, LULUPET_EVENT_BASE,
                                    ESP_EVENT_ANY_ID,
                                    service_connect_event_handler, NULL);

    wifi_event_init();
    ESP_LOGI(TAG, "load lid token from nvs");
    nvs_read_lid_token(); // TODO: error handling
    ESP_LOGI(TAG, "load WiFi Setting from nvs");
    nvs_read_wifi_config(); // TODO: error handling
    wifi_init_from_nvs();
    set_led_cmd(LED_GREEN_1HZ);
    wifi_check_connect(WIFI_CONN_CHK_MS, WIFI_CONN_RETRY);
    set_led_cmd(LED_GREEN_SOLID);
    ESP_LOGI(TAG, "connected to AP");
    snpt_time_check();
    ESP_LOGI(TAG, "Update time from SNTP");
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

esp_err_t nvs_write_lid_token(char *lid, char *token) {
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
    err = nvs_set_blob(handle, NVSAPPLID, lid, sizeof(lulupet_lid_get));
    if (err != ESP_OK) {
        return esp_err_print(err, __func__, __LINE__);
    }
    ESP_LOGI(TAG, "save token: %s into nvs", token);
    err = nvs_set_blob(handle, NVSAPPTOKEN, token, sizeof(lulupet_token_get));
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

char *app_wifi_get_lid(void) { return &lulupet_lid_get[0]; }

char *app_wifi_get_token(void) { return &lulupet_token_get[0]; }

bool app_wifi_check_connect(uint32_t wait_ms) {
    EventBits_t wifi_event_bits;

    wifi_event_bits =
        xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true,
                            wait_ms / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "wifi_event_bits: 0x%x", wifi_event_bits);
    if ((wifi_event_bits & WIFI_CONNECTED_BIT)) {
        return true;
    } else {
        return false;
    }
}

void app_wifi_main(esp_event_loop_handle_t event_loop) {
    ESP_LOGI(TAG, "start connect process");

    nvs_init();
    if (!nvs_read_wifichecked()) {
        blufi_start(event_loop);
    } else {
        wifi_start(event_loop);
    }

    start_httpc_task(event_loop);
}
