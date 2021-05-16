#include "app_wifi.h"
#include "cJSON.h"
#include "esp_attr.h"
#include "esp_camera.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "fb_gfx.h"
#include "img_converters.h"
#include "lwip/apps/sntp.h"
#include "lwip/err.h"
#include "mbedtls/base64.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "include/app_key.h"
#include "include/app_led.h"
#include "include/app_wifi.h"
#include "include/blufi.h"
#include "include/board_driver.h"
#include "include/event.h"
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
#define WIFI_TEST_MODE 0
#define BLUFI_CHK_CONN_MS 300000 // TODO: can be set by command/config file
#define WIFI_CONN_CHK_MS 10000   // TODO: can be set by command/config file
#define WIFI_CONN_RETRY 4

typedef struct {
    esp_event_loop_handle_t evt_loop;
} httpc_task_config_t;

typedef struct {
    key_loop_event_t key_event;
} task_connect_cb_t;

static httpc_task_config_t task_conf;
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
static char lulupet_lid[10] = "lid118";
static char lulupet_token[10] = "WebLid118";
static char lulupet_lid_get[NVS_LULUPET_LID_LEN];
static char lulupet_token_get[NVS_LULUPET_TOKEN_LEN];

static esp_err_t esp_err_print(esp_err_t err, const char *func, uint32_t line);
static void service_connect_event_handler(void *arg, esp_event_base_t base,
                                          int32_t event_id, void *event_data);
static void wifi_event_init(void);
static void wifi_init_from_nvs(void);
static void wifi_check_connect(uint32_t wait_ms, uint8_t retry);
static bool wifi_event_check_conn(uint32_t wait_ms);
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event);
static void set_led_cmd(unsigned int led_cmd_load);
static void sntp_obtain_time(void);
static void sntp_check(void);
static void snpt_time_check(void);
static void httpc_task(void *pvParameter);
static void httpc_task_start(esp_event_loop_handle_t event_loop);

static esp_err_t nvs_init(void);
static int32_t nvs_read_wifichecked(void);
static esp_err_t nvs_reset_wifi_val(void);
static esp_err_t nvs_read_lid_token(void);
static esp_err_t nvs_read_wifi_config(void);

static void camera_take_photo(camera_fb_t **fb);

// unused code
#if 0
#define EXAMPLE_WIFI_SSID "SlingXCorp"
#define EXAMPLE_WIFI_PASS "25413113"

static char urlbuffer[100];
extern const char howsmyssl_com_root_cert_pem_start[] asm(
    "_binary_lulupet_com_root_cert_pem_start");
extern const char howsmyssl_com_root_cert_pem_end[] asm(
    "_binary_lulupet_com_root_cert_pem_end");

static void initialise_test_wifi(void);
static esp_err_t read_wifi_nvs(void);

static void initialise_test_wifi(void) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());

    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta =
            {
                .ssid = EXAMPLE_WIFI_SSID,
                .password = EXAMPLE_WIFI_PASS,
            },
    };
    BLUFI_INFO("Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

#endif

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
        if (wifi_event_check_conn(wait_ms) == true) {
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

static bool wifi_event_check_conn(uint32_t wait_ms) {
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

static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key,
                 evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        if (!esp_http_client_is_chunked_response(evt->client)) {
            // Write out data
            // printf("%.*s", evt->data_len, (char*)evt->data);
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

static void camera_take_photo(camera_fb_t **fb) {
    *fb = esp_camera_fb_get();

    if (!fb) {
        ESP_LOGE(TAG, "camera take photo failed");
        esp_camera_fb_return(*fb);
        ESP_LOGE(
            TAG,
            "resolve camera problem, reboot system"); // TODO: record into log
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        esp_restart();
        return;
    }
    ESP_LOGI(TAG, "camera take photo ok");
}

static void http_post_photo(esp_http_client_handle_t client, char *json_url_val,
                            int json_url_val_len, time_t *timestamp) {

#define HTTP_PAYLOAD_HEADER_LEN 200
#define HTTP_PAYLOAD_FOOTER_LEN 50
#define HTTP_PAYLOAD_LENGTH_LEN 10
#define HTTP_PAYLOAD_HEADER_FILENAME "lulupet-cat.jpg"

    bool client_open = false;
    int client_rd_len;
    char *payload_header = NULL;
    char *payload_footer = NULL;
    char *payload_length = NULL;
    char *json_str = NULL;
    cJSON *json_root = NULL;
    camera_fb_t *fb = NULL;
    esp_err_t err;

    if (json_url_val == NULL) {
        esp_err_print(ESP_ERR_NO_MEM, __func__, __LINE__);
        return;
    }

    // read unix timestamp
    *timestamp = time(NULL);
    camera_take_photo(&fb);
    ESP_LOGW(TAG, "L%d", __LINE__);
    if (fb->format != PIXFORMAT_JPEG) {
        ESP_LOGE(TAG, "camera use the %d format", fb->format);
        return;
    }
    ESP_LOGW(TAG, "L%d", __LINE__);
    // HTTP HEAD
    const char *content_type =
        "multipart/form-data; boundary=----WebKitFormBoundarykqaGaA5tlVdQyckh";
    ESP_LOGW(TAG, "L%d", __LINE__);
    const char *boundary = "----WebKitFormBoundarykqaGaA5tlVdQyckh";
    ESP_LOGW(TAG, "L%d", __LINE__);
    payload_header = calloc(HTTP_PAYLOAD_HEADER_LEN, sizeof(char));
    if (payload_header == NULL) {
        esp_err_print(ESP_ERR_NO_MEM, __func__, __LINE__);
        goto http_post_photo_end;
    }
    snprintf(payload_header, HTTP_PAYLOAD_HEADER_LEN,
             "--%s\r\nContent-Disposition: form-data; name=\"image\"; "
             "filename=\"%s\"\r\nContent-Type: image/jpeg\r\n\r\n",
             boundary, HTTP_PAYLOAD_HEADER_FILENAME);
    ESP_LOGW(TAG, "L%d", __LINE__);
    payload_footer = calloc(HTTP_PAYLOAD_FOOTER_LEN, sizeof(char));
    if (payload_footer == NULL) {
        esp_err_print(ESP_ERR_NO_MEM, __func__, __LINE__);
        goto http_post_photo_end;
    }
    snprintf(payload_footer, HTTP_PAYLOAD_FOOTER_LEN, "\r\n--%s--\r\n",
             boundary);
    ESP_LOGW(TAG, "L%d", __LINE__);
    int content_length =
        strlen(payload_header) + fb->len + strlen(payload_footer);
    payload_length = calloc(HTTP_PAYLOAD_LENGTH_LEN, sizeof(char));
    if (payload_length == NULL) {
        esp_err_print(ESP_ERR_NO_MEM, __func__, __LINE__);
        goto http_post_photo_end;
    }
    snprintf(payload_length, HTTP_PAYLOAD_LENGTH_LEN, "%d", content_length);
    ESP_LOGW(TAG, "L%d", __LINE__);
    ESP_LOGI(TAG, "payloadLength:\n%s", payload_length);
    ESP_LOGI(TAG, "payloadHeader:\n%s", payload_header);
    ESP_LOGI(TAG, "payloadFooter:\n%s", payload_footer);
    ESP_LOGW(TAG, "L%d", __LINE__);
    // esp_http_client_open -> esp_http_client_write ->
    // esp_http_client_fetch_headers -> esp_http_client_read (and option)
    // esp_http_client_close.
    esp_http_client_set_header(client, "Host", SERVER_URL);
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "Connection", "close");
    esp_http_client_set_header(client, "Content-Type", content_type);
    esp_http_client_set_header(client, "Content-Length", payload_length);
    esp_http_client_set_url(client, HTTP_PHOTO_URL);
    esp_http_client_set_method(client, HTTP_METHOD_POST);

    if ((err = esp_http_client_open(client, content_length)) != ESP_OK) {
        esp_err_print(err, __func__, __LINE__);
        goto http_post_photo_end;
    }
    client_open = true;
    ESP_LOGI(TAG, "http client open");

    if ((client_rd_len = esp_http_client_write(client, payload_header,
                                               strlen(payload_header))) < 0) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_photo_end;
    }
    ESP_LOGI(TAG, "http client write header, length: %d", client_rd_len);

    if ((client_rd_len = esp_http_client_write(client, (const char *)fb->buf,
                                               (fb->len))) < 0) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_photo_end;
    }
    ESP_LOGI(TAG, "http client write playload, length: %d", client_rd_len);

    if ((client_rd_len = esp_http_client_write(client, payload_footer,
                                               strlen(payload_footer))) < 0) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_photo_end;
    }
    ESP_LOGI(TAG, "http client write footer, length: %d", client_rd_len);

    // fetch json string example:
    // {"msg": "Upload Success", "result": "Success", "image": {"name":
    // "victor-test.jpg", "size": 31285, "content_type": "image/jpeg", "url":
    // "http://lulupet.williamhsu.com.tw/media/7881a26566c74127b8e97a2632596d32.jpg"}}
    if ((content_length = esp_http_client_fetch_headers(client)) < 0) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_photo_end;
    }
    ESP_LOGI(TAG, "http client fetech header, length =%d", content_length);

    int read_len;
    json_str = calloc(MAX_HTTP_RECV_BUFFER + 1, sizeof(char));
    if (json_str == NULL) {
        esp_err_print(ESP_ERR_NO_MEM, __func__, __LINE__);
        goto http_post_photo_end;
    }

    if (content_length > MAX_HTTP_RECV_BUFFER) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_photo_end;
    }

    // TODO: use while loop to read data with retry
    if ((read_len = esp_http_client_read(client, json_str, content_length)) <=
        0) {
        ESP_LOGE(TAG, "Error read data");
    }
    ESP_LOGI(TAG, "read_len: %d", read_len);
    ESP_LOGI(TAG, "http client read: %s", json_str);

    json_root = cJSON_Parse(json_str);

    if (json_root == NULL) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_photo_end;
    }
    cJSON *json_image_obj = cJSON_GetObjectItem(json_root, "image");
    if (json_image_obj == NULL) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_photo_end;
    }
    cJSON *json_url_obj = cJSON_GetObjectItem(json_image_obj, "url");
    if (json_url_obj == NULL) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_photo_end;
    }

    snprintf(json_url_val, json_url_val_len, "%s",
             cJSON_GetStringValue(json_url_obj));
    ESP_LOGI(TAG, "read url:%s", json_url_val);

http_post_photo_end:
    if (fb) {
        esp_camera_fb_return(fb);
    }
    if (payload_header) {
        free(payload_header);
    }
    if (payload_footer) {
        free(payload_footer);
    }
    if (payload_length) {
        free(payload_length);
    }
    if (json_str) {
        free(json_str);
    }
    if (json_root) {
        cJSON_Delete(json_root);
    }
    if (client_open) {
        esp_http_client_close(client);
        ESP_LOGI(TAG, "http client close");
    }

    return;
}

static void http_post_raw(esp_http_client_handle_t client, char *json_url_val,
                          uint32_t weight, bool pir, time_t timestamp) {
#define HTTP_POST_RAW_DATA_LEN 256

    int client_wr_len;
    int content_length;
    esp_err_t err;
    bool client_open = false;
    char *json_str = NULL;
    char *post_data_raw = calloc(HTTP_POST_RAW_DATA_LEN, sizeof(char));
    if (post_data_raw == NULL) {
        esp_err_print(ESP_ERR_NO_MEM, __func__, __LINE__);
        goto http_post_raw_end;
    }

    snprintf(post_data_raw, HTTP_POST_RAW_DATA_LEN,
             "lid=%s&token=%s&weight=%u&pir=%d&pic=%s&tt=%ld", lulupet_lid,
             lulupet_token, weight, pir, json_url_val, timestamp);
    ESP_LOGI(TAG, "post data:\n%s", post_data_raw);

    esp_http_client_set_url(client, HTTP_RAW_URL);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "accept", "application/json");
    esp_http_client_set_header(client, "Content-Type",
                               "application/x-www-form-urlencoded");
    esp_http_client_set_header(
        client, "X-CSRFToken",
        "eA8ob2RLGxH6sQ7njh6pokrwNNTxR7gDqpfhPY9VyO8M9B8HZIaMFrKClihBLO39");

    if ((err = esp_http_client_open(client, strlen(post_data_raw))) != ESP_OK) {
        esp_err_print(ESP_ERR_NO_MEM, __func__, __LINE__);
        goto http_post_raw_end;
    }
    client_open = true;
    ESP_LOGI(TAG, "http client open");

    if ((client_wr_len = esp_http_client_write(client, post_data_raw,
                                               strlen(post_data_raw))) < 0) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_raw_end;
    }
    ESP_LOGI(TAG, "http client write, length: %d", client_wr_len);

    if ((content_length = esp_http_client_fetch_headers(client)) < 0) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_raw_end;
    }
    ESP_LOGI(TAG, "http client fetech, length: %d", content_length);

    if (content_length > MAX_HTTP_RECV_BUFFER) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_raw_end;
    }

    json_str = calloc(MAX_HTTP_RECV_BUFFER + 1, sizeof(char));
    if (json_str == NULL) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_raw_end;
    }

    int read_len;

    if ((read_len = esp_http_client_read(client, json_str, content_length)) <=
        0) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_raw_end;
    }
    ESP_LOGI(TAG, "http client read json: %s", json_str);
    ESP_LOGI(TAG, "read_len: %d", read_len);

http_post_raw_end:
    if (post_data_raw) {
        free(post_data_raw);
    }
    if (json_str) {
        free(json_str);
    }
    if (client_open) {
        esp_http_client_close(client);
        ESP_LOGI(TAG, "http client close");
    }
}

static void http_post_data(void) {
#define JSON_URL_VAL_LEN 256

    ESP_LOGI(TAG, "Free Heap Internal is:  %d Byte",
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "Free Heap PSRAM    is:  %d Byte",
             heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    esp_http_client_config_t config = {
        .url = HTTP_PHOTO_URL,
        .event_handler = http_event_handler,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        return;
    }
    ESP_LOGI(TAG, "http client init");

    char *json_url_val = calloc(JSON_URL_VAL_LEN, sizeof(char));
    if (json_url_val == NULL) {
        esp_err_print(ESP_ERR_NO_MEM, __func__, __LINE__);
        return;
    }

    time_t unix_timestamp;
    http_post_photo(client, json_url_val, JSON_URL_VAL_LEN, &unix_timestamp);
    vTaskDelay(pdMS_TO_TICKS(100));
    http_post_raw(client, json_url_val, 1230, 1, unix_timestamp);

    esp_http_client_cleanup(client);
    ESP_LOGI(TAG, "http client cleanup");
    return;
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
    // vTaskDelay(30000 / portTICK_PERIOD_MS);
}

static void httpc_task(void *pvParameter) {
    httpc_task_config_t *conf = (httpc_task_config_t *)pvParameter;
    int i = 0;

    while (i < 1) {
        ESP_LOGI(TAG, "checking WiFi status");
        xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true,
                            portMAX_DELAY);
        ESP_LOGI(TAG, "start to upload photo");
        http_post_data();
        // capture_photo_only();
        ESP_LOGI(TAG, "http post data test : %d ok", i);
        i++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "end");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void httpc_task_start(esp_event_loop_handle_t event_loop) {

    task_conf.evt_loop = event_loop;

    xTaskCreate(&httpc_task, "httpc_task", 4096, (void *)&task_conf, 5, NULL);
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

void app_wifi_main(esp_event_loop_handle_t event_loop) {
    ESP_LOGI(TAG, "start connect process");

    nvs_init();
    if (!nvs_read_wifichecked()) {
        blufi_start(event_loop);
    } else {
        wifi_start(event_loop);
    }

    httpc_task_start(event_loop);
}
