#include "cJSON.h"
#include "esp_camera.h"
#include "esp_err.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

#include "include/app_camera.h"
#include "include/app_led.h"
#include "include/app_weight.h"
#include "include/app_wifi.h"
#include "include/board_driver.h"
#include "include/event.h"
#include "include/nvs_op.h"
#include "include/task_fs.h"
#include "include/task_httpc.h"
#include "include/util.h"

#define TAG "httpc_task"
#define HTTPC_SAVE_TIMEVAL_INTO_NVS_MS 1800000    // 30 min
#define HTTPC_WAIT_FIRST_CONNTECTED_WIFI_MS 20000 // 20 sec
#define HTTPC_TASK_PERIOD_MS 100
#define HTTP_POST_RAW_DATA_LEN 256
#define HTTP_PAYLOAD_HEADER_LEN 200
#define HTTP_PAYLOAD_FOOTER_LEN 50
#define HTTP_PAYLOAD_LENGTH_LEN 10
#define JSON_URL_VAL_LEN 256
#define HTTP_PAYLOAD_HEADER_FILENAME "lulupet-cat.jpg"
#define UPDATE_FIRMWARE_BUF_SIZE 0x1000
#define HTTP_POST_IMAGEHELPER_RETRY 5
#define HTTP_POST_IMAGEHELPER_RETRY_FAILED_STR "error_image_not_send"

typedef struct {
} httpc_ota_event_t;

typedef struct {
    bool task_enable;

    esp_event_loop_handle_t evt_loop;
    bool weight_event_update;
    bool ota_event_update;
    weight_take_photo_event_t weight_take_photo_evt;
    httpc_ota_event_t ota_evt;
    SemaphoreHandle_t data_mutex;
    uint32_t err_line;
} httpc_task_config_t;

typedef struct {
    uint8_t idx;
    bool loop;
    httpc_photo_buf_t buf[CAM_RING_BUF_SIZE];
} httpc_photo_ring_buf_t;

static httpc_task_config_t task_conf;
static httpc_photo_ring_buf_t photo_ring_buf;

static esp_err_t esp_err_print(esp_err_t err, const char *func, uint32_t line);
static esp_err_t http_event_handler(esp_http_client_event_t *evt);
static esp_err_t http_post_imageHelper(esp_http_client_handle_t client,
                                       char *json_url_val, int json_url_val_len,
                                       camera_fb_t *fb);
static void http_post_rawdata(esp_http_client_handle_t client,
                              char *json_url_val, time_t timestamp,
                              weight_take_photo_event_t *take_photo_event);
static void http_send_photo_buf(httpc_photo_buf_t *photo_buf,
                                esp_http_client_handle_t client,
                                char *json_url_val, int json_url_val_len);
static int http_get_ota_update_config_latest(httpc_ota_event_t *event,
                                             char *cfg_url, int cfg_url_len,
                                             char *ver_str, int ver_str_len);
static int http_get_ota_update_latest(httpc_ota_event_t *event, char *ota_url,
                                      int ota_url_len, char *ver_str,
                                      int ver_str_len);
static int http_ota(char *ota_url);
static esp_err_t http_get_cfg(char *cfg_url);
static httpc_photo_buf_t *http_photo_buf_pop_ram(uint8_t idx);
static bool http_photo_buf_get_loop_ram(void);
static uint8_t http_photo_buf_get_idx_ram(void);
static bool http_photo_buf_exist_ram(void);
static void http_photo_buf_init(void);

static void httpc_task(void *pvParameter);

static esp_err_t esp_err_print(esp_err_t err, const char *func, uint32_t line) {
    ESP_LOGE(TAG, "err:%s %s():L%d", esp_err_to_name(err), func, line);
    task_conf.err_line = line;
    return err;
}

static void httpc_loop_event_handler(void *arg, esp_event_base_t base,
                                     int32_t event_id, void *event_data) {

    xSemaphoreTake(task_conf.data_mutex, portMAX_DELAY);
    switch (event_id) {
    case LULUPET_EVENT_TAKE_PHOTO:
        memcpy(&task_conf.weight_take_photo_evt,
               (weight_take_photo_event_t *)event_data,
               sizeof(weight_take_photo_event_t));
        ESP_LOGW(
            TAG,
            "weight_take_photo_event recv: weight[%d g] pir[%d] eventid[%d]",
            task_conf.weight_take_photo_evt.weight_g,
            task_conf.weight_take_photo_evt.pir_val,
            task_conf.weight_take_photo_evt.eventid);
        task_conf.weight_event_update = true;
        break;
    case LULUPET_EVENT_OTA:
        memcpy(&task_conf.ota_evt, (httpc_ota_event_t *)event_data,
               sizeof(httpc_ota_event_t));
        task_conf.ota_event_update = true;
        break;
    default:
        break;
    }
    xSemaphoreGive(task_conf.data_mutex);
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

static esp_err_t http_post_imageHelper(esp_http_client_handle_t client,
                                       char *json_url_val, int json_url_val_len,
                                       camera_fb_t *fb) {
    bool client_open = false;
    int client_rd_len;
    char *payload_header = NULL;
    char *payload_footer = NULL;
    char *payload_length = NULL;
    char *json_str = NULL;
    cJSON *json_root = NULL;
    esp_err_t esp_err = ESP_OK;

    if (json_url_val == NULL) {
        return esp_err_print(ESP_ERR_NO_MEM, __func__, __LINE__);
    }

    // HTTP HEAD
    const char *content_type =
        "multipart/form-data; boundary=----WebKitFormBoundarykqaGaA5tlVdQyckh";

    const char *boundary = "----WebKitFormBoundarykqaGaA5tlVdQyckh";

    payload_header = calloc(HTTP_PAYLOAD_HEADER_LEN, sizeof(char));
    if (payload_header == NULL) {
        esp_err = ESP_ERR_NO_MEM;
        esp_err_print(esp_err, __func__, __LINE__);
        goto http_post_photo_end;
    }
    snprintf(payload_header, HTTP_PAYLOAD_HEADER_LEN,
             "--%s\r\nContent-Disposition: form-data; name=\"image\"; "
             "filename=\"%s\"\r\nContent-Type: image/jpeg\r\n\r\n",
             boundary, HTTP_PAYLOAD_HEADER_FILENAME);

    payload_footer = calloc(HTTP_PAYLOAD_FOOTER_LEN, sizeof(char));
    if (payload_footer == NULL) {
        esp_err = ESP_ERR_NO_MEM;
        esp_err_print(ESP_ERR_NO_MEM, __func__, __LINE__);
        goto http_post_photo_end;
    }
    snprintf(payload_footer, HTTP_PAYLOAD_FOOTER_LEN, "\r\n--%s--\r\n",
             boundary);

    int content_length =
        strlen(payload_header) + fb->len + strlen(payload_footer);
    payload_length = calloc(HTTP_PAYLOAD_LENGTH_LEN, sizeof(char));
    if (payload_length == NULL) {
        esp_err = ESP_ERR_NO_MEM;
        esp_err_print(ESP_ERR_NO_MEM, __func__, __LINE__);
        goto http_post_photo_end;
    }
    snprintf(payload_length, HTTP_PAYLOAD_LENGTH_LEN, "%d", content_length);

    ESP_LOGI(TAG, "payloadLength:\n%s", payload_length);
    ESP_LOGI(TAG, "payloadHeader:\n%s", payload_header);
    ESP_LOGI(TAG, "payloadFooter:\n%s", payload_footer);

    // esp_http_client_open -> esp_http_client_write ->
    // esp_http_client_fetch_headers -> esp_http_client_read (and option)
    // esp_http_client_close.
    esp_http_client_set_header(client, "Host", SERVER_URL);
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "Connection", "close");
    esp_http_client_set_header(client, "Content-Type", content_type);
    esp_http_client_set_header(client, "Content-Length", payload_length);
    esp_http_client_set_url(client, HTTP_IMAGE_HELPLER_URL);
    esp_http_client_set_method(client, HTTP_METHOD_POST);

    if ((esp_err = esp_http_client_open(client, content_length)) != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto http_post_photo_end;
    }
    client_open = true;
    ESP_LOGI(TAG, "http client open");

    if ((client_rd_len = esp_http_client_write(client, payload_header,
                                               strlen(payload_header))) < 0) {
        esp_err = ESP_FAIL;
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_photo_end;
    }
    ESP_LOGI(TAG, "http client write header, length: %d", client_rd_len);

    if ((client_rd_len = esp_http_client_write(client, (const char *)fb->buf,
                                               (fb->len))) < 0) {
        esp_err = ESP_FAIL;
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_photo_end;
    }
    ESP_LOGI(TAG, "http client write playload, length: %d", client_rd_len);

    if ((client_rd_len = esp_http_client_write(client, payload_footer,
                                               strlen(payload_footer))) < 0) {
        esp_err = ESP_FAIL;
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_photo_end;
    }
    ESP_LOGI(TAG, "http client write footer, length: %d", client_rd_len);

    // fetch json string example:
    // {"msg": "Upload Success", "result": "Success", "image": {"name":
    // "victor-test.jpg", "size": 31285, "content_type": "image/jpeg", "url":
    // "http://lulupet.williamhsu.com.tw/media/7881a26566c74127b8e97a2632596d32.jpg"}}
    if ((content_length = esp_http_client_fetch_headers(client)) < 0) {
        esp_err = ESP_FAIL;
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_photo_end;
    }
    ESP_LOGI(TAG, "http client fetech header, length: %d", content_length);

    int read_len;
    json_str = calloc(MAX_HTTP_RECV_BUFFER + 1, sizeof(char));
    if (json_str == NULL) {
        esp_err = ESP_FAIL;
        esp_err_print(ESP_ERR_NO_MEM, __func__, __LINE__);
        goto http_post_photo_end;
    }

    if (content_length > MAX_HTTP_RECV_BUFFER) {
        esp_err = ESP_FAIL;
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_photo_end;
    }

    // TODO: use while loop to read data with retry
    if ((read_len = esp_http_client_read(client, json_str, content_length)) <=
        0) {
        esp_err = ESP_FAIL;
        ESP_LOGE(TAG, "Error read data");
    }
    ESP_LOGI(TAG, "read_len: %d", read_len);
    ESP_LOGI(TAG, "http client read: %s", json_str);

    json_root = cJSON_Parse(json_str);

    if (json_root == NULL) {
        esp_err = ESP_FAIL;
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_photo_end;
    }
    cJSON *json_image_obj = cJSON_GetObjectItem(json_root, "image");
    if (json_image_obj == NULL) {
        esp_err = ESP_FAIL;
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_photo_end;
    }
    cJSON *json_url_obj = cJSON_GetObjectItem(json_image_obj, "url");
    if (json_url_obj == NULL) {
        esp_err = ESP_FAIL;
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto http_post_photo_end;
    }

    snprintf(json_url_val, json_url_val_len, "%s",
             cJSON_GetStringValue(json_url_obj));
    ESP_LOGI(TAG, "read url:%s", json_url_val);

http_post_photo_end:
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

    return esp_err;
}

static void http_post_rawdata(esp_http_client_handle_t client,
                              char *json_url_val, time_t timestamp,
                              weight_take_photo_event_t *take_photo_event) {
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
             "lid=%s&token=%s&eventid=%d&weight=%u&pir=%d&pic=%s&tt=%ld",
             app_wifi_get_lid(), app_wifi_get_token(),
             take_photo_event->eventid, take_photo_event->weight_g,
             take_photo_event->pir_val, json_url_val, timestamp);
    ESP_LOGI(TAG, "post data:\n%s", post_data_raw);

    esp_http_client_set_url(client, HTTP_RAW_DATA_URL);
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

static void http_send_photo_buf(httpc_photo_buf_t *photo_buf,
                                esp_http_client_handle_t client,
                                char *json_url_val, int json_url_val_len) {
    esp_err_t esp_err;
    for (int i = 0; i < HTTP_POST_IMAGEHELPER_RETRY; ++i) {
        if ((esp_err =
                 http_post_imageHelper(client, json_url_val, JSON_URL_VAL_LEN,
                                       photo_buf->fb)) == ESP_OK)
            break;
    }

    if (esp_err != ESP_OK) {
        snprintf(json_url_val, json_url_val_len, "%s_%d",
                 HTTP_POST_IMAGEHELPER_RETRY_FAILED_STR, task_conf.err_line);
    }

    http_post_rawdata(client, json_url_val, photo_buf->unix_timestamp,
                      &photo_buf->weight_take_photo_evt);
}

static int http_get_ota_update_config_latest(httpc_ota_event_t *event,
                                             char *cfg_url, int cfg_url_len,
                                             char *ver_str, int ver_str_len) {
    ESP_LOGI(TAG, "Free Heap Internal is:  %d Byte",
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "Free Heap PSRAM    is:  %d Byte",
             heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    int ret = -1;
    char *json_str = NULL;
    bool client_open = false;
    int content_length;
    int read_len;
    esp_err_t esp_err;
    cJSON *rsp_root_obj = NULL;

    esp_http_client_config_t config = {
        .url = HTTP_OTA_UPDATE_CONFIG_LATEST_URL,
        .event_handler = http_event_handler,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }
    ESP_LOGI(TAG, "http client init");

    esp_http_client_set_method(client, HTTP_METHOD_GET);
    esp_http_client_set_header(client, "accept", "application/json");
    esp_http_client_set_header(client, "token", "");
    esp_http_client_set_header(client, "lid", app_wifi_get_lid());
    esp_http_client_set_header(client, "litter-token", app_wifi_get_token());
    esp_http_client_set_header(
        client, "X-CSRFToken",
        "eA8ob2RLGxH6sQ7njh6pokrwNNTxR7gDqpfhPY9VyO8M9B8HZIaMFrKClihBLO39");

    if ((esp_err = esp_http_client_open(client, 0)) != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }
    client_open = true;

    if ((content_length = esp_http_client_fetch_headers(client)) < 0) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }
    ESP_LOGI(TAG, "http client fetech header, length: %d", content_length);

    if (content_length > MAX_HTTP_RECV_BUFFER) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }

    json_str = calloc(MAX_HTTP_RECV_BUFFER + 1, sizeof(char));
    if (json_str == NULL) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }

    if ((read_len = esp_http_client_read(client, json_str, content_length)) <=
        0) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }
    ESP_LOGI(TAG, "http client read json: %s", json_str);
    ESP_LOGI(TAG, "read_len: %d", read_len);

    // {"msg": "Request Successful, download latest config file version: v1.1",
    // "result": "Success", "data": {"id": 2, "version": "v1.1", "enable": true,
    // "latest": true, "file_url":
    // "http://lulupet.williamhsu.com.tw/media/config_file/weight.cfg",
    // "display_tag": ""}}
    rsp_root_obj = cJSON_Parse(json_str);
    if (rsp_root_obj == NULL) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }

    cJSON *data_obj = cJSON_GetObjectItemCaseSensitive(rsp_root_obj, "data");
    if (data_obj == NULL) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }

    // get version
    cJSON *version_obj = cJSON_GetObjectItemCaseSensitive(data_obj, "version");
    if (version_obj == NULL) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }
    ESP_LOGI(TAG, "ota version: %s", cJSON_GetStringValue(version_obj));
    if (ver_str == NULL || ver_str_len == 0) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }
    snprintf(ver_str, ver_str_len, "%s", cJSON_GetStringValue(version_obj));

    // get file_url
    cJSON *file_url_obj =
        cJSON_GetObjectItemCaseSensitive(data_obj, "file_url");
    if (file_url_obj == NULL) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }
    ESP_LOGI(TAG, "cfg file_url_obj: %s", cJSON_GetStringValue(file_url_obj));

    if (cfg_url == NULL || cfg_url_len == 0) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }

    snprintf(cfg_url, cfg_url_len, "%s", cJSON_GetStringValue(file_url_obj));

    ret = 0;

_end:
    if (rsp_root_obj) {
        free(rsp_root_obj);
    }
    if (json_str) {
        free(json_str);
    }
    if (client_open) {
        ESP_LOGI(TAG, "http client close");
        esp_http_client_close(client);
    }
    ESP_LOGI(TAG, "http client cleanup");
    esp_http_client_cleanup(client);

    return ret;
}

static int http_get_ota_update_latest(httpc_ota_event_t *event, char *ota_url,
                                      int ota_url_len, char *ver_str,
                                      int ver_str_len) {
    ESP_LOGI(TAG, "Free Heap Internal is:  %d Byte",
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "Free Heap PSRAM    is:  %d Byte",
             heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    int ret = -1;
    char *json_str = NULL;
    bool client_open = false;
    int content_length;
    int read_len;
    esp_err_t esp_err;
    cJSON *rsp_root_obj = NULL;

    esp_http_client_config_t config = {
        .url = HTTP_OTA_UPDATE_LATEST_URL,
        .event_handler = http_event_handler,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }
    ESP_LOGI(TAG, "http client init");

    esp_http_client_set_method(client, HTTP_METHOD_GET);
    esp_http_client_set_header(client, "accept", "application/json");
    esp_http_client_set_header(client, "token", "");
    esp_http_client_set_header(client, "lid", app_wifi_get_lid());
    esp_http_client_set_header(client, "litter-token", app_wifi_get_token());
    esp_http_client_set_header(
        client, "X-CSRFToken",
        "eA8ob2RLGxH6sQ7njh6pokrwNNTxR7gDqpfhPY9VyO8M9B8HZIaMFrKClihBLO39");

    if ((esp_err = esp_http_client_open(client, 0)) != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }
    client_open = true;

    if ((content_length = esp_http_client_fetch_headers(client)) < 0) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }
    ESP_LOGI(TAG, "http client fetech header, length: %d", content_length);

    if (content_length > MAX_HTTP_RECV_BUFFER) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }

    json_str = calloc(MAX_HTTP_RECV_BUFFER + 1, sizeof(char));
    if (json_str == NULL) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }

    if ((read_len = esp_http_client_read(client, json_str, content_length)) <=
        0) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }
    ESP_LOGI(TAG, "http client read json: %s", json_str);
    ESP_LOGI(TAG, "read_len: %d", read_len);

    // {"msg": "Request Success", "result": "Success", "data": {"id": 3,
    // "version": "V0.1.0", "enable": true, "latest": true, "file_url":
    // "http://lulupet.williamhsu.com.tw/media/ota_file/lulupet_fw_v0.2.0.bin",
    // "type": "release", "display_tag": ""}}
    rsp_root_obj = cJSON_Parse(json_str);
    if (rsp_root_obj == NULL) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }

    cJSON *data_obj = cJSON_GetObjectItemCaseSensitive(rsp_root_obj, "data");
    if (data_obj == NULL) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }

    // get version
    cJSON *version_obj = cJSON_GetObjectItemCaseSensitive(data_obj, "version");
    if (version_obj == NULL) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }
    ESP_LOGI(TAG, "ota version: %s", cJSON_GetStringValue(version_obj));
    if (ver_str == NULL || ver_str_len == 0) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }
    snprintf(ver_str, ver_str_len, "%s", cJSON_GetStringValue(version_obj));

    // get file_url
    cJSON *file_url_obj =
        cJSON_GetObjectItemCaseSensitive(data_obj, "file_url");
    if (file_url_obj == NULL) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }
    ESP_LOGI(TAG, "ota file_url_obj: %s", cJSON_GetStringValue(file_url_obj));

    if (ota_url == NULL || ota_url_len == 0) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }

    snprintf(ota_url, ota_url_len, "%s", cJSON_GetStringValue(file_url_obj));

    ret = 0;

_end:
    if (rsp_root_obj) {
        free(rsp_root_obj);
    }
    if (json_str) {
        free(json_str);
    }
    if (client_open) {
        ESP_LOGI(TAG, "http client close");
        esp_http_client_close(client);
    }
    ESP_LOGI(TAG, "http client cleanup");
    esp_http_client_cleanup(client);

    return ret;
}

static esp_err_t http_get_cfg(char *cfg_url) {
    char *cfg_buf = NULL;
    bool client_open = false;
    int content_length;
    esp_err_t esp_err;
    esp_http_client_config_t config = {
        .url = cfg_url,
        .event_handler = http_event_handler,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        esp_err = ESP_FAIL;
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }
    ESP_LOGI(TAG, "http client init");

    esp_http_client_set_method(client, HTTP_METHOD_GET);

    if ((esp_err = esp_http_client_open(client, 0)) != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }
    client_open = true;

    if ((content_length = esp_http_client_fetch_headers(client)) < 0) {
        esp_err = ESP_FAIL;
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }
    ESP_LOGI(TAG, "http client fetech header, length: %d", content_length);

    cfg_buf = calloc(UPDATE_FIRMWARE_BUF_SIZE, sizeof(char));
    if (cfg_buf == NULL) {
        esp_err_print(ESP_FAIL, __func__, __LINE__);
        goto _end;
    }

    int size = esp_http_client_read(client, cfg_buf, content_length);
    ESP_LOGI(TAG, "size: %d", size);
    if (size != content_length) {
        esp_err = ESP_FAIL;
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }

    ESP_LOGI(TAG, "config:\n%s", cfg_buf);

    weight_conf_ver1_t weight_confg_v1;
    if ((esp_err = fs_parser_weight_cfg_v1(cfg_buf, &weight_confg_v1)) !=
        ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
        goto _end;
    }

    weight_update_weight_conf_v1(&weight_confg_v1);

_end:
    if (cfg_buf) {
        free(cfg_buf);
    }
    if (client_open) {
        ESP_LOGI(TAG, "http client close");
        esp_http_client_close(client);
    }
    ESP_LOGI(TAG, "http client cleanup");
    esp_http_client_cleanup(client);
    return esp_err;
}

static int http_ota(char *ota_url) {
    esp_err_t esp_err;
    esp_http_client_config_t config = {
        .url = ota_url,
        .event_handler = http_event_handler,
    };

    board_cam_deinit();
    vTaskDelay(pdMS_TO_TICKS(100));

    set_led_cmd(LED_GREEN_BLUE_1HZ);
    esp_err = esp_https_ota(&config);
    if (esp_err == ESP_OK) {
        set_led_cmd(LED_ALL_OFF);
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Firmware upgrade failed");
        esp_restart();
    }
}

static httpc_photo_buf_t *http_photo_buf_pop_ram(uint8_t idx) {
    return &photo_ring_buf.buf[idx];
}

static bool http_photo_buf_get_loop_ram(void) { return photo_ring_buf.loop; }

static uint8_t http_photo_buf_get_idx_ram(void) { return photo_ring_buf.idx; }

static bool http_photo_buf_exist_ram(void) {
    if (photo_ring_buf.loop == false && photo_ring_buf.idx == 0)
        return false;
    else
        return true;
}

static void http_photo_buf_init(void) {
    for (int i = 0; i < CAM_RING_BUF_SIZE; ++i) {
        httpc_photo_buf_t *photo_buf = &photo_ring_buf.buf[i];
        if (photo_buf->fb != NULL) {
            esp_camera_fb_return(photo_buf->fb);
            photo_buf->fb = NULL;
        }
    }
    photo_ring_buf.idx = 0;
    photo_ring_buf.loop = false;
}

static bool http_photo_buf_exist_fs(void) { return fs_check_photo_cfg_exist(); }

static void save_timeval_into_nvs(void) {
    struct timeval time_val;

    gettimeofday(&time_val, NULL);
    ESP_LOGI(TAG, "tv_sec: %li tv_usec: %lu", time_val.tv_sec,
             time_val.tv_usec);

    sntp_show_time(time_val.tv_sec);
    nvs_write_rtc_timeval(time_val);
}

static void load_timeval_from_nvs(void) {
    struct timeval now;
    if ((nvs_read_rtc_timeval(&now)) == ESP_OK) {
        ESP_LOGW(TAG, "Sync. time with nvs timeval");
        settimeofday(&now, NULL);
    }
}

static void ota_check(bool force) {
    char url[256];
    char ver_str[8];

    if (http_get_ota_update_config_latest(&task_conf.ota_evt, url, sizeof(url),
                                          ver_str, sizeof(ver_str)) != 0) {
        ESP_LOGE(TAG, "L%d", __LINE__);
    } else {
        ESP_LOGI(TAG, "cfg ver_str: %s L%d", ver_str, __LINE__);
        ESP_LOGI(TAG, "cfg url: %s L%d", url, __LINE__);
        if (http_get_cfg(url) != ESP_OK) {
            ESP_LOGE(TAG, "L%d", __LINE__);
        }
    }

    if (http_get_ota_update_latest(&task_conf.ota_evt, url, sizeof(url),
                                   ver_str, sizeof(ver_str)) != 0) {
        ESP_LOGE(TAG, "L%d", __LINE__);
        return;
    }
    ESP_LOGI(TAG, "ota ver_str: %s L%d", ver_str, __LINE__);
    ESP_LOGI(TAG, "ota now ver: V%d.%d.%d L%d", VERSION_MAJOR, VERSION_MINOR,
             VERSION_PATCH, __LINE__);
    ESP_LOGI(TAG, "ota url: %s L%d", url, __LINE__);

    int ver_major = VERSION_MAJOR;
    int ver_minor = VERSION_MINOR;
    int ver_patch = VERSION_PATCH;

    if (sscanf(ver_str, "V%d.%d.%d", &ver_major, &ver_minor, &ver_patch) != 3) {
        ESP_LOGE(TAG, "L%d", __LINE__);
    }

    int get_num =
        sscanf(ver_str, "V%d.%d.%d", &ver_major, &ver_minor, &ver_patch);
    ESP_LOGI(TAG, "get_num: %d", get_num);
    ESP_LOGI(TAG, "ver_major: %d", ver_major);
    ESP_LOGI(TAG, "ver_minor: %d", ver_minor);
    ESP_LOGI(TAG, "ver_patch: %d", ver_patch);

    // check server version is larger that
    bool update = false;
    if (ver_major > VERSION_MAJOR) {
        update = true;
    } else if (ver_major == VERSION_MAJOR) {
        if (ver_minor > VERSION_MINOR) {
            update = true;
        } else if (ver_minor == VERSION_PATCH) {
            if (ver_patch > VERSION_PATCH) {
                update = true;
            }
        }
    }

    ESP_LOGW(TAG, "ota update: %d", update);

    if (update || force) {
        if (http_ota(url) != 0) {
            ESP_LOGE(TAG, "L%d", __LINE__);
        }
    }
}

#if (!FUNC_TESTING_FW) // testing firmware do not do that
static void first_connect_to_wifi(void) {
    ESP_LOGW(TAG, "1st wifi connected");

    uint8_t auto_update = 1;
    nvs_read_auto_update(&auto_update);
    ESP_LOGW(TAG, "auto_update: %d", auto_update);
    if (auto_update) {
        ota_check(false);
    }

    ESP_LOGW(TAG, "upload photo");
    weight_take_photo_event_t event;
    event.eventid = RAWDATA_EVENTID_TEST;
    event.pir_val = board_get_pir_status();
    event.weight_g = weight_get_now_weight_int();
    http_photo_buf_push_ram(&event);
    http_send_photo_process(HTTPC_PHOTO_SRC_RAM);
}
#endif // #if (!FUNC_TESTING_FW)

static void httpc_task(void *pvParameter) {
    httpc_task_config_t *conf = (httpc_task_config_t *)pvParameter;

    task_conf.data_mutex = xSemaphoreCreateMutex();
    esp_event_handler_register_with(conf->evt_loop, LULUPET_EVENT_BASE,
                                    ESP_EVENT_ANY_ID, httpc_loop_event_handler,
                                    NULL);

    task_conf.task_enable = true;
    bool wifi_connected = false;
    bool first_wifi_connected = false;
    uint32_t save_timval_into_fs_cnt = 0;
    uint32_t wait_first_wifi_connected_cnt = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(HTTPC_TASK_PERIOD_MS));
        if (pdTRUE == xSemaphoreTake(task_conf.data_mutex, portMAX_DELAY)) {

            if (app_wifi_check_connect(10)) {
                wifi_connected = true;
                if (first_wifi_connected == false && app_wifi_check_sntp()) {
                    first_wifi_connected = true;
#if (!FUNC_TESTING_FW) // testing firmware do not do that
                    save_timeval_into_nvs();
                    first_connect_to_wifi();
#endif
                }
            } else {
                wifi_connected = false;
                if (first_wifi_connected == false &&
                    (wait_first_wifi_connected_cnt++ ==
                     (HTTPC_WAIT_FIRST_CONNTECTED_WIFI_MS /
                      HTTPC_TASK_PERIOD_MS))) {
                    ESP_LOGW(TAG,
                             "can not connect to wifi, sync. time from nvs");
                    load_timeval_from_nvs();
                }
            }

            if (task_conf.weight_event_update) {
                if (wifi_connected) {
                    http_photo_buf_push_ram(&task_conf.weight_take_photo_evt);
                } else {
                    http_photo_buf_push_fs(&task_conf.weight_take_photo_evt);
                }

                task_conf.weight_event_update = false;
            }

            if (task_conf.ota_event_update) {
                if (wifi_connected) {
                    ota_check(true);
                } else {
                    ESP_LOGE(TAG, "wifi disconnect");
                }
                task_conf.ota_event_update = false;
            }

            if (wifi_connected) {
                if (http_photo_buf_exist_ram()) {
                    http_send_photo_process(HTTPC_PHOTO_SRC_RAM);
                }
                if (http_photo_buf_exist_fs()) {
                    http_send_photo_process(HTTPC_PHOTO_SRC_FS);
                }
            }
#if (!FUNC_PHOTO_RINGBUFFER)
            if (!wifi_connected) {
                if (http_photo_buf_exist_ram()) {
                    http_photo_buf_init();
                }
            }
#endif
            if (save_timval_into_fs_cnt++ ==
                (HTTPC_SAVE_TIMEVAL_INTO_NVS_MS / HTTPC_TASK_PERIOD_MS)) {
                save_timval_into_fs_cnt = 0;
                save_timeval_into_nvs();
            }

            xSemaphoreGive(task_conf.data_mutex);
        }
    }
}

void start_httpc_task(esp_event_loop_handle_t event_loop) {
    task_conf.evt_loop = event_loop;
    xTaskCreate(&httpc_task, "httpc_task", 8192, (void *)&task_conf, 5, NULL);
}

esp_err_t httpc_ota_post_event(esp_event_loop_handle_t event_loop) {
    if (event_loop == NULL) {
        goto _err;
    }

    if (task_conf.task_enable == false) {
        esp_err_print(ESP_ERR_NOT_FOUND, __func__, __LINE__);
        goto _err;
    }

    httpc_ota_event_t event;

    esp_err_t esp_err =
        esp_event_post_to(event_loop, LULUPET_EVENT_BASE, LULUPET_EVENT_OTA,
                          &event, sizeof(event), pdMS_TO_TICKS(1000));

    return esp_err;

_err:
    ESP_LOGE(TAG, "%s L%d", esp_err_to_name(ESP_ERR_INVALID_ARG), __LINE__);
    return ESP_ERR_INVALID_ARG;
}

void http_photo_buf_push_ram(weight_take_photo_event_t *take_photo_event) {
    ESP_LOGI(TAG, "Photo ring buffer push in idx[%u] loop[%d] L%d",
             photo_ring_buf.idx, photo_ring_buf.loop, __LINE__);

    httpc_photo_buf_t *photo_buf = &photo_ring_buf.buf[photo_ring_buf.idx];
    if (photo_ring_buf.loop) {
        // delete original buf
        if (photo_buf->fb) {
            esp_camera_fb_return(photo_buf->fb);
            photo_buf->fb = NULL;
        }
    }

    memcpy(&photo_buf->weight_take_photo_evt, take_photo_event,
           sizeof(weight_take_photo_event_t));
    photo_buf->unix_timestamp = time(NULL);
    camera_take_photo(&photo_buf->fb);
    if (photo_buf->fb->format != PIXFORMAT_JPEG) {
        ESP_LOGE(TAG, "camera use the %d format",
                 photo_buf->fb->format); // TODO: record into fetal error nvs
        esp_camera_fb_return(photo_buf->fb);
        return;
    }

    ++photo_ring_buf.idx;
    if (photo_ring_buf.idx == CAM_RING_BUF_SIZE) {
        photo_ring_buf.loop = true;
        photo_ring_buf.idx = 0;
    }

    ESP_LOGI(TAG, "Photo ring buffer push out idx[%u] loop[%d] L%d",
             photo_ring_buf.idx, photo_ring_buf.loop, __LINE__);
}

void http_photo_buf_push_fs(weight_take_photo_event_t *take_photo_event) {
    time_t unix_timestamp;
    camera_fb_t *fb = NULL;
    ESP_LOGI(TAG, "%s start", __func__);
    ESP_LOGI(TAG, "sizeof(camera_fb_t): %d", sizeof(camera_fb_t));
    unix_timestamp = time(NULL);
    camera_take_photo(&fb);
    if (fb->format != PIXFORMAT_JPEG) {
        ESP_LOGE(TAG, "camera use the %d format",
                 fb->format); // TODO: record into fetal error nvs
        esp_camera_fb_return(fb);
        return;
    }
    fs_save_photo(take_photo_event, unix_timestamp, fb);
    camera_return_photo(&fb);
    ESP_LOGI(TAG, "%s end", __func__);
    return;
}

esp_err_t http_photo_buf_pop_fs(httpc_photo_buf_t *photo_buf) {
    photo_buf->fb = calloc(1, sizeof(camera_fb_t));
    if (photo_buf->fb == NULL) {
        return esp_err_print(ESP_ERR_NO_MEM, __func__, __LINE__);
    }

    return fs_load_photo(&photo_buf->weight_take_photo_evt,
                         &photo_buf->unix_timestamp, photo_buf->fb);
}

void http_send_photo_process(httpc_photo_source_e photo_src) {
    esp_http_client_config_t config = {
        .url = HTTP_IMAGE_HELPLER_URL,
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
        goto _end;
    }

    httpc_photo_buf_t *photo_buf_ram = NULL;
    if (photo_src == HTTPC_PHOTO_SRC_RAM) {
        // check photo in the ram
        bool buf_loop = http_photo_buf_get_loop_ram();
        uint8_t buf_start_idx = http_photo_buf_get_idx_ram();

        if (buf_loop == false && buf_start_idx == 0) {
            ESP_LOGI(TAG, "without any photos in the buffer");
        } else {
            if (buf_loop) {
                for (int i = buf_start_idx; i < CAM_RING_BUF_SIZE; ++i) {
                    ESP_LOGI(TAG, "Send Photo ram buf[%d]", i);
                    photo_buf_ram = http_photo_buf_pop_ram(i);
                    http_send_photo_buf(photo_buf_ram, client, json_url_val,
                                        JSON_URL_VAL_LEN);
                }
            }
            for (int i = 0; i < buf_start_idx; ++i) {
                ESP_LOGI(TAG, "Send Photo ram buf[%d]", i);
                photo_buf_ram = http_photo_buf_pop_ram(i);
                http_send_photo_buf(photo_buf_ram, client, json_url_val,
                                    JSON_URL_VAL_LEN);
            }
        }
    }

    httpc_photo_buf_t photo_buf_fs;
    photo_buf_fs.fb = NULL;
    if (photo_src == HTTPC_PHOTO_SRC_FS) {
        // check photo in the fs
        if (http_photo_buf_pop_fs(&photo_buf_fs) == ESP_OK) {
            http_send_photo_buf(&photo_buf_fs, client, json_url_val,
                                JSON_URL_VAL_LEN);
        }
    }

_end:
    if (photo_src == HTTPC_PHOTO_SRC_RAM) {
        http_photo_buf_init();
    }
    if (json_url_val) {
        free(json_url_val);
    }
    if (photo_buf_fs.fb != NULL) {
        if (photo_buf_fs.fb->buf != NULL) {
            free(photo_buf_fs.fb->buf);
        }
        free(photo_buf_fs.fb);
    }
    esp_http_client_cleanup(client);
    ESP_LOGI(TAG, "http client cleanup");

    return;
}
