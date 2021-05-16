#include <string.h>

#include "cJSON.h"
#include "esp_camera.h"
#include "esp_err.h"
#include "esp_http_client.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "include/app_camera.h"
#include "include/app_wifi.h"
#include "include/task_httpc.h"
#include "include/util.h"

#define TAG "httpc_task"
#define HTTPC_TASK_PERIOD_MS 100
#define HTTP_POST_RAW_DATA_LEN 256
#define HTTP_PAYLOAD_HEADER_LEN 200
#define HTTP_PAYLOAD_FOOTER_LEN 50
#define HTTP_PAYLOAD_LENGTH_LEN 10
#define HTTP_PAYLOAD_HEADER_FILENAME "lulupet-cat.jpg"

typedef struct {
    esp_event_loop_handle_t evt_loop;
} httpc_task_config_t;

static httpc_task_config_t task_conf;

static esp_err_t esp_err_print(esp_err_t err, const char *func, uint32_t line);
static esp_err_t http_event_handler(esp_http_client_event_t *evt);
static void http_post_photo(esp_http_client_handle_t client, char *json_url_val,
                            int json_url_val_len, time_t *timestamp);
static void http_post_raw(esp_http_client_handle_t client, char *json_url_val,
                          uint32_t weight, bool pir, time_t timestamp);
static void http_post_data(void);
static void httpc_task(void *pvParameter);

static esp_err_t esp_err_print(esp_err_t err, const char *func, uint32_t line) {
    ESP_LOGE(TAG, "err:%s %s:L%d", esp_err_to_name(err), func, line);
    return err;
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

static void http_post_photo(esp_http_client_handle_t client, char *json_url_val,
                            int json_url_val_len, time_t *timestamp) {
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

    if (fb->format != PIXFORMAT_JPEG) {
        ESP_LOGE(TAG, "camera use the %d format", fb->format);
        return;
    }

    // HTTP HEAD
    const char *content_type =
        "multipart/form-data; boundary=----WebKitFormBoundarykqaGaA5tlVdQyckh";

    const char *boundary = "----WebKitFormBoundarykqaGaA5tlVdQyckh";

    payload_header = calloc(HTTP_PAYLOAD_HEADER_LEN, sizeof(char));
    if (payload_header == NULL) {
        esp_err_print(ESP_ERR_NO_MEM, __func__, __LINE__);
        goto http_post_photo_end;
    }
    snprintf(payload_header, HTTP_PAYLOAD_HEADER_LEN,
             "--%s\r\nContent-Disposition: form-data; name=\"image\"; "
             "filename=\"%s\"\r\nContent-Type: image/jpeg\r\n\r\n",
             boundary, HTTP_PAYLOAD_HEADER_FILENAME);

    payload_footer = calloc(HTTP_PAYLOAD_FOOTER_LEN, sizeof(char));
    if (payload_footer == NULL) {
        esp_err_print(ESP_ERR_NO_MEM, __func__, __LINE__);
        goto http_post_photo_end;
    }
    snprintf(payload_footer, HTTP_PAYLOAD_FOOTER_LEN, "\r\n--%s--\r\n",
             boundary);

    int content_length =
        strlen(payload_header) + fb->len + strlen(payload_footer);
    payload_length = calloc(HTTP_PAYLOAD_LENGTH_LEN, sizeof(char));
    if (payload_length == NULL) {
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
    ESP_LOGI(TAG, "http client fetech header, length: %d", content_length);

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
             "lid=%s&token=%s&weight=%u&pir=%d&pic=%s&tt=%ld",
             app_wifi_get_lid(), app_wifi_get_token(), weight, pir,
             json_url_val, timestamp);
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

static void httpc_task(void *pvParameter) {
    httpc_task_config_t *conf = (httpc_task_config_t *)pvParameter;

    int i = 0;
    while (i < 1) {
        ESP_LOGI(TAG, "checking WiFi status");
        if (!app_wifi_check_connect(1000)) {
            ESP_LOGE(TAG, "wifi disconnect");
            vTaskDelay(pdMS_TO_TICKS(HTTPC_TASK_PERIOD_MS));
            continue;
        }
        ESP_LOGI(TAG, "start to upload photo");
        http_post_data();
        // capture_photo_only();
        ESP_LOGI(TAG, "http post data test : %d ok", i);
        i++;
    }

    ESP_LOGI(TAG, "end");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(HTTPC_TASK_PERIOD_MS));
    }
}

void start_httpc_task(esp_event_loop_handle_t event_loop) {
    task_conf.evt_loop = event_loop;
    xTaskCreate(&httpc_task, "httpc_task", 4096, (void *)&task_conf, 5, NULL);
}