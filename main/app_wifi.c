/* ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in
 * which case, it is free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the "Software"), to deal in the
 * Software without restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "app_wifi.h"
#include "../../json/cJSON/cJSON.h"
#include "blufi_example.h"
#include "esp_attr.h"
#include "esp_blufi_api.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_camera.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_gap_ble_api.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "fb_gfx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "img_converters.h"
#include "include/app_led.h"
#include "include/board_driver.h"
#include "include/util.h"
#include "lwip/apps/sntp.h"
#include "lwip/err.h"
#include "mbedtls/base64.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <esp_err.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#define TAG "app_wifi"

#define BLUFI_DEVICE_NAME "BLUFI_DEVICE"
#define DATA_LENGTH 512    /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128 /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS                                            \
    1000 /*!< delay time between different test items */

#define DEFAULT_VREF 3300 // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 500 // Multisampling

/* Constants that aren't configurable in menuconfig */
#define SERVER_URL "lulupet.williamhsu.com.tw"
#define HTTP_PHOTO_URL "http://lulupet.williamhsu.com.tw/imageHelper/"
#define HTTPS_PHOTO_URL "https://lulupet.williamhsu.com.tw/imageHelper/"
#define HTTP_RAW_URL "http://lulupet.williamhsu.com.tw/rawdata"
#define HTTPS_ENABLE_URL "https://lulupet.williamhsu.com.tw/litter/enable/"
#define HTTP_ENABLE_URL "http://lulupet.williamhsu.com.tw/litter/enable/"
#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2048
#define DUMMY_SENSOR 1
#define WIFI_LIST_NUM 10

/* NVS storage define */
#define STORAGE_NAMESPACE "storage"
#define NVSWIFISETTING "wificonfig"
#define NVSWIFICHECK "wifichecked"
#define NVSAPPLID "applid"
#define NVSAPPTOKEN "apptoken"

/* Test WiFi STA configuration */
#define EXAMPLE_WIFI_SSID "SlingXCorp"
#define EXAMPLE_WIFI_PASS "25413113"
#define WIFI_TEST_MODE 0

static uint8_t example_service_uuid128[32] = {
    /* LSB
       <-------------------------------------------------------------------------------->
       MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00,
};
// static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23,
// 0x45, 0x56};
static esp_ble_adv_data_t example_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, // slave connection min interval, Time =
                            // min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time =
                            // max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = example_service_uuid128,
    .flag = 0x6,
};
static esp_ble_adv_params_t example_adv_params = {
    .adv_int_min = 0x100,
    .adv_int_max = 0x100,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};
static wifi_config_t sta_config;
static wifi_config_t ap_config;
static uint8_t server_if;
static uint16_t conn_id;
/* FreeRTOS event group to signal when we are connected & ready to make a
 * request */
static EventGroupHandle_t wifi_event_group;
/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;
/* store the station info for send back to phone */
static bool gl_sta_connected = false;
static uint8_t gl_sta_bssid[6];
static uint8_t gl_sta_ssid[32];
static int gl_sta_ssid_len;
char urlbuffer[100];
/* lulupet API id and token */
char lulupet_lid[10] = "lid118";
char lulupet_token[10] = "WebLid118";
char lulupet_lid_get[20];
char lulupet_token_get[180];

extern const char howsmyssl_com_root_cert_pem_start[] asm(
    "_binary_lulupet_com_root_cert_pem_start");
extern const char howsmyssl_com_root_cert_pem_end[] asm(
    "_binary_lulupet_com_root_cert_pem_end");

static void example_event_callback(esp_blufi_cb_event_t event,
                                   esp_blufi_cb_param_t *param);
static void initialise_nvs_wifi(void);
static void set_led_cmd(unsigned int led_cmd_load);
static void obtain_time(void);
static void initialize_sntp(void);
static void check_time_sntp(void);
static void app_httpc_main(void);
static void http_post_task(void *pvParameter);
static void http_get_enable(void);
static void http_post_data(void);
static int32_t wifi_check_nvs(void);
static esp_err_t event_handler(void *ctx, system_event_t *event);
static esp_err_t initial_nvs(void);
static esp_err_t store_wifi_nvs(void);
static esp_err_t store_idtoken_nvs(void);
static esp_err_t clear_wifi_nvs(void);
static esp_err_t read_idtoken_nvs(void);
static esp_err_t load_wifi_nvs(void);

// unused static function
#if 0
static void capture_photo_only(void);
static void http_post_rawdata(void);
static void http_post_photo(void);
static void initialise_test_wifi(void);
static esp_err_t read_wifi_nvs(void);

static void capture_photo_only(void) {

    ESP_LOGI(TAG, "Free Heap Internal is:  %d Byte",
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "Free Heap PSRAM    is:  %d Byte",
             heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    // Camera capture to fb
    camera_fb_t *fb = NULL;

    fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        esp_camera_fb_return(fb);
        ESP_LOGE(TAG, "Resolve Camera problem, reboot system");
        while (1) {
            // Nothing
        }
        return;
    }
    ESP_LOGI(TAG, "Camera capture ok");

    if (fb->format == PIXFORMAT_JPEG) {
        ESP_LOGI(TAG, "Camera capture JPEG");
    } else {
        ESP_LOGI(TAG, "Camera capture RAW");
    }

    esp_camera_fb_return(fb);

    return;
}

static void http_post_rawdata(void) {
    
    // xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true,
    // portMAX_DELAY);

    esp_err_t err;

    esp_http_client_config_t config = {
        .host = SERVER_URL,
        .path = "/rawdata",
        .event_handler = _http_event_handler,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    ESP_LOGI(TAG, "http post raw data");

    // read adc value
    unsigned int *sensor_adc = (unsigned int *)malloc(sizeof(unsigned int));
#if DUMMY_SENSOR
    *sensor_adc = 999;
#else
    i2c_mcp3221_readADC(I2C_MASTER_NUM, sensor_adc);
#endif

// read PIR
#if DUMMY_SENSOR
    int sensor_pir = 1;
#else
    int sensor_pir = gpio_get_level(GPIO_INPUT_PIR);
#endif

    // read unix timestamp
    time_t seconds;
    seconds = time(NULL);

    // const char *post_data =
    // "lid=lid118&token=WebLid118&weight=100&pir=1&pic=http%3A%2F%2Fwww.google.com&tt=1603191727";
    char *post_data = malloc(200);
    sprintf(post_data, "lid=%s&token=%s&weight=%d&pir=%d&pic=%s&tt=%ld",
            lulupet_lid, lulupet_token, *sensor_adc, sensor_pir, urlbuffer,
            seconds);
    ESP_LOGI(TAG, "post data:\r\n%s", post_data);

    esp_http_client_set_url(client, "http://lulupet.williamhsu.com.tw/rawdata");
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "accept", "application/json");
    esp_http_client_set_header(client, "Content-Type",
                               "application/x-www-form-urlencoded");
    esp_http_client_set_header(
        client, "X-CSRFToken",
        "eA8ob2RLGxH6sQ7njh6pokrwNNTxR7gDqpfhPY9VyO8M9B8HZIaMFrKClihBLO39");
    if ((err = esp_http_client_open(client, strlen(post_data))) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s",
                 esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return;
    }
    int wlen = esp_http_client_write(client, post_data, strlen(post_data));
    if (wlen < 0) {
        ESP_LOGE(TAG, "Write failed");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return;
    }
    ESP_LOGI(TAG, "http client write, length =%d", wlen);
    int content_length = esp_http_client_fetch_headers(client);
    if (content_length < 0) {
        ESP_LOGE(TAG, "Failed to fetch header");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return;
    }
    ESP_LOGI(TAG, "http client fetech, length =%d", content_length);
    int total_read_len = 0, read_len;
    char *buffer = malloc(MAX_HTTP_RECV_BUFFER + 1);
    if (total_read_len < content_length &&
        content_length <= MAX_HTTP_RECV_BUFFER) {
        read_len = esp_http_client_read(client, buffer, content_length);
        if (read_len <= 0) {
            ESP_LOGE(TAG, "Error read data");
        }
        ESP_LOGI(TAG, "http client read:%s", buffer);
        buffer[read_len] = 0;
        ESP_LOGI(TAG, "read_len = %d", read_len);
    }

    free(sensor_adc);
    esp_http_client_close(client);
    ESP_LOGI(TAG, "http client close");

    esp_http_client_cleanup(client);
    ESP_LOGI(TAG, "http client cleanup");
}

static void http_post_photo(void) {
    // Camera capture to fb
    camera_fb_t *fb = NULL;

    fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        return;
    }
    ESP_LOGI(TAG, "Camera capture ok");

    size_t fb_len = 0;
    if (fb->format == PIXFORMAT_JPEG) {
        fb_len = fb->len;
        ESP_LOGI(TAG, "Camera capture JPEG");
    } else {
        ESP_LOGI(TAG, "Camera capture RAW");
    }

    // HTTP HEAD process
    int fileSize = fb_len;
    // char contentType[80];
    char *contentType = malloc(80);
    strcpy(
        contentType,
        "multipart/form-data; boundary=----WebKitFormBoundarykqaGaA5tlVdQyckh");
    // char boundary[50] = "----WebKitFormBoundarykqaGaA5tlVdQyckh";
    char *boundary = malloc(50);
    strcpy(boundary, "----WebKitFormBoundarykqaGaA5tlVdQyckh");
    // char payloadHeader[200] = {0};
    char *payloadHeader = malloc(200);
    sprintf(payloadHeader,
            "--%s\r\nContent-Disposition: form-data; name=\"image\"; "
            "filename=\"%s\"\r\nContent-Type: image/jpeg\r\n\r\n",
            boundary, "victor-test.jpg");

    // char payloadFooter[50] = {0};
    char *payloadFooter = malloc(50);
    sprintf(payloadFooter, "\r\n--%s--\r\n", boundary);

    int headLength = strlen(payloadHeader);
    int footerLength = strlen(payloadFooter);
    int contentLength = headLength + fileSize + footerLength;
    ESP_LOGI(TAG, "payloadHeader length =%d", headLength);
    ESP_LOGI(TAG, "picture length =%d", fileSize);
    ESP_LOGI(TAG, "payloadFooter length =%d", footerLength);
    ESP_LOGI(TAG, "contentLength length =%d", contentLength);
    // char payloadLength[10] = {0};
    char *payloadLength = malloc(10);
    sprintf(payloadLength, "%d", contentLength);
    ESP_LOGI(TAG, "payloadLength length =%s", payloadLength);
    ESP_LOGI(TAG, "payloadHeader:\r\n%s", payloadHeader);
    ESP_LOGI(TAG, "payloadFooter:\r\n%s", payloadFooter);

    // HTTP process
    esp_http_client_config_t config = {
        .url = HTTP_PHOTO_URL,
        .event_handler = _http_event_handler,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    ESP_LOGI(TAG, "http client init");

    // esp_http_client_open -> esp_http_client_write ->
    // esp_http_client_fetch_headers -> esp_http_client_read (and option)
    // esp_http_client_close.
    esp_http_client_set_header(client, "Host", SERVER_URL);
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "Connection", "close");
    esp_http_client_set_header(client, "Content-Type", contentType);
    esp_http_client_set_header(client, "Content-Length", payloadLength);
    esp_http_client_set_url(client, config.url);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_err_t err;
    if ((err = esp_http_client_open(client, contentLength)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s",
                 esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return;
    }
    ESP_LOGI(TAG, "http client open");
    int writeres;
    writeres =
        esp_http_client_write(client, payloadHeader, strlen(payloadHeader));
    if (writeres < 0) {
        ESP_LOGE(TAG, "Write failed");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return;
    }
    ESP_LOGI(TAG, "http client write, length =%d", writeres);
    writeres = esp_http_client_write(client, (const char *)fb->buf, (fb->len));
    if (writeres < 0) {
        ESP_LOGE(TAG, "Write failed");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return;
    }
    ESP_LOGI(TAG, "http client write, length =%d", writeres);
    writeres =
        esp_http_client_write(client, payloadFooter, strlen(payloadFooter));
    if (writeres < 0) {
        ESP_LOGE(TAG, "Write failed");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return;
    }
    ESP_LOGI(TAG, "http client write, length =%d", writeres);

    int content_length = esp_http_client_fetch_headers(client);
    if (content_length < 0) {
        ESP_LOGE(TAG, "Failed to fetch header");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return;
    }
    ESP_LOGI(TAG, "http client fetech, length =%d", content_length);
    int total_read_len = 0, read_len;
    // char *buffer = malloc(MAX_HTTP_RECV_BUFFER + 1);
    char *buffer = malloc(MAX_HTTP_RECV_BUFFER + 1);
    if (total_read_len < content_length &&
        content_length <= MAX_HTTP_RECV_BUFFER) {
        read_len = esp_http_client_read(client, buffer, content_length);
        if (read_len <= 0) {
            ESP_LOGE(TAG, "Error read data");
        }
        ESP_LOGI(TAG, "http client read:%s", buffer);
        ESP_LOGI(TAG, "read_len = %d", read_len);

        cJSON *pJsonRoot = cJSON_Parse(buffer);
        if (NULL != pJsonRoot) {
            cJSON *pImage = cJSON_GetObjectItem(pJsonRoot, "image");
            if (NULL != pImage) {
                cJSON *pURL = cJSON_GetObjectItem(pImage, "url");
                if (NULL != pURL) {
                    if (cJSON_IsString(pURL)) {
                        sprintf(urlbuffer, "%s", pURL->valuestring);
                        // ESP_LOGI(TAG, "get url:%s", pURL->valuestring);
                        ESP_LOGI(TAG, "read url:%s", urlbuffer);
                    } else
                        ESP_LOGI(TAG, "url is not string");
                } else
                    ESP_LOGI(TAG, "get object url fail");
            } else
                ESP_LOGI(TAG, "get object image fail");
        } else
            ESP_LOGI(TAG, "json parse fail");

        buffer[read_len] = 0;
    }
    free(buffer);
    esp_http_client_close(client);
    ESP_LOGI(TAG, "http client close");
    esp_http_client_cleanup(client);
    ESP_LOGI(TAG, "http client cleanup");

    esp_camera_fb_return(fb);
}

static void initialise_test_wifi(void) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());

    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
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

static esp_err_t read_wifi_nvs(void) {
    nvs_handle_t handle;
    esp_err_t err;
    int32_t value = 0;
    wifi_config_t wifi_config_stored;
    memset(&wifi_config_stored, 0x0, sizeof(wifi_config_stored));
    uint32_t len = sizeof(wifi_config_stored);

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;

    // Read
    ESP_ERROR_CHECK(nvs_get_i32(handle, NVSWIFICHECK, &value));
    ESP_ERROR_CHECK(
        nvs_get_blob(handle, NVSWIFISETTING, &wifi_config_stored, &len));
    BLUFI_INFO("Read WiFi check tag : %d", value);
    BLUFI_INFO("Read WiFi configure ssid:%s passwd:%s",
               wifi_config_stored.sta.ssid, wifi_config_stored.sta.password);

    // Close
    nvs_close(handle);
    return ESP_OK;
}

#endif

static void set_led_cmd(unsigned int led_cmd_load) {
    xQueueSend(led_cmd_que, (void *)&led_cmd_load, (TickType_t)0);
}

static esp_err_t initial_nvs(void) {
    // Initialize NVS
    esp_err_t err;
    err = nvs_flash_init();
    BLUFI_INFO("NVS init");
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        BLUFI_INFO("NVS error, erase...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    BLUFI_INFO("NVS ok");
    return ESP_OK;
}

static esp_err_t store_idtoken_nvs(void) {
    nvs_handle_t handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;

    // Write
    ESP_ERROR_CHECK(nvs_set_blob(handle, NVSAPPLID, &lulupet_lid_get,
                                 sizeof(lulupet_lid_get)));
    ESP_ERROR_CHECK(nvs_set_blob(handle, NVSAPPTOKEN, &lulupet_token_get,
                                 sizeof(lulupet_token_get)));
    // Commit
    ESP_ERROR_CHECK(nvs_commit(handle));

    // Close
    nvs_close(handle);
    return ESP_OK;
}

static esp_err_t store_wifi_nvs(void) {
    nvs_handle_t handle;
    esp_err_t err;
    int32_t set_value = 1;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;

    // Write
    ESP_ERROR_CHECK(nvs_set_i32(handle, NVSWIFICHECK, set_value));
    ESP_ERROR_CHECK(
        nvs_set_blob(handle, NVSWIFISETTING, &sta_config, sizeof(sta_config)));

    // Commit
    ESP_ERROR_CHECK(nvs_commit(handle));

    // Close
    nvs_close(handle);
    return ESP_OK;
}

static esp_err_t clear_wifi_nvs(void) {
    nvs_handle_t handle;
    esp_err_t err;
    int32_t set_value = 0;
    wifi_config_t wifi_config = {};

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;

    // Write
    ESP_ERROR_CHECK(nvs_set_i32(handle, NVSWIFICHECK, set_value));
    ESP_ERROR_CHECK(nvs_set_blob(handle, NVSWIFISETTING, &wifi_config,
                                 sizeof(wifi_config)));

    // Commit
    ESP_ERROR_CHECK(nvs_commit(handle));

    // Close
    nvs_close(handle);
    return ESP_OK;
}

static esp_err_t read_idtoken_nvs(void) {
    nvs_handle_t handle;
    esp_err_t err;
    uint32_t len;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;

    // Read
    len = sizeof(lulupet_lid_get);
    ESP_ERROR_CHECK(nvs_get_blob(handle, NVSAPPLID, &lulupet_lid_get, &len));
    len = sizeof(lulupet_token_get);
    ESP_ERROR_CHECK(
        nvs_get_blob(handle, NVSAPPTOKEN, &lulupet_token_get, &len));
    BLUFI_INFO("Read lid   : %s", lulupet_lid_get);
    BLUFI_INFO("Read token : %s", lulupet_token_get);

    // Close
    nvs_close(handle);
    return ESP_OK;
}

static esp_err_t load_wifi_nvs(void) {
    nvs_handle_t handle;
    esp_err_t err;
    int32_t value = 0;
    // wifi_config_t wifi_config_stored;
    memset(&sta_config, 0x0, sizeof(sta_config));
    uint32_t len = sizeof(sta_config);

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;

    // Read
    ESP_ERROR_CHECK(nvs_get_i32(handle, NVSWIFICHECK, &value));
    ESP_ERROR_CHECK(nvs_get_blob(handle, NVSWIFISETTING, &sta_config, &len));
    BLUFI_INFO("Read WiFi check tag : %d", value);
    BLUFI_INFO("Read WiFi configure ssid:%s passwd:%s", sta_config.sta.ssid,
               sta_config.sta.password);
    // Close
    nvs_close(handle);
    return ESP_OK;
}

static int32_t wifi_check_nvs(void) {
    nvs_handle_t handle;
    esp_err_t err;
    int32_t value = 0;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        BLUFI_INFO("NVS open error!");
        initial_nvs();
        clear_wifi_nvs();
        return value;
    }

    // Read
    // ESP_ERROR_CHECK ( nvs_get_i32(handle,  NVSWIFICHECK,   &value) );
    err = nvs_get_i32(handle, NVSWIFICHECK, &value);
    if (err != ESP_OK) {
        BLUFI_INFO("NVS NVSWIFICHECK not found");
        BLUFI_INFO("NVS WiFi Parameter create");
        int32_t set_value = 0;
        wifi_config_t wifi_config = {};
        // Write
        ESP_ERROR_CHECK(nvs_set_i32(handle, NVSWIFICHECK, set_value));
        ESP_ERROR_CHECK(nvs_set_blob(handle, NVSWIFISETTING, &wifi_config,
                                     sizeof(wifi_config)));
        nvs_get_i32(handle, NVSWIFICHECK, &value);
    }
    BLUFI_INFO("Read WiFi check tag : %d", value);
    // Close
    nvs_close(handle);
    return value;
}

static void initialise_nvs_wifi(void) {
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {};
    // wifi_config.sta.ssid = sta_config.sta.ssid;
    // wifi_config.sta.password = sta_config.sta.password;
    strcpy((char *)wifi_config.sta.ssid, (char *)sta_config.sta.ssid);
    strcpy((char *)wifi_config.sta.password, (char *)sta_config.sta.password);
    BLUFI_INFO("Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static esp_err_t event_handler(void *ctx, system_event_t *event) {
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

/* connect infor*/
static esp_err_t example_net_event_handler(void *ctx, system_event_t *event) {
    wifi_mode_t mode;

    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP: {
        esp_blufi_extra_info_t info;

        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        esp_wifi_get_mode(&mode);

        memset(&info, 0, sizeof(esp_blufi_extra_info_t));
        memcpy(info.sta_bssid, gl_sta_bssid, 6);
        info.sta_bssid_set = true;
        info.sta_ssid = gl_sta_ssid;
        info.sta_ssid_len = gl_sta_ssid_len;
        esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0,
                                        &info);
        break;
    }
    case SYSTEM_EVENT_STA_CONNECTED:
        gl_sta_connected = true;
        memcpy(gl_sta_bssid, event->event_info.connected.bssid, 6);
        memcpy(gl_sta_ssid, event->event_info.connected.ssid,
               event->event_info.connected.ssid_len);
        gl_sta_ssid_len = event->event_info.connected.ssid_len;
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        gl_sta_connected = false;
        memset(gl_sta_ssid, 0, 32);
        memset(gl_sta_bssid, 0, 6);
        gl_sta_ssid_len = 0;
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_AP_START:
        esp_wifi_get_mode(&mode);

        /* TODO: get config or information of softap, then set to report
         * extra_info */
        if (gl_sta_connected) {
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0,
                                            NULL);
        } else {
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, 0,
                                            NULL);
        }
        break;
    case SYSTEM_EVENT_SCAN_DONE: {
        uint16_t apCount = 0;
        esp_wifi_scan_get_ap_num(&apCount);
        if (apCount == 0) {
            BLUFI_INFO("Nothing AP found");
            break;
        }
        wifi_ap_record_t *ap_list =
            (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * apCount);
        if (!ap_list) {
            BLUFI_ERROR("malloc error, ap_list is NULL");
            break;
        }
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&apCount, ap_list));
        esp_blufi_ap_record_t *blufi_ap_list = (esp_blufi_ap_record_t *)malloc(
            apCount * sizeof(esp_blufi_ap_record_t));
        if (!blufi_ap_list) {
            if (ap_list) {
                free(ap_list);
            }
            BLUFI_ERROR("malloc error, blufi_ap_list is NULL");
            break;
        }
        for (int i = 0; i < apCount; ++i) {
            blufi_ap_list[i].rssi = ap_list[i].rssi;
            memcpy(blufi_ap_list[i].ssid, ap_list[i].ssid,
                   sizeof(ap_list[i].ssid));
        }
        esp_blufi_send_wifi_list(apCount, blufi_ap_list);
        esp_wifi_scan_stop();
        free(ap_list);
        free(blufi_ap_list);
        break;
    }
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void) {
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(example_net_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static esp_blufi_callbacks_t example_callbacks = {
    .event_cb = example_event_callback,
    .negotiate_data_handler = blufi_dh_negotiate_data_handler,
    .encrypt_func = blufi_aes_encrypt,
    .decrypt_func = blufi_aes_decrypt,
    .checksum_func = blufi_crc_checksum,
};

static void example_event_callback(esp_blufi_cb_event_t event,
                                   esp_blufi_cb_param_t *param) {
    /* actually, should post to blufi_task handle the procedure,
     * now, as a example, we do it more simply */
    switch (event) {
    case ESP_BLUFI_EVENT_INIT_FINISH:
        BLUFI_INFO("BLUFI init finish\n");

        esp_ble_gap_set_device_name(BLUFI_DEVICE_NAME);
        esp_ble_gap_config_adv_data(&example_adv_data);
        break;
    case ESP_BLUFI_EVENT_DEINIT_FINISH:
        BLUFI_INFO("BLUFI deinit finish\n");
        break;
    case ESP_BLUFI_EVENT_BLE_CONNECT:
        BLUFI_INFO("BLUFI ble connect\n");
        server_if = param->connect.server_if;
        conn_id = param->connect.conn_id;
        esp_ble_gap_stop_advertising();
        blufi_security_init();
        break;
    case ESP_BLUFI_EVENT_BLE_DISCONNECT:
        BLUFI_INFO("BLUFI ble disconnect\n");
        blufi_security_deinit();
        esp_ble_gap_start_advertising(&example_adv_params);
        break;
    case ESP_BLUFI_EVENT_SET_WIFI_OPMODE:
        BLUFI_INFO("BLUFI Set WIFI opmode %d\n", param->wifi_mode.op_mode);
        ESP_ERROR_CHECK(esp_wifi_set_mode(param->wifi_mode.op_mode));
        break;
    case ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP:
        BLUFI_INFO("BLUFI requset wifi connect to AP\n");
        /* there is no wifi callback when the device has already connected to
        this wifi so disconnect wifi before connection.
        */
        esp_wifi_disconnect();
        esp_wifi_connect();
        break;
    case ESP_BLUFI_EVENT_REQ_DISCONNECT_FROM_AP:
        BLUFI_INFO("BLUFI requset wifi disconnect from AP\n");
        esp_wifi_disconnect();
        break;
    case ESP_BLUFI_EVENT_REPORT_ERROR:
        BLUFI_ERROR("BLUFI report error, error code %d\n",
                    param->report_error.state);
        esp_blufi_send_error_info(param->report_error.state);
        break;
    case ESP_BLUFI_EVENT_GET_WIFI_STATUS: {
        wifi_mode_t mode;
        esp_blufi_extra_info_t info;

        esp_wifi_get_mode(&mode);

        if (gl_sta_connected) {
            memset(&info, 0, sizeof(esp_blufi_extra_info_t));
            memcpy(info.sta_bssid, gl_sta_bssid, 6);
            info.sta_bssid_set = true;
            info.sta_ssid = gl_sta_ssid;
            info.sta_ssid_len = gl_sta_ssid_len;
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0,
                                            &info);
        } else {
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, 0,
                                            NULL);
        }
        BLUFI_INFO("BLUFI get wifi status from AP\n");

        break;
    }
    case ESP_BLUFI_EVENT_RECV_SLAVE_DISCONNECT_BLE:
        BLUFI_INFO("blufi close a gatt connection");
        esp_blufi_close(server_if, conn_id);
        break;
    case ESP_BLUFI_EVENT_DEAUTHENTICATE_STA:
        /* TODO */
        break;
    case ESP_BLUFI_EVENT_RECV_STA_BSSID:
        memcpy(sta_config.sta.bssid, param->sta_bssid.bssid, 6);
        sta_config.sta.bssid_set = 1;
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        BLUFI_INFO("Recv STA BSSID %s\n", sta_config.sta.ssid);
        break;
    case ESP_BLUFI_EVENT_RECV_STA_SSID:
        strncpy((char *)sta_config.sta.ssid, (char *)param->sta_ssid.ssid,
                param->sta_ssid.ssid_len);
        sta_config.sta.ssid[param->sta_ssid.ssid_len] = '\0';
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        BLUFI_INFO("Recv STA SSID %s\n", sta_config.sta.ssid);
        break;
    case ESP_BLUFI_EVENT_RECV_STA_PASSWD:
        strncpy((char *)sta_config.sta.password,
                (char *)param->sta_passwd.passwd, param->sta_passwd.passwd_len);
        sta_config.sta.password[param->sta_passwd.passwd_len] = '\0';
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        BLUFI_INFO("Recv STA PASSWORD %s\n", sta_config.sta.password);
        break;
    case ESP_BLUFI_EVENT_RECV_SOFTAP_SSID:
        strncpy((char *)ap_config.ap.ssid, (char *)param->softap_ssid.ssid,
                param->softap_ssid.ssid_len);
        ap_config.ap.ssid[param->softap_ssid.ssid_len] = '\0';
        ap_config.ap.ssid_len = param->softap_ssid.ssid_len;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("Recv SOFTAP SSID %s, ssid len %d\n", ap_config.ap.ssid,
                   ap_config.ap.ssid_len);
        break;
    case ESP_BLUFI_EVENT_RECV_SOFTAP_PASSWD:
        strncpy((char *)ap_config.ap.password,
                (char *)param->softap_passwd.passwd,
                param->softap_passwd.passwd_len);
        ap_config.ap.password[param->softap_passwd.passwd_len] = '\0';
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("Recv SOFTAP PASSWORD %s len = %d\n", ap_config.ap.password,
                   param->softap_passwd.passwd_len);
        break;
    case ESP_BLUFI_EVENT_RECV_SOFTAP_MAX_CONN_NUM:
        if (param->softap_max_conn_num.max_conn_num > 4) {
            return;
        }
        ap_config.ap.max_connection = param->softap_max_conn_num.max_conn_num;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("Recv SOFTAP MAX CONN NUM %d\n",
                   ap_config.ap.max_connection);
        break;
    case ESP_BLUFI_EVENT_RECV_SOFTAP_AUTH_MODE:
        if (param->softap_auth_mode.auth_mode >= WIFI_AUTH_MAX) {
            return;
        }
        ap_config.ap.authmode = param->softap_auth_mode.auth_mode;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("Recv SOFTAP AUTH MODE %d\n", ap_config.ap.authmode);
        break;
    case ESP_BLUFI_EVENT_RECV_SOFTAP_CHANNEL:
        if (param->softap_channel.channel > 13) {
            return;
        }
        ap_config.ap.channel = param->softap_channel.channel;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("Recv SOFTAP CHANNEL %d\n", ap_config.ap.channel);
        break;
    case ESP_BLUFI_EVENT_GET_WIFI_LIST: {
        wifi_scan_config_t scanConf = {
            .ssid = NULL, .bssid = NULL, .channel = 0, .show_hidden = false};
        ESP_ERROR_CHECK(esp_wifi_scan_start(&scanConf, true));
        break;
    }
    case ESP_BLUFI_EVENT_RECV_CUSTOM_DATA:
        BLUFI_INFO("Recv Custom Data %d\n", param->custom_data.data_len);
        // esp_log_buffer_hex("Custom Data", param->custom_data.data,
        // param->custom_data.data_len);
        char *custimdataBuffer = malloc(200);
        sprintf(custimdataBuffer, "%s", param->custom_data.data);
        cJSON *pJsonRoot = cJSON_Parse(custimdataBuffer);
        if (NULL != pJsonRoot) {
            cJSON *plid = cJSON_GetObjectItem(pJsonRoot, "lid");
            if (NULL != plid) {
                if (cJSON_IsString(plid)) {
                    sprintf(lulupet_lid_get, "%s", plid->valuestring);
                    ESP_LOGI(TAG, "read lid:%s", lulupet_lid_get);
                } else
                    ESP_LOGI(TAG, "lid is not string");
            } else
                ESP_LOGI(TAG, "get object lid fail");

            cJSON *ptoken = cJSON_GetObjectItem(pJsonRoot, "token");
            if (NULL != ptoken) {
                if (cJSON_IsString(ptoken)) {
                    sprintf(lulupet_token_get, "%s", ptoken->valuestring);
                    ESP_LOGI(TAG, "read token:%s", lulupet_token_get);
                } else
                    ESP_LOGI(TAG, "token is not string");
            } else
                ESP_LOGI(TAG, "get object token fail");
        } else
            ESP_LOGI(TAG, "json parse fail");
        free(custimdataBuffer);
        break;
    case ESP_BLUFI_EVENT_RECV_USERNAME:
        /* Not handle currently */
        break;
    case ESP_BLUFI_EVENT_RECV_CA_CERT:
        /* Not handle currently */
        break;
    case ESP_BLUFI_EVENT_RECV_CLIENT_CERT:
        /* Not handle currently */
        break;
    case ESP_BLUFI_EVENT_RECV_SERVER_CERT:
        /* Not handle currently */
        break;
    case ESP_BLUFI_EVENT_RECV_CLIENT_PRIV_KEY:
        /* Not handle currently */
        break;
        ;
    case ESP_BLUFI_EVENT_RECV_SERVER_PRIV_KEY:
        /* Not handle currently */
        break;
    default:
        break;
    }
}

static void example_gap_event_handler(esp_gap_ble_cb_event_t event,
                                      esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&example_adv_params);
        break;
    default:
        break;
    }
}

static void blufi_run(void) {
    esp_err_t ret;

    initialise_wifi();

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        BLUFI_ERROR("%s initialize bt controller failed: %s\n", __func__,
                    esp_err_to_name(ret));
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        BLUFI_ERROR("%s enable bt controller failed: %s\n", __func__,
                    esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        BLUFI_ERROR("%s init bluedroid failed: %s\n", __func__,
                    esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        BLUFI_ERROR("%s init bluedroid failed: %s\n", __func__,
                    esp_err_to_name(ret));
        return;
    }

    BLUFI_INFO("BD ADDR: " ESP_BD_ADDR_STR "\n",
               ESP_BD_ADDR_HEX(esp_bt_dev_get_address()));

    BLUFI_INFO("BLUFI VERSION %04x\n", esp_blufi_get_version());

    ret = esp_ble_gap_register_callback(example_gap_event_handler);
    if (ret) {
        BLUFI_ERROR("%s gap register failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_blufi_register_callbacks(&example_callbacks);
    if (ret) {
        BLUFI_ERROR("%s blufi register failed, error code = %x\n", __func__,
                    ret);
        return;
    }

    esp_blufi_profile_init();
}

static void check_time_sntp(void) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2020 - 1900)) {
        BLUFI_INFO("Time is not set yet. Connecting to WiFi and getting time "
                   "over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }
    char strftime_buf[64];

    // Set timezone to China Standard Time
    setenv("TZ", "CST-8", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    BLUFI_INFO("The current date/time in Taipei is: %s", strftime_buf);

    time_t seconds;
    seconds = time(NULL);
    BLUFI_INFO("Seconds since January 1, 1970 = %ld\n", seconds);
}

static void obtain_time(void) {
    // ESP_ERROR_CHECK( nvs_flash_init() );
    // initialise_wifi();
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true,
                        portMAX_DELAY);
    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int retry_count = 10;
    while (timeinfo.tm_year < (2020 - 1900) && ++retry < retry_count) {
        BLUFI_INFO("Waiting for system time to be set... (%d/%d)", retry,
                   retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    // ESP_ERROR_CHECK( esp_wifi_stop() );
}

static void initialize_sntp(void) {
    BLUFI_INFO("Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

static void app_httpc_main(void) {
    ESP_LOGI(TAG, "start");
    // board_init();    // TODO: why init again?

    xTaskCreate(&http_post_task, "http_post_task", 3072, NULL, 5, NULL);
}

static void http_post_task(void *pvParameter) {
    int i = 0;

    while (i < 300) {
        ESP_LOGI(TAG, "Checking WiFi status");
        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true,
                            portMAX_DELAY);
        ESP_LOGI(TAG, "Start to upload photo");
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

esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
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

static void http_get_enable(void) {

    esp_http_client_config_t config = {
        .url = HTTP_ENABLE_URL,
        .event_handler = _http_event_handler,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    ESP_LOGI(TAG, "http client init");

    // esp_http_client_open -> esp_http_client_write ->
    // esp_http_client_fetch_headers -> esp_http_client_read (and option)
    // esp_http_client_close.
    esp_http_client_set_method(client, HTTP_METHOD_GET);
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "lid", lulupet_lid_get);
    esp_http_client_set_header(client, "token", lulupet_token_get);
    esp_http_client_set_header(
        client, "X-CSRFToken",
        "FPhy0U9ujY0xfAa5DYJtDoWtDTV2kFlgMXSQXqyF1MJDy0f4E4hWHodp9LTE6wMV");

    esp_err_t err;
    if ((err = esp_http_client_open(client, 0)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s",
                 esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return;
    }
    ESP_LOGI(TAG, "http client open");
    int content_length = esp_http_client_fetch_headers(client);
    ESP_LOGI(TAG, "http client fetech, length =%d", content_length);
    int total_read_len = 0, read_len;
    char *buffer = malloc(MAX_HTTP_RECV_BUFFER + 1);
    if (total_read_len < content_length &&
        content_length <= MAX_HTTP_RECV_BUFFER) {
        read_len = esp_http_client_read(client, buffer, content_length);
        if (read_len <= 0) {
            ESP_LOGE(TAG, "Error read data");
        }
        ESP_LOGI(TAG, "http client read:%s", buffer);
        buffer[read_len] = 0;
        ESP_LOGI(TAG, "read_len = %d", read_len);
    }

    esp_http_client_close(client);
    ESP_LOGI(TAG, "http client cleanup");
}

static void http_post_data(void) {

    ESP_LOGI(TAG, "Free Heap Internal is:  %d Byte",
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "Free Heap PSRAM    is:  %d Byte",
             heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    esp_err_t err;

    // Camera capture to fb
    camera_fb_t *fb = NULL;

    fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        esp_camera_fb_return(fb);
        ESP_LOGE(TAG, "Resolve Camera problem, reboot system");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        esp_restart();
        return;
    }
    ESP_LOGI(TAG, "Camera capture ok");

    size_t fb_len = 0;
    if (fb->format == PIXFORMAT_JPEG) {
        fb_len = fb->len;
        ESP_LOGI(TAG, "Camera capture JPEG");
    } else {
        ESP_LOGI(TAG, "Camera capture RAW");
    }

    // read adc value
    unsigned int *sensor_adc = (unsigned int *)malloc(sizeof(unsigned int));
// unsigned int *sensor_adc;
#if DUMMY_SENSOR
    *sensor_adc = 999;
#else
    // TODO: get sensor_adc from app_weight
#endif

// read PIR
#if DUMMY_SENSOR
    int sensor_pir = 1;
#else
    int sensor_pir = gpio_get_level(GPIO_INPUT_PIR);
#endif

    // read unix timestamp
    time_t seconds;
    seconds = time(NULL);

    // HTTP HEAD process
    int fileSize = fb_len;
    const char *contentType =
        "multipart/form-data; boundary=----WebKitFormBoundarykqaGaA5tlVdQyckh";
    const char *boundary = "----WebKitFormBoundarykqaGaA5tlVdQyckh";
    char *payloadHeader = malloc(200);
    sprintf(payloadHeader,
            "--%s\r\nContent-Disposition: form-data; name=\"image\"; "
            "filename=\"%s\"\r\nContent-Type: image/jpeg\r\n\r\n",
            boundary, "victor-test.jpg");
    char *payloadFooter = malloc(50);
    sprintf(payloadFooter, "\r\n--%s--\r\n", boundary);

    int headLength = strlen(payloadHeader);
    int footerLength = strlen(payloadFooter);
    int contentLength = headLength + fileSize + footerLength;
    ESP_LOGI(TAG, "payloadHeader length =%d", headLength);
    ESP_LOGI(TAG, "picture length =%d", fileSize);
    ESP_LOGI(TAG, "payloadFooter length =%d", footerLength);
    ESP_LOGI(TAG, "contentLength length =%d", contentLength);
    char *payloadLength = malloc(5);
    sprintf(payloadLength, "%d", contentLength);
    ESP_LOGI(TAG, "payloadLength length =%s", payloadLength);
    ESP_LOGI(TAG, "payloadHeader:\r\n%s", payloadHeader);
    ESP_LOGI(TAG, "payloadFooter:\r\n%s", payloadFooter);

    // HTTP process
    esp_http_client_config_t config = {
        .url = HTTP_PHOTO_URL,
        .event_handler = _http_event_handler,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    ESP_LOGI(TAG, "http client init");

    // esp_http_client_open -> esp_http_client_write ->
    // esp_http_client_fetch_headers -> esp_http_client_read (and option)
    // esp_http_client_close.
    esp_http_client_set_header(client, "Host", SERVER_URL);
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "Connection", "close");
    esp_http_client_set_header(client, "Content-Type", contentType);
    esp_http_client_set_header(client, "Content-Length", payloadLength);
    esp_http_client_set_url(client, config.url);
    esp_http_client_set_method(client, HTTP_METHOD_POST);

    if ((err = esp_http_client_open(client, contentLength)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s",
                 esp_err_to_name(err));
        esp_http_client_cleanup(client);
        esp_camera_fb_return(fb);
        free(payloadHeader);
        free(payloadFooter);
        free(payloadLength);
        return;
    }
    ESP_LOGI(TAG, "http client open");
    int writeres;
    writeres =
        esp_http_client_write(client, payloadHeader, strlen(payloadHeader));
    if (writeres < 0) {
        ESP_LOGE(TAG, "Write failed");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        esp_camera_fb_return(fb);
        free(payloadHeader);
        free(payloadFooter);
        free(payloadLength);
        return;
    }
    ESP_LOGI(TAG, "http client write, length =%d", writeres);
    writeres = esp_http_client_write(client, (const char *)fb->buf, (fb->len));
    if (writeres < 0) {
        ESP_LOGE(TAG, "Write failed");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        esp_camera_fb_return(fb);
        free(payloadHeader);
        free(payloadFooter);
        free(payloadLength);
        return;
    }
    ESP_LOGI(TAG, "http client write, length =%d", writeres);
    writeres =
        esp_http_client_write(client, payloadFooter, strlen(payloadFooter));
    if (writeres < 0) {
        ESP_LOGE(TAG, "Write failed");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        esp_camera_fb_return(fb);
        free(payloadHeader);
        free(payloadFooter);
        free(payloadLength);
        return;
    }
    ESP_LOGI(TAG, "http client write, length =%d", writeres);

    int content_length = esp_http_client_fetch_headers(client);
    if (content_length < 0) {
        ESP_LOGE(TAG, "Failed to fetch header");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        esp_camera_fb_return(fb);
        free(payloadHeader);
        free(payloadFooter);
        free(payloadLength);
        return;
    }
    ESP_LOGI(TAG, "http client fetech, length =%d", content_length);
    int total_read_len = 0, read_len;
    char *buffer = malloc(MAX_HTTP_RECV_BUFFER + 1);
    char *urlbuffer_get = malloc(100);
    if (total_read_len < content_length &&
        content_length <= MAX_HTTP_RECV_BUFFER) {
        read_len = esp_http_client_read(client, buffer, content_length);
        if (read_len <= 0) {
            ESP_LOGE(TAG, "Error read data");
        }
        ESP_LOGI(TAG, "http client read:%s", buffer);
        ESP_LOGI(TAG, "read_len = %d", read_len);

        cJSON *pJsonRoot = cJSON_Parse(buffer);
        if (NULL != pJsonRoot) {
            cJSON *pImage = cJSON_GetObjectItem(pJsonRoot, "image");
            if (NULL != pImage) {
                cJSON *pURL = cJSON_GetObjectItem(pImage, "url");
                if (NULL != pURL) {
                    if (cJSON_IsString(pURL)) {
                        sprintf(urlbuffer_get, "%s", pURL->valuestring);
                        ESP_LOGI(TAG, "read url:%s", urlbuffer_get);
                    } else
                        ESP_LOGI(TAG, "url is not string");
                } else
                    ESP_LOGI(TAG, "get object url fail");
            } else
                ESP_LOGI(TAG, "get object image fail");
        } else
            ESP_LOGI(TAG, "json parse fail");

        buffer[read_len] = 0;
    }
    esp_http_client_close(client);
    ESP_LOGI(TAG, "http client close");

    esp_camera_fb_return(fb);
    free(payloadHeader);
    free(payloadFooter);
    free(payloadLength);
    free(buffer);

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "http post raw data");
    char *post_data_raw = malloc(200);
    sprintf(post_data_raw, "lid=%s&token=%s&weight=%d&pir=%d&pic=%s&tt=%ld",
            lulupet_lid, lulupet_token, *sensor_adc, sensor_pir, urlbuffer_get,
            seconds);
    ESP_LOGI(TAG, "post data:\r\n%s", post_data_raw);

    esp_http_client_set_url(client, HTTP_RAW_URL);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "accept", "application/json");
    esp_http_client_set_header(client, "Content-Type",
                               "application/x-www-form-urlencoded");
    esp_http_client_set_header(
        client, "X-CSRFToken",
        "eA8ob2RLGxH6sQ7njh6pokrwNNTxR7gDqpfhPY9VyO8M9B8HZIaMFrKClihBLO39");
    if ((err = esp_http_client_open(client, strlen(post_data_raw))) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s",
                 esp_err_to_name(err));
        esp_http_client_cleanup(client);
        free(urlbuffer_get);
        free(sensor_adc);
        free(post_data_raw);
        return;
    }
    int wlen_raw;
    wlen_raw =
        esp_http_client_write(client, post_data_raw, strlen(post_data_raw));
    if (wlen_raw < 0) {
        ESP_LOGE(TAG, "Write failed");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        free(urlbuffer_get);
        free(sensor_adc);
        free(post_data_raw);
        return;
    }
    ESP_LOGI(TAG, "http client write, length =%d", wlen_raw);
    int content_length_raw = esp_http_client_fetch_headers(client);
    if (content_length < 0) {
        ESP_LOGE(TAG, "Failed to fetch header");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        free(urlbuffer_get);
        free(sensor_adc);
        free(post_data_raw);
        return;
    }
    ESP_LOGI(TAG, "http client fetech, length =%d", content_length_raw);
    int total_read_len_raw = 0, read_len_raw;
    char *buffer_raw = malloc(MAX_HTTP_RECV_BUFFER + 1);
    if (total_read_len_raw < content_length_raw &&
        content_length_raw <= MAX_HTTP_RECV_BUFFER) {
        read_len_raw =
            esp_http_client_read(client, buffer_raw, content_length_raw);
        if (read_len_raw <= 0) {
            ESP_LOGE(TAG, "Error read data");
        }
        ESP_LOGI(TAG, "http client read:%s", buffer_raw);
        ESP_LOGI(TAG, "read_len = %d", read_len_raw);
    }

    esp_http_client_close(client);
    ESP_LOGI(TAG, "http client close");

    esp_http_client_cleanup(client);
    ESP_LOGI(TAG, "http client cleanup");

    free(urlbuffer_get);
    free(sensor_adc);
    free(post_data_raw);
    free(buffer_raw);
    return;
}

void app_wifi_main(void) {
    // Creat message queue and LED task
    BLUFI_INFO("Start WiFi");

    // Read WiFi setting from NVS
    // IF YES, connect to WiFi
    // IF NO , run BlueFi
    initial_nvs();
    // set_led_cmd(LED_ALL_OFF);
    if (wifi_check_nvs() == 1) {
        BLUFI_INFO("Load lid token from NVS");
        read_idtoken_nvs();
        BLUFI_INFO("Load WiFi Setting from NVS");
        load_wifi_nvs();
        initialise_nvs_wifi();
        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true,
                            10000 / portTICK_PERIOD_MS);
        if (CONNECTED_BIT != true) {
            BLUFI_INFO("Can't Connected to AP");
            BLUFI_INFO("Clear NVS setting");
            clear_wifi_nvs();
            set_led_cmd(LED_RED_SOLID);
            while (1) {
                vTaskDelay(10000 / portTICK_PERIOD_MS);
            }
        }
        set_led_cmd(LED_GREEN_SOLID);
        BLUFI_INFO("Connected to AP");
        check_time_sntp();
        BLUFI_INFO("Update time from SNTP");
        vTaskDelay(30000 / portTICK_PERIOD_MS);
        BLUFI_INFO("Clear NVS setting");
        clear_wifi_nvs();
    } else {
        BLUFI_INFO("No WiFi Setting in NVS, run BlueFi");
        set_led_cmd(LED_BLUE_2HZ);
        blufi_run();
        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true,
                            portMAX_DELAY);
        BLUFI_INFO("Connected to AP");
        vTaskDelay(500 / portTICK_PERIOD_MS);
        BLUFI_INFO("Enable lid and token");
        http_get_enable();
        store_idtoken_nvs();
        BLUFI_INFO("Store WiFi setting to NVS");
        store_wifi_nvs();
        // TBD
        set_led_cmd(LED_ALL_OFF);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        BLUFI_INFO("Reboot ...");
        esp_restart();
    }

    app_httpc_main();
}
