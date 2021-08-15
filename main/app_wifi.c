#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lwip/apps/sntp.h"

#include <string.h>
#include <time.h>

#include "include/app_key.h"
#include "include/app_led.h"
#include "include/app_wifi.h"
#include "include/blufi.h"
#include "include/board_driver.h"
#include "include/event.h"
#include "include/nvs_op.h"
#include "include/task_httpc.h"
#include "include/util.h"
#include "sdkconfig.h"

#define TAG "app_wifi"

/* Constants that aren't configurable in menuconfig */
#define DUMMY_SENSOR 1

/* Test WiFi STA configuration */
#define WIFI_CHECK_WAIT_TIME_MS 10000
#define WIFI_CHECK_KEY_TIMES_MS 100
#define WIFI_CONN_CHK_MS 10000 // TODO: can be set by command/config file
#define WIFI_CONN_RETRY 4

typedef struct {
    key_loop_event_t key_event;
    bool ntp_done;
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

#if 0 // unused
static esp_err_t esp_err_print(esp_err_t err, const char *func, uint32_t line);
#endif
static void service_connect_event_handler(void *arg, esp_event_base_t base,
                                          int32_t event_id, void *event_data);
static void wifi_event_init(void);
static void wifi_init_from_nvs(void);
static void wifi_check_connect(uint32_t wait_ms, uint8_t retry);
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event);
static void set_led_cmd(unsigned int led_cmd_load);
static void sntp_obtain_time(void);
static void sntp_check(void);
static void sntp_time_check(void);

#if 0 // unused
static esp_err_t esp_err_print(esp_err_t err, const char *func, uint32_t line) {
    ESP_LOGE(TAG, "err:%s %s():L%d", esp_err_to_name(err), func, line);
    return err;
}
#endif

static void set_led_cmd(unsigned int led_cmd_load) {
    xQueueSend(led_cmd_que, (void *)&led_cmd_load, (TickType_t)0);
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

static void wifi_system_reset(void) {
    ESP_LOGW(TAG, "reboot");
    set_led_cmd(LED_ALL_OFF);
    esp_restart();
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
    }

    ESP_LOGI(TAG, "can't connecte to AP[SSID:%s]", sta_config.sta.ssid);
    set_led_cmd(LED_RED_1HZ);

    uint32_t wifi_check_key_time = 0;
    while (1) {
        vTaskDelay(WIFI_CHECK_KEY_TIMES_MS / portTICK_PERIOD_MS);
        wifi_check_key_time += WIFI_CHECK_KEY_TIMES_MS;

        if (task_conn_cb.key_event.key_event_type ==
            KEY_EVENT_PRESS_2_TIMES_WITHIN_3_SEC) {
            ESP_LOGW(
                TAG, "receive key event: %s",
                app_key_event_type_str(task_conn_cb.key_event.key_event_type));
            task_conn_cb.key_event.key_event_type =
                KEY_EVENT_NONE; // reset key event
            nvs_reset_wifi_val();
            vTaskDelay(WIFI_CHECK_KEY_TIMES_MS / portTICK_PERIOD_MS);
            wifi_system_reset();
        }

        if (task_conn_cb.key_event.key_event_type ==
            KEY_EVENT_PRESS_OVER_5_SEC) {
            ESP_LOGW(
                TAG, "receive key event: %s",
                app_key_event_type_str(task_conn_cb.key_event.key_event_type));
            task_conn_cb.key_event.key_event_type =
                KEY_EVENT_NONE; // reset key event
            wifi_system_reset();
        }

        if (wifi_check_key_time >= WIFI_CHECK_WAIT_TIME_MS) {
            wifi_system_reset();
        }
    }
}

static void wifi_event_init(void) {
    wifi_event_group = xEventGroupCreate();
    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
}

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event) {
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(TAG, "-----SYSTEM_EVENT_STA_START-----");
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "-----SYSTEM_EVENT_STA_GOT_IP-----");
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        set_led_cmd(LED_GREEN_SOLID);
        ESP_LOGI(TAG, "connected to AP");
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        ESP_LOGI(TAG, "-----SYSTEM_EVENT_STA_DISCONNECTED-----");
        if (app_wifi_check_connect(10) == true) {
            set_led_cmd(LED_GREEN_1HZ);
        }
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void sntp_time_check(void) {
    time_t now_time;
    struct tm timeinfo;

    time(&now_time);
    localtime_r(&now_time, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2020 - 1900)) {
        ESP_LOGI(TAG,
                 "Time is not set yet. Connecting to WiFi and getting time "
                 "over NTP.");
        sntp_obtain_time();
        // update 'now' variable with current time
        time(&now_time);
    }

    // Set timezone to CST-8
    setenv("TZ", "CST-8", 1);
    tzset();

    struct timeval now;
    gettimeofday(&now, NULL);
    sntp_show_time(now.tv_sec);

    time_t seconds;
    seconds = time(NULL);
    ESP_LOGI(TAG, "Seconds since January 1, 1970 = %ld", seconds);

    task_conn_cb.ntp_done = true;
}

static void sntp_obtain_time(void) {
    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    bool sntp_ok = false;
    const int retry_count = 10;

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true,
                        portMAX_DELAY);

    for (int retry_times = 0; retry_times < 3; ++retry_times) {
        sntp_check();
        // wait for time to be set
        retry = 0;
        for (;;) {
            ESP_LOGI(TAG, "waiting for system time to be set... (%d/%d)", retry,
                     retry_count);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            time(&now);
            localtime_r(&now, &timeinfo);

            if (timeinfo.tm_year >= (2020 - 1900)) {
                sntp_ok = true;
                break;
            }
            if (++retry > retry_count) {
                break;
            }
        }
        if (sntp_ok)
            break;

        sntp_stop();
    }

    if (!sntp_ok) {
        ESP_LOGE(TAG, "sntp sync. failed L%d", __LINE__);
    }
}

static void sntp_check(void) {
    // An application with this initialization code will periodically
    // synchronize the time. The time synchronization period is determined by
    // CONFIG_LWIP_SNTP_UPDATE_DELAY (default value is one hour).
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
    nvs_read_lid_token(lulupet_lid_get, sizeof(lulupet_lid_get),
                       lulupet_token_get,
                       sizeof(lulupet_token_get)); // TODO: error handling
    ESP_LOGI(TAG, "load WiFi Setting from nvs");
    nvs_read_wifi_config(&sta_config); // TODO: error handling
    wifi_init_from_nvs();
    set_led_cmd(LED_GREEN_1HZ);
    wifi_check_connect(WIFI_CONN_CHK_MS, WIFI_CONN_RETRY);
    sntp_time_check();
    ESP_LOGI(TAG, "Update time from SNTP");
}

char *app_wifi_get_lid(void) { return &lulupet_lid_get[0]; }

char *app_wifi_get_token(void) { return &lulupet_token_get[0]; }

bool app_wifi_check_connect(uint32_t wait_ms) {
    EventBits_t wifi_event_bits;

    wifi_event_bits =
        xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true,
                            wait_ms / portTICK_PERIOD_MS);
    ESP_LOGD(TAG, "wifi_event_bits: 0x%x", wifi_event_bits);
    if ((wifi_event_bits & WIFI_CONNECTED_BIT)) {
        return true;
    } else {
        return false;
    }
}

bool app_wifi_check_sntp(void) { return task_conn_cb.ntp_done; }

void app_wifi_main(esp_event_loop_handle_t event_loop) {
    ESP_LOGI(TAG, "start connect process");

    if (!nvs_read_wifichecked()) {
        blufi_start(event_loop);
    } else {
        wifi_start(event_loop);
        start_httpc_task(event_loop);
    }
}
