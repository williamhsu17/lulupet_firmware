#include <string.h>

#include "cJSON.h"
#include "esp_bit_defs.h"
#include "esp_blufi_api.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_gap_ble_api.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "include/app_key.h"
#include "include/app_led.h"
#include "include/app_wifi.h"
#include "include/blufi.h"
#include "include/event.h"
#include "include/util.h"

#define BLUFI_DEVICE_NAME "Lulupet AI Litter Box"
#define BLUFI_CHK_CONN_MS 300000 // TODO: can be set by command/config file

typedef struct {
    esp_event_loop_handle_t evt_handler;
    key_loop_event_t evt_key;
} blufi_loop_event_t;

typedef struct {
    blufi_loop_event_t loop_evt;
} blufi_task_config_t;

static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min = 0x100,
    .adv_int_max = 0x100,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};
static uint8_t ble_service_uuid128[32] = {
    /* LSB
       <-------------------------------------------------------------------------------->
       MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00,
};
static esp_ble_adv_data_t ble_adv_data = {
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
    .p_service_uuid = ble_service_uuid128,
    .flag = 0x6,
};

static blufi_task_config_t task_conf;
static EventGroupHandle_t wifi_event_group;
/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
static const int WIFI_CONNECTED_BIT = BIT0;
/* store the station info for send back to phone */
static bool gl_sta_connected = false;
static uint8_t gl_sta_bssid[6];
static uint8_t gl_sta_ssid[32];
static int gl_sta_ssid_len;
/* lulupet API id and token */
static char lulupet_lid_get[20];
static char lulupet_token_get[180];
static wifi_config_t sta_config;
static wifi_config_t ap_config;
static uint8_t server_if;
static uint16_t conn_id;

static esp_err_t http_event_handler(esp_http_client_event_t *evt);
static void blufi_event_callback(esp_blufi_cb_event_t event,
                                 esp_blufi_cb_param_t *param);
static void blufi_loop_event_handler(void *arg, esp_event_base_t base,
                                     int32_t event_id, void *event_data);
static esp_err_t blufi_wifi_event_handler(void *ctx, system_event_t *event);
static void ble_gap_event_handler(esp_gap_ble_cb_event_t event,
                                  esp_ble_gap_cb_param_t *param);
static void blufi_set_led_cmd(unsigned int led_cmd_load);
static void blufi_wifi_init(void);
static void blufi_init(void);
static bool blufi_wifi_event_check_conn(uint32_t wait_ms);
static void blufi_check_connect(void);
static void blufi_wifi_event_init(void);
static void http_get_enable(void);

static esp_blufi_callbacks_t blufi_callbacks = {
    .event_cb = blufi_event_callback,
    .negotiate_data_handler = blufi_dh_negotiate_data_handler,
    .encrypt_func = blufi_aes_encrypt,
    .decrypt_func = blufi_aes_decrypt,
    .checksum_func = blufi_crc_checksum,
};

static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        BLUFI_INFO("HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        BLUFI_INFO("HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        BLUFI_INFO("HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        BLUFI_INFO("HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key,
                   evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        BLUFI_INFO("HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        if (!esp_http_client_is_chunked_response(evt->client)) {
            // Write out data
            // printf("%.*s", evt->data_len, (char*)evt->data);
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        BLUFI_INFO("HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        BLUFI_INFO("HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

static void blufi_event_callback(esp_blufi_cb_event_t event,
                                 esp_blufi_cb_param_t *param) {
    /* actually, should post to blufi_task handle the procedure,
     * now, as a example, we do it more simply */
    switch (event) {
    case ESP_BLUFI_EVENT_INIT_FINISH:
        esp_ble_gap_set_device_name(BLUFI_DEVICE_NAME);
        esp_ble_gap_config_adv_data(&ble_adv_data);
        BLUFI_INFO("init finish");
        break;
    case ESP_BLUFI_EVENT_DEINIT_FINISH:
        blufi_set_led_cmd(LED_ALL_OFF);
        BLUFI_INFO("deinit finish");
        break;
    case ESP_BLUFI_EVENT_BLE_CONNECT:
        server_if = param->connect.server_if;
        conn_id = param->connect.conn_id;
        esp_ble_gap_stop_advertising();
        blufi_security_init();
        blufi_set_led_cmd(LED_BLUE_SOLID);
        BLUFI_INFO("ble connect");
        break;
    case ESP_BLUFI_EVENT_BLE_DISCONNECT:
        blufi_security_deinit();
        esp_ble_gap_start_advertising(&ble_adv_params);
        esp_wifi_disconnect();
        blufi_set_led_cmd(LED_BLUE_1HZ);
        BLUFI_INFO("ble disconnect");
        break;
    case ESP_BLUFI_EVENT_SET_WIFI_OPMODE:
        ESP_ERROR_CHECK(esp_wifi_set_mode(param->wifi_mode.op_mode));
        BLUFI_INFO("Set WIFI opmode %d", param->wifi_mode.op_mode);
        break;
    case ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP:
        BLUFI_INFO("requset wifi connect to AP");
        /* there is no wifi callback when the device has already connected to
        this wifi so disconnect wifi before connection.
        */
        esp_wifi_disconnect();
        esp_wifi_connect();
        break;
    case ESP_BLUFI_EVENT_REQ_DISCONNECT_FROM_AP:
        BLUFI_INFO("requset wifi disconnect from AP");
        esp_wifi_disconnect();
        break;
    case ESP_BLUFI_EVENT_REPORT_ERROR:
        BLUFI_ERROR("report error, error code %d", param->report_error.state);
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
        BLUFI_INFO("get wifi status from AP");
        break;
    }
    case ESP_BLUFI_EVENT_RECV_SLAVE_DISCONNECT_BLE:
        esp_blufi_close(server_if, conn_id);
        BLUFI_INFO("close a gatt connection");
        break;
    case ESP_BLUFI_EVENT_DEAUTHENTICATE_STA:
        /* TODO */
        break;
    case ESP_BLUFI_EVENT_RECV_STA_BSSID:
        memcpy(sta_config.sta.bssid, param->sta_bssid.bssid, 6);
        sta_config.sta.bssid_set = 1;
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        BLUFI_INFO("recv STA BSSID %s", sta_config.sta.ssid);
        break;
    case ESP_BLUFI_EVENT_RECV_STA_SSID:
        strncpy((char *)sta_config.sta.ssid, (char *)param->sta_ssid.ssid,
                param->sta_ssid.ssid_len);
        sta_config.sta.ssid[param->sta_ssid.ssid_len] = '\0';
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        BLUFI_INFO("recv STA SSID %s", sta_config.sta.ssid);
        break;
    case ESP_BLUFI_EVENT_RECV_STA_PASSWD:
        strncpy((char *)sta_config.sta.password,
                (char *)param->sta_passwd.passwd, param->sta_passwd.passwd_len);
        sta_config.sta.password[param->sta_passwd.passwd_len] = '\0';
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        BLUFI_DEBUG("recv STA PASSWORD %s", sta_config.sta.password);
        break;
    case ESP_BLUFI_EVENT_RECV_SOFTAP_SSID:
        strncpy((char *)ap_config.ap.ssid, (char *)param->softap_ssid.ssid,
                param->softap_ssid.ssid_len);
        ap_config.ap.ssid[param->softap_ssid.ssid_len] = '\0';
        ap_config.ap.ssid_len = param->softap_ssid.ssid_len;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("recv SOFTAP SSID %s, ssid len %d", ap_config.ap.ssid,
                   ap_config.ap.ssid_len);
        break;
    case ESP_BLUFI_EVENT_RECV_SOFTAP_PASSWD:
        strncpy((char *)ap_config.ap.password,
                (char *)param->softap_passwd.passwd,
                param->softap_passwd.passwd_len);
        ap_config.ap.password[param->softap_passwd.passwd_len] = '\0';
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("recv SOFTAP PASSWORD %s len = %d", ap_config.ap.password,
                   param->softap_passwd.passwd_len);
        break;
    case ESP_BLUFI_EVENT_RECV_SOFTAP_MAX_CONN_NUM:
        if (param->softap_max_conn_num.max_conn_num > 4) {
            return;
        }
        ap_config.ap.max_connection = param->softap_max_conn_num.max_conn_num;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("recv SOFTAP MAX CONN NUM %d", ap_config.ap.max_connection);
        break;
    case ESP_BLUFI_EVENT_RECV_SOFTAP_AUTH_MODE:
        if (param->softap_auth_mode.auth_mode >= WIFI_AUTH_MAX) {
            return;
        }
        ap_config.ap.authmode = param->softap_auth_mode.auth_mode;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("recv SOFTAP AUTH MODE %d", ap_config.ap.authmode);
        break;
    case ESP_BLUFI_EVENT_RECV_SOFTAP_CHANNEL:
        if (param->softap_channel.channel > 13) {
            return;
        }
        ap_config.ap.channel = param->softap_channel.channel;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("recv SOFTAP CHANNEL %d", ap_config.ap.channel);
        break;
    case ESP_BLUFI_EVENT_GET_WIFI_LIST: {
        wifi_scan_config_t scanConf = {
            .ssid = NULL, .bssid = NULL, .channel = 0, .show_hidden = false};
        ESP_ERROR_CHECK(esp_wifi_scan_start(&scanConf, true));
        break;
    }
    case ESP_BLUFI_EVENT_RECV_CUSTOM_DATA: {
        BLUFI_INFO("recv custom data length[%d]", param->custom_data.data_len);
        char *custom_data_buf = malloc(param->custom_data.data_len + 1);
        if (!custom_data_buf) {
            BLUFI_ERROR("malloc failed %s:L%d", __func__, __LINE__);
            break;
        }
        snprintf(custom_data_buf, param->custom_data.data_len + 1, "%s",
                 param->custom_data.data);
        BLUFI_INFO("custom_data_buf: %s", custom_data_buf);
        cJSON *pJsonRoot = cJSON_Parse(custom_data_buf);
        if (NULL != pJsonRoot) {
            cJSON *plid = cJSON_GetObjectItem(pJsonRoot, "lid");
            if (NULL != plid) {
                if (cJSON_IsString(plid)) {
                    sprintf(lulupet_lid_get, "%s", plid->valuestring);
                    BLUFI_INFO("read lid:%s", lulupet_lid_get);
                } else
                    BLUFI_ERROR("lid is not string");
            } else
                BLUFI_ERROR("get object lid fail");

            cJSON *ptoken = cJSON_GetObjectItem(pJsonRoot, "token");
            if (NULL != ptoken) {
                if (cJSON_IsString(ptoken)) {
                    sprintf(lulupet_token_get, "%s", ptoken->valuestring);
                    BLUFI_INFO("read token:%s", lulupet_token_get);
                } else
                    BLUFI_ERROR("token is not string");
            } else
                BLUFI_ERROR("get object token fail");
        } else {
            BLUFI_ERROR("json parse fail");
        }
        if (custom_data_buf) {
            free(custom_data_buf);
        }
        break;
    }
    case ESP_BLUFI_EVENT_RECV_USERNAME:
        BLUFI_INFO("recv username");
        /* Not handle currently */
        break;
    case ESP_BLUFI_EVENT_RECV_CA_CERT:
        BLUFI_INFO("recv CA CERT");
        /* Not handle currently */
        break;
    case ESP_BLUFI_EVENT_RECV_CLIENT_CERT:
        BLUFI_INFO("recv CLIENT CERT");
        /* Not handle currently */
        break;
    case ESP_BLUFI_EVENT_RECV_SERVER_CERT:
        BLUFI_INFO("recv SERVER CERT");
        /* Not handle currently */
        break;
    case ESP_BLUFI_EVENT_RECV_CLIENT_PRIV_KEY:
        BLUFI_INFO("recv CLIENT PRIV KEY");
        /* Not handle currently */
        break;
    case ESP_BLUFI_EVENT_RECV_SERVER_PRIV_KEY:
        BLUFI_INFO("recv SERVER PRIV KEY");
        /* Not handle currently */
        break;
    default:
        BLUFI_WARNING("recv not support event: %d", event);
        break;
    }
}

static void blufi_loop_event_handler(void *arg, esp_event_base_t base,
                                     int32_t event_id, void *event_data) {
    switch (event_id) {
    case LULUPET_EVENT_KEY:
        memcpy(&task_conf.loop_evt.evt_key, (key_loop_event_t *)event_data,
               sizeof(key_loop_event_t));
        BLUFI_WARNING(
            "key event: %s",
            app_key_event_type_str(task_conf.loop_evt.evt_key.key_event_type));
        break;
    default:
        break;
    }
}

static esp_err_t blufi_wifi_event_handler(void *ctx, system_event_t *event) {
    wifi_mode_t mode;

    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        BLUFI_INFO("wifi STA start");
        break;
    case SYSTEM_EVENT_STA_GOT_IP: {
        esp_blufi_extra_info_t info;

        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_get_mode(&mode);
        memset(&info, 0, sizeof(esp_blufi_extra_info_t));
        memcpy(info.sta_bssid, gl_sta_bssid, 6);
        info.sta_bssid_set = true;
        info.sta_ssid = gl_sta_ssid;
        info.sta_ssid_len = gl_sta_ssid_len;
        esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0,
                                        &info);
        BLUFI_INFO("wifi STA got IP");
        break;
    }
    case SYSTEM_EVENT_STA_CONNECTED:
        gl_sta_connected = true;
        memcpy(gl_sta_bssid, event->event_info.connected.bssid, 6);
        memcpy(gl_sta_ssid, event->event_info.connected.ssid,
               event->event_info.connected.ssid_len);
        gl_sta_ssid_len = event->event_info.connected.ssid_len;
        BLUFI_INFO("wifi STA conneted");
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        gl_sta_connected = false;
        memset(gl_sta_ssid, 0, 32);
        memset(gl_sta_bssid, 0, 6);
        gl_sta_ssid_len = 0;
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        BLUFI_INFO("wifi STA disconneted");
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
        BLUFI_INFO("wifi AP start");
        break;
    case SYSTEM_EVENT_SCAN_DONE: {
        uint16_t apCount = 0;
        esp_wifi_scan_get_ap_num(&apCount);
        if (apCount == 0) {
            BLUFI_ERROR("Nothing AP found");
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
        if (ap_list) {
            free(ap_list);
        }
        if (blufi_ap_list) {
            free(blufi_ap_list);
        }
        BLUFI_INFO("wifi scan done");
        break;
    }
    default:
        BLUFI_INFO("wifi didn't support event: %d", event->event_id);
        break;
    }
    return ESP_OK;
}

static void ble_gap_event_handler(esp_gap_ble_cb_event_t event,
                                  esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&ble_adv_params);
        blufi_set_led_cmd(LED_BLUE_1HZ);
        break;
    default:
        break;
    }
}

static void blufi_set_led_cmd(unsigned int led_cmd_load) {
    xQueueSend(led_cmd_que, (void *)&led_cmd_load, (TickType_t)0);
}

static void blufi_wifi_init(void) {
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(blufi_wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void blufi_init(void) {
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        BLUFI_ERROR("%s:L%d initialize bt controller failed: %s", __func__,
                    __LINE__, esp_err_to_name(ret));
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        BLUFI_ERROR("%s:L%d enable bt controller failed: %s", __func__,
                    __LINE__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        BLUFI_ERROR("%s:L%d init bluedroid failed: %s", __func__, __LINE__,
                    esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        BLUFI_ERROR("%s:L%d init bluedroid failed: %s", __func__, __LINE__,
                    esp_err_to_name(ret));
        return;
    }

    BLUFI_INFO("BD ADDR: " ESP_BD_ADDR_STR "",
               ESP_BD_ADDR_HEX(esp_bt_dev_get_address()));

    BLUFI_INFO("BLUFI VERSION %04x", esp_blufi_get_version());

    ret = esp_ble_gap_register_callback(ble_gap_event_handler);
    if (ret) {
        BLUFI_ERROR("%s:L%d gap register failed, error code = %x", __func__,
                    __LINE__, ret);
        return;
    }

    ret = esp_blufi_register_callbacks(&blufi_callbacks);
    if (ret) {
        BLUFI_ERROR("%s:L%d blufi register failed, error code = %x", __func__,
                    __LINE__, ret);
        return;
    }
}

static bool blufi_wifi_event_check_conn(uint32_t wait_ms) {
    EventBits_t wifi_event_bits;

    wifi_event_bits =
        xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true,
                            wait_ms / portTICK_PERIOD_MS);
    BLUFI_INFO("wifi_event_bits: 0x%x", wifi_event_bits);
    if ((wifi_event_bits & WIFI_CONNECTED_BIT)) {
        return true;
    } else {
        return false;
    }
}

static void blufi_check_connect(void) {

    blufi_loop_event_t *evt = &task_conf.loop_evt;

    blufi_wifi_init();
    blufi_init();
    while (1) {
        esp_blufi_profile_init(); // start blufi funciton
        if (blufi_wifi_event_check_conn(BLUFI_CHK_CONN_MS))
            return;

        BLUFI_WARNING(
            "blufi can't connect to AP, wait %s",
            app_key_event_type_str(KEY_EVENT_PRESS_2_TIMES_WITHIN_3_SEC));
        esp_blufi_profile_deinit(); // stop blufi funciton

        while (1) {
            BLUFI_DEBUG("wait key event");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            if (evt->evt_key.key_event_type ==
                KEY_EVENT_PRESS_2_TIMES_WITHIN_3_SEC) {
                BLUFI_INFO("receive key event: %s",
                           app_key_event_type_str(evt->evt_key.key_event_type));
                evt->evt_key.key_event_type = KEY_EVENT_NONE; // reset key event
                break;
            }
        }
    }
}

static void blufi_wifi_event_init(void) {
    wifi_event_group = xEventGroupCreate();
    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
}

static void http_get_enable(void) {

    esp_http_client_config_t config = {
        .url = HTTP_ENABLE_URL,
        .event_handler = http_event_handler,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    BLUFI_INFO("http client init");

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
        BLUFI_ERROR("Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return;
    }
    BLUFI_INFO("http client open");
    int content_length = esp_http_client_fetch_headers(client);
    BLUFI_INFO("http client fetech, length: %d", content_length);
    int total_read_len = 0, read_len;
    char *buffer = calloc(MAX_HTTP_RECV_BUFFER + 1, sizeof(char));
    if (total_read_len < content_length &&
        content_length <= MAX_HTTP_RECV_BUFFER) {
        read_len = esp_http_client_read(client, buffer, content_length);
        if (read_len <= 0) {
            BLUFI_ERROR("Error read data");
        }
        buffer[read_len] = 0;
        BLUFI_INFO("read_len: %d", read_len);
        BLUFI_INFO("http client read: %s", buffer);
    }

    esp_http_client_close(client);
    BLUFI_INFO("http client cleanup");
}

void blufi_start(esp_event_loop_handle_t event_loop) {
    BLUFI_INFO("app_key_main start");

    task_conf.loop_evt.evt_handler = event_loop;
    task_conf.loop_evt.evt_key.key_event_type = KEY_EVENT_NONE;

    esp_event_handler_register_with(task_conf.loop_evt.evt_handler,
                                    LULUPET_EVENT_BASE, ESP_EVENT_ANY_ID,
                                    blufi_loop_event_handler, NULL);
    blufi_wifi_event_init();
    BLUFI_INFO("w/o WiFi configuration in NVS. run blufi");
    blufi_check_connect();
    BLUFI_INFO("connected to AP");
    vTaskDelay(500 / portTICK_PERIOD_MS);
    BLUFI_INFO("enable lid and token");
    http_get_enable();
    nvs_write_lid_token();
    BLUFI_INFO("store WiFi setting to NVS");
    nvs_set_wifi_val();
    blufi_set_led_cmd(LED_ALL_OFF);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    BLUFI_WARNING("reboot");
    esp_restart();
}