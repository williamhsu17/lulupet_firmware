#include "cJSON.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <string.h>

#include "include/event.h"
#include "include/util.h"

#include "include/task_fs.h"

#define TAG "fs_task"
#define FS_TASK_PERIOD_MS 100

typedef struct {
} fs_task_event_t;

typedef struct {
    esp_event_loop_handle_t evt_loop;

    fs_task_event_t ota_evt;
    SemaphoreHandle_t data_mutex;
} fs_task_config_t;

static fs_task_config_t task_conf;

static esp_err_t esp_err_print(esp_err_t err, const char *func, uint32_t line);
static void fs_event_handler(void *arg, esp_event_base_t base, int32_t event_id,
                             void *event_data);

static void fs_task(void *pvParameter);

static esp_err_t esp_err_print(esp_err_t err, const char *func, uint32_t line) {
    ESP_LOGE(TAG, "err:%s %s():L%d", esp_err_to_name(err), func, line);
    return err;
}

static void fs_event_handler(void *arg, esp_event_base_t base, int32_t event_id,
                             void *event_data) {

    xSemaphoreTake(task_conf.data_mutex, portMAX_DELAY);
    switch (event_id) {
    default:
        break;
    }
    xSemaphoreGive(task_conf.data_mutex);
}

static void fs_task(void *pvParameter) {
    fs_task_config_t *conf = (fs_task_config_t *)pvParameter;

    task_conf.data_mutex = xSemaphoreCreateMutex();
    esp_event_handler_register_with(conf->evt_loop, LULUPET_EVENT_BASE,
                                    ESP_EVENT_ANY_ID, fs_event_handler, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(FS_TASK_PERIOD_MS));
        if (pdTRUE == xSemaphoreTake(task_conf.data_mutex, portMAX_DELAY)) {

            xSemaphoreGive(task_conf.data_mutex);
        }
    }
}

void fs_task_start(esp_event_loop_handle_t event_loop) {
    task_conf.evt_loop = event_loop;
    xTaskCreate(&fs_task, "fs_task", 4096, (void *)&task_conf, 0, NULL);
}
