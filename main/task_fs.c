#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_vfs_fat.h"

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

    bool fs_mount;

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

static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

static esp_err_t fs_mount(void) {
    const char *partition_label = "storage";
    const esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 4,
        .format_if_mount_failed = true,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE};
    ESP_LOGW(TAG, "start mount fs");
    esp_err_t esp_err = esp_vfs_fat_spiflash_mount("/fs", partition_label,
                                                   &mount_config, &s_wl_handle);
    ESP_LOGW(TAG, "end mount fs");

    if (esp_err != ESP_OK) {
        esp_err_print(esp_err, __func__, __LINE__);
    }

    // Print FAT FS size information
    size_t bytes_total, bytes_free;
    fs_get_fatfs_usage(&bytes_total, &bytes_free);
    ESP_LOGI(TAG, "FAT FS: %d kB total, %d kB free", bytes_total / 1024,
             bytes_free / 1024);

    return esp_err;
}

static void fs_task(void *pvParameter) {
    fs_task_config_t *conf = (fs_task_config_t *)pvParameter;

    task_conf.data_mutex = xSemaphoreCreateMutex();
    esp_event_handler_register_with(conf->evt_loop, LULUPET_EVENT_BASE,
                                    ESP_EVENT_ANY_ID, fs_event_handler, NULL);

    task_conf.fs_mount = false;
    fs_mount();
    task_conf.fs_mount = true;

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

bool fs_get_mount(void) { return task_conf.fs_mount; }

void fs_get_fatfs_usage(size_t *out_total_bytes, size_t *out_free_bytes) {
    FATFS *fs;
    size_t free_clusters;
    int res = f_getfree("0:", &free_clusters, &fs);
    assert(res == FR_OK);
    size_t total_sectors = (fs->n_fatent - 2) * fs->csize;
    size_t free_sectors = free_clusters * fs->csize;

    // assuming the total size is < 4GiB, should be true for SPI Flash
    if (out_total_bytes != NULL) {
        *out_total_bytes = total_sectors * fs->ssize;
    }
    if (out_free_bytes != NULL) {
        *out_free_bytes = free_sectors * fs->ssize;
    }
}
