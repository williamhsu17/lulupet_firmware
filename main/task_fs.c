#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_vfs_fat.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

#include "include/event.h"
#include "include/util.h"

#include "include/task_fs.h"

#define TAG "fs_task"
#define FS_TASK_PERIOD_MS 100
#define FS_ROOT_NAME "/fs"

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

static void list_data_partitions(void);
static esp_err_t fs_mount(void);
static void fs_task(void *pvParameter);

static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

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

static void list_data_partitions(void) {
    ESP_LOGI(TAG, "Listing data partitions:");
    esp_partition_iterator_t it = esp_partition_find(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, NULL);

    for (; it != NULL; it = esp_partition_next(it)) {
        const esp_partition_t *part = esp_partition_get(it);
        ESP_LOGI(TAG, "- partition '%s', subtype %d, offset 0x%x, size %d kB",
                 part->label, part->subtype, part->address, part->size / 1024);
    }

    esp_partition_iterator_release(it);
}

static esp_err_t fs_mount(void) {

    // List the available partitions
    list_data_partitions();

    const char *partition_label = "storage";
    const esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 4,
        .format_if_mount_failed = true,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE};
    ESP_LOGW(TAG, "start mount fs");
    esp_err_t esp_err = esp_vfs_fat_spiflash_mount(
        FS_ROOT_NAME, partition_label, &mount_config, &s_wl_handle);
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

esp_err_t fs_save_photo(weight_take_photo_event_t *take_photo_evt,
                        time_t timestamp, camera_fb_t *fb) {

    if (take_photo_evt == NULL || fb == NULL) {
        return esp_err_print(ESP_ERR_INVALID_ARG, __func__, __LINE__);
    }

    ESP_LOGI(TAG, "len:%d", fb->len);
    ESP_LOGI(TAG, "width:%d", fb->width);
    ESP_LOGI(TAG, "height:%d", fb->height);
    ESP_LOGI(TAG, "format:%d", fb->format);
    ESP_LOGI(TAG, "tv_sec:%ld", fb->timestamp.tv_sec);
    ESP_LOGI(TAG, "tv_usec:%ld", fb->timestamp.tv_usec);
    ESP_LOGI(TAG, "eventid:%d", take_photo_evt->eventid);
    ESP_LOGI(TAG, "weight_g:%d", take_photo_evt->weight_g);
    ESP_LOGI(TAG, "pir_val:%d", take_photo_evt->pir_val);
    ESP_LOGI(TAG, "timestamp:%ld", timestamp);

    char file_name[128];

    snprintf(file_name, sizeof(file_name), "%s/%ld.cfg", FS_ROOT_NAME,
             timestamp);

    ESP_LOGI(TAG, "opening file: %s", file_name);
    FILE *f = NULL;

    f = fopen(file_name, "wb");
    if (f == NULL) {
        ESP_LOGI(TAG, "errno: %d", errno);
        return esp_err_print(ESP_ERR_INVALID_RESPONSE, __func__, __LINE__);
    }
    fprintf(f, "len:%d\n", fb->len);
    fprintf(f, "width:%d\n", fb->width);
    fprintf(f, "height:%d\n", fb->height);
    fprintf(f, "format:%d\n", fb->format);
    fprintf(f, "tv_sec:%ld\n", fb->timestamp.tv_sec);
    fprintf(f, "tv_usec:%ld\n", fb->timestamp.tv_usec);
    fprintf(f, "eventid:%d\n", take_photo_evt->eventid);
    fprintf(f, "weight_g:%d\n", take_photo_evt->weight_g);
    fprintf(f, "pir_val:%d\n", take_photo_evt->pir_val);
    fprintf(f, "timestamp:%ld\n", timestamp);
    fclose(f);
    ESP_LOGI(TAG, "close file: %s", file_name);

    snprintf(file_name, sizeof(file_name), "%s/%ld.jpg", FS_ROOT_NAME,
             timestamp);
    ESP_LOGI(TAG, "opening file: %s", file_name);
    f = fopen(file_name, "wb");
    if (f == NULL) {
        return esp_err_print(ESP_ERR_INVALID_RESPONSE, __func__, __LINE__);
    }
    int fwrite_size = fwrite(fb->buf, 1, fb->len, f);
    fclose(f);
    ESP_LOGI(TAG, "close file: %s", file_name);
    ESP_LOGI(TAG, "fwrite_size[%d], fb->len[%d]", fwrite_size, fb->len);
    if (fwrite_size != fb->len) {
        return esp_err_print(ESP_ERR_INVALID_RESPONSE, __func__, __LINE__);
    }

    return ESP_OK;
}
