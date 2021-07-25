#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_vfs_dev.h"
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
static bool fs_cfg_file_check_param(bool *exist, char *line, char *param,
                                    char *result);
static esp_err_t fs_parser_photo_cfg(char *content,
                                     weight_take_photo_event_t *take_photo_evt,
                                     time_t *timestamp, camera_fb_t *fb);
static int fs_get_file_size(char *fime_name);

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

static bool fs_cfg_file_check_param(bool *exist, char *line, char *param,
                                    char *result) {
    int prefix_len;
    if (!(*exist) && strstr(line, param) != NULL) {
        prefix_len = strlen(param);
        memcpy(result, line + prefix_len, strlen(line) - prefix_len);
        ESP_LOGI(TAG, "param[%s] result[%s]", param, result);
        (*exist) = true;
        return true;
    } else {
        return false;
    }
}

static esp_err_t fs_parser_photo_cfg(char *content,
                                     weight_take_photo_event_t *take_photo_evt,
                                     time_t *timestamp, camera_fb_t *fb) {
    char *start_p;
    char *end_p;
    char tmp_str[64];
    char line_str[256];

    bool len_exist = false;
    bool width_exist = false;
    bool height_exist = false;
    bool format_exist = false;
    bool tv_sec_exist = false;
    bool tv_usec_exist = false;
    bool eventid_exist = false;
    bool weight_g_exist = false;
    bool pir_val_exist = false;
    bool timestamp_exist = false;

    ESP_LOGD(TAG, "strlen(content): %d", strlen(content));

    start_p = content;
    for (int i = 0; i < strlen(content); ++i) {
        if (content[i] == '\n') {
            end_p = content + i;
            memset(line_str, 0x00, sizeof(line_str));
            memcpy(line_str, start_p, end_p - start_p);
            ESP_LOGD(TAG, "line_str: %s", line_str);

            memset(tmp_str, 0x00, sizeof(tmp_str));
            if (fs_cfg_file_check_param(&len_exist, line_str,
                                        "len:", tmp_str)) {
                fb->len = atoi(tmp_str);
            }
            if (fs_cfg_file_check_param(&width_exist, line_str,
                                        "width:", tmp_str)) {
                fb->width = atoi(tmp_str);
            }
            if (fs_cfg_file_check_param(&height_exist, line_str,
                                        "height:", tmp_str)) {
                fb->height = atoi(tmp_str);
            }
            if (fs_cfg_file_check_param(&format_exist, line_str,
                                        "format:", tmp_str)) {
                fb->format = atoi(tmp_str);
            }
            if (fs_cfg_file_check_param(&tv_sec_exist, line_str,
                                        "tv_sec:", tmp_str)) {
                fb->timestamp.tv_sec = atoi(tmp_str);
            }
            if (fs_cfg_file_check_param(&tv_usec_exist, line_str,
                                        "tv_usec:", tmp_str)) {
                fb->timestamp.tv_usec = atoi(tmp_str);
            }
            if (fs_cfg_file_check_param(&eventid_exist, line_str,
                                        "eventid:", tmp_str)) {
                take_photo_evt->eventid = atoi(tmp_str);
            }
            if (fs_cfg_file_check_param(&weight_g_exist, line_str,
                                        "weight_g:", tmp_str)) {
                take_photo_evt->weight_g = atoi(tmp_str);
            }
            if (fs_cfg_file_check_param(&pir_val_exist, line_str,
                                        "pir_val:", tmp_str)) {
                take_photo_evt->pir_val = atoi(tmp_str);
            }
            if (fs_cfg_file_check_param(&timestamp_exist, line_str,
                                        "timestamp:", tmp_str)) {
                (*timestamp) = atoi(tmp_str);
            }

            if (len_exist & width_exist & height_exist & format_exist &
                tv_sec_exist & tv_usec_exist & eventid_exist & weight_g_exist &
                pir_val_exist & timestamp_exist)
                return ESP_OK;

            start_p = content + i + 1;
        }
    }

    return ESP_FAIL;
}

static int fs_get_file_size(char *fime_name) {
    struct stat st;

    if (stat(fime_name, &st) != 0) {
        ESP_LOGE(TAG, "file: %s can not get size", fime_name);
        esp_err_print(ESP_ERR_NOT_FOUND, __func__, __LINE__);
        return 0;
    }

    ESP_LOGI(TAG, "file[%s] size: %li", fime_name, st.st_size);

    return st.st_size;
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

esp_err_t fs_load_photo(weight_take_photo_event_t *take_photo_evt,
                        time_t *timestamp, camera_fb_t *fb) {
    if (take_photo_evt == NULL || fb == NULL) {
        return esp_err_print(ESP_ERR_INVALID_ARG, __func__, __LINE__);
    }

    esp_err_t esp_err = ESP_OK;
    DIR *dir;
    struct dirent *entry;
    char cfg_file_name[384];
    char jpg_file_name[256];
    bool find_cfg = false;

    if (!(dir = opendir(FS_ROOT_NAME))) {
        return esp_err_print(ESP_ERR_NOT_FOUND, __func__, __LINE__);
    }

    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_DIR) {
            if (strcmp(entry->d_name, ".") == 0 ||
                strcmp(entry->d_name, "..") == 0)
                continue;
        } else {
            ESP_LOGI(TAG, "file: %s", entry->d_name);
            if (strstr(entry->d_name, ".cfg") != NULL) {
                find_cfg = true;
                snprintf(cfg_file_name, sizeof(cfg_file_name), "%s/%s",
                         FS_ROOT_NAME, entry->d_name);
                break;
            }
        }
    }

    closedir(dir);

    if (!find_cfg) {
        return esp_err_print(ESP_ERR_NOT_FOUND, __func__, __LINE__);
    }

    bool cfg_rm = false;
    bool jpg_rm = false;
    char *cfg_content = NULL;
    char prefix_name[64];
    int cfg_file_size;
    int jpg_file_size;
    size_t read_size;
    FILE *cfg_f = NULL;
    FILE *jpg_f = NULL;

    // check corresponding .jpg file exist
    char *tmp_p = strstr(cfg_file_name, ".cfg");
    if (tmp_p == NULL) {
        esp_err = ESP_ERR_NOT_FOUND;
        esp_err_print(esp_err, __func__, __LINE__);
        cfg_rm = true;
        goto _end;
    }
    memcpy(prefix_name, cfg_file_name, tmp_p - cfg_file_name);
    snprintf(jpg_file_name, sizeof(jpg_file_name), "%s.jpg", prefix_name);
    if ((jpg_file_size = fs_get_file_size(jpg_file_name)) == 0) {
        esp_err = ESP_ERR_NOT_FOUND;
        esp_err_print(esp_err, __func__, __LINE__);
        cfg_rm = true;
        goto _end;
    }

    // read & parser .cfg file
    if ((cfg_file_size = fs_get_file_size(cfg_file_name)) == 0) {
        esp_err = ESP_ERR_NOT_FOUND;
        esp_err_print(esp_err, __func__, __LINE__);
        cfg_rm = jpg_rm = true;
        goto _end;
    }

    cfg_content = calloc(1, cfg_file_size + 8);
    if (cfg_content == NULL) {
        esp_err_print(ESP_ERR_NO_MEM, __func__, __LINE__);
        goto _end;
    }
    if ((cfg_f = fopen(cfg_file_name, "rb")) == NULL) {
        esp_err = ESP_ERR_NOT_FOUND;
        esp_err_print(esp_err, __func__, __LINE__);
        cfg_rm = jpg_rm = true;
        goto _end;
    }
    read_size = fread(cfg_content, 1, cfg_file_size, cfg_f);
    ESP_LOGI(TAG, "read_size: %d", read_size);
    ESP_LOGD(TAG, "%s", cfg_content);

    if (read_size != cfg_file_size) {
        esp_err = ESP_ERR_INVALID_SIZE;
        esp_err_print(esp_err, __func__, __LINE__);
        cfg_rm = jpg_rm = true;
        goto _end;
    }

    if (fs_parser_photo_cfg(cfg_content, take_photo_evt, timestamp, fb) !=
        ESP_OK) {
        esp_err = ESP_ERR_NOT_SUPPORTED;
        esp_err_print(esp_err, __func__, __LINE__);
        cfg_rm = jpg_rm = true;
        goto _end;
    }

    // read .jpg into fb->buf
    fb->buf = calloc(1, jpg_file_size);
    if (fb->buf == NULL) {
        esp_err_print(ESP_ERR_NO_MEM, __func__, __LINE__);
        goto _end;
    }

    if ((jpg_f = fopen(jpg_file_name, "rb")) == NULL) {
        esp_err = ESP_ERR_NOT_FOUND;
        esp_err_print(esp_err, __func__, __LINE__);
        cfg_rm = jpg_rm = true;
        goto _end;
    }
    read_size = fread(fb->buf, 1, jpg_file_size, jpg_f);
    ESP_LOGI(TAG, "read_size: %d", read_size);
    if (read_size != jpg_file_size) {
        esp_err = ESP_ERR_INVALID_SIZE;
        esp_err_print(esp_err, __func__, __LINE__);
        cfg_rm = jpg_rm = true;
        goto _end;
    }

    cfg_rm = jpg_rm = true;

_end:
    if (cfg_content)
        free(cfg_content);
    if (cfg_f)
        fclose(cfg_f);
    if (jpg_f)
        fclose(jpg_f);
    if (cfg_rm)
        remove(cfg_file_name);
    if (jpg_rm)
        remove(jpg_file_name);
    return esp_err;
}
