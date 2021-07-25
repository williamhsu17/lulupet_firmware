#ifndef _TASK_FS_H_
#define _TASK_FS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_camera.h"
#include "esp_event.h"

#include "app_weight.h"

void fs_task_start(esp_event_loop_handle_t loop);
bool fs_get_mount(void);
void fs_get_fatfs_usage(size_t *out_total_bytes, size_t *out_free_bytes);
esp_err_t fs_save_photo(weight_take_photo_event_t *take_photo_evt,
                        time_t timestamp, camera_fb_t *fb);
esp_err_t fs_load_photo(weight_take_photo_event_t *take_photo_evt,
                        time_t *timestamp, camera_fb_t *fb);

#ifdef __cplusplus
}
#endif

#endif // end of _TASK_FS_H_
