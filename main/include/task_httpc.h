#ifndef _TASK_HTTPC_H_
#define _TASK_HTTPC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_camera.h"
#include "esp_event.h"

#include "app_weight.h"

typedef enum {
    HTTPC_PHOTO_SRC_RAM = 0,
    HTTPC_PHOTO_SRC_FS,
} httpc_photo_source_e;

typedef struct {
    weight_take_photo_event_t weight_take_photo_evt;
    time_t unix_timestamp;
    camera_fb_t *fb;
} httpc_photo_buf_t;

void start_httpc_task(esp_event_loop_handle_t loop);

esp_err_t httpc_ota_post_event(esp_event_loop_handle_t event_loop);

void http_send_photo_process(httpc_photo_source_e photo_src);
void http_photo_buf_push_ram(weight_take_photo_event_t *take_photo_event);
void http_photo_buf_push_fs(weight_take_photo_event_t *take_photo_event);
esp_err_t http_photo_buf_pop_fs(httpc_photo_buf_t *photo_buf);

#ifdef __cplusplus
}
#endif

#endif // end of _TASK_HTTPC_H_
