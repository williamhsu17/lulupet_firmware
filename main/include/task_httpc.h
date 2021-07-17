#ifndef _TASK_HTTPC_H_
#define _TASK_HTTPC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_event.h"

#include "app_weight.h"

void start_httpc_task(esp_event_loop_handle_t loop);

esp_err_t httpc_ota_post_event(esp_event_loop_handle_t event_loop);

void http_send_photo_process(void);
void http_photo_buf_push(weight_take_photo_event_t *take_photo_event);

#ifdef __cplusplus
}
#endif

#endif // end of _TASK_HTTPC_H_
