#ifndef _TASK_HTTPC_H_
#define _TASK_HTTPC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_event.h"

void start_httpc_task(esp_event_loop_handle_t loop);

esp_err_t httpc_ota_post_event(esp_event_loop_handle_t event_loop);

#ifdef __cplusplus
}
#endif

#endif // end of _TASK_HTTPC_H_
