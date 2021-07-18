#ifndef _TASK_FS_H_
#define _TASK_FS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_event.h"

void fs_task_start(esp_event_loop_handle_t loop);

#ifdef __cplusplus
}
#endif

#endif // end of _TASK_FS_H_
