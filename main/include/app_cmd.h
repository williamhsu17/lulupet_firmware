#ifndef _APP_CMD_H_
#define _APP_CMD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

void app_cmd_main(esp_event_loop_handle_t event_loop);
esp_err_t register_system(void);

#ifdef __cplusplus
}
#endif

#endif // end of _APP_CMD_H_
