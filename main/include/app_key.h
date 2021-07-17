#ifndef _APP_KEY_H_
#define _APP_KEY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_event.h"

typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_PRESS_OVER_5_SEC,
    KEY_EVENT_PRESS_2_TIMES_WITHIN_3_SEC,
} key_event_type_e;

typedef struct {
    key_event_type_e key_event_type;
} key_loop_event_t;

char *app_key_event_type_str(key_event_type_e type);
void app_key_main(esp_event_loop_handle_t loop);
void key_check_wakeup(void);

#ifdef __cplusplus
}
#endif

#endif // end of _APP_KEY_H_
