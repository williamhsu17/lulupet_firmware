#ifndef _APP_LED_H_
#define _APP_LED_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern QueueHandle_t led_cmd_que;

/* LED CMD */
#define LED_ALL_OFF 0
#define LED_WHITE_CNT 1
#define LED_RED_CNT 2
#define LED_BLUE_CNT 3
#define LED_GREEN_CNT 4
#define LED_BLUE_2HZ 5

void app_led_main(void);

#ifdef __cplusplus
}
#endif

#endif // end of _APP_LED_H_