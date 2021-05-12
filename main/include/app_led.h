#ifndef _APP_LED_H_
#define _APP_LED_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdbool.h>

extern QueueHandle_t led_cmd_que;

/* LED CMD */
#define LED_ALL_OFF 0
#define LED_WHITE_SOLID 1
#define LED_RED_SOLID 2
#define LED_BLUE_SOLID 3
#define LED_GREEN_SOLID 4
#define LED_BLUE_2HZ 5

void app_led_main(void);

#ifdef __cplusplus
}
#endif

#endif // end of _APP_LED_H_