#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TICK_PERIOD_MS 1
static uint32_t systick = 0;

static void tick_inc(void *arg) {
    (void)arg;
    systick++;
}

uint32_t timer_tick_diff(uint32_t start_tick, uint32_t end_tick) {
    return (end_tick > start_tick) ? (end_tick - start_tick)
                                   : (UINT32_MAX - start_tick) + end_tick;
}

uint32_t timer_tick_ms(void) { return systick; }

esp_err_t timer_tick_init(void) {
    /* Create and start a periodic timer interrupt to call tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &tick_inc, .name = "periodic_tick"};
    esp_timer_handle_t periodic_timer;
    esp_err_t esp_err;

    esp_err = esp_timer_create(&periodic_timer_args, &periodic_timer);
    esp_err |= esp_timer_start_periodic(periodic_timer, TICK_PERIOD_MS * 1000);

    return esp_err;
}
