#ifndef __TIMER_TICK_H__
#define __TIMER_TICK_H__

#include <stdint.h>

/**
 * @brief Get durint time from start_tick to end_tick
 *
 * @param start_tick[in] start tick
 * @param end_tick[in] end_tick
 *
 * @retval during time: unit: ms
 */
uint32_t timer_tick_diff(uint32_t start_tick, uint32_t end_tick);

uint32_t timer_tick_ms(void);
esp_err_t timer_tick_init(void);

#endif // __TIMER_TICK_H__
