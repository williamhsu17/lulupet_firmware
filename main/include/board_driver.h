#ifndef _BOARD_DRIVER_H_
#define _BOARD_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

esp_err_t i2c_mcp3221_readADC(i2c_port_t i2c_num, unsigned int *buffer);
esp_err_t i2c_RV3029_readTIME(i2c_port_t i2c_num, unsigned int *buffer);
esp_err_t i2c_RV3029_writeTIME(i2c_port_t i2c_num, int time_hour, int time_min,
                               int time_sec);
esp_err_t i2c_RV3029_readDay(i2c_port_t i2c_num, unsigned int *buffer);
esp_err_t i2c_RV3029_writeDay(i2c_port_t i2c_num, int time_year, int time_mon,
                              int time_day, int time_weekday);
esp_err_t i2c_BCT3253_writeREG(i2c_port_t i2c_num, int offset_address,
                               int value);
esp_err_t i2c_MCP23016_writeREG(i2c_port_t i2c_num, int offset_address,
                                int value);
esp_err_t i2c_MCP23016_readREG(i2c_port_t i2c_num, int offset_address,
                               unsigned int *buffer);
esp_err_t board_init(void);

#ifdef __cplusplus
}
#endif

#endif // end of _BOARD_DRIVER_H_
