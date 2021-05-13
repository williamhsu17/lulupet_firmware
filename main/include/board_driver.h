#ifndef _BOARD_DRIVER_H_
#define _BOARD_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

typedef enum {
    LED_TYPE_W = 0,
    LED_TYPE_R,
    LED_TYPE_G,
    LED_TYPE_B,
    LED_TYPE_IR,   // ioe ir led
    LED_TYPE_BD_W, // ioe white led
    LED_TYPE_MAX,
} led_type_e;

esp_err_t i2c_mcp3221_readADC(i2c_port_t i2c_num, unsigned int *buffer);
esp_err_t i2c_RV3029_readTIME(i2c_port_t i2c_num, unsigned int *buffer);
esp_err_t i2c_RV3029_writeTIME(i2c_port_t i2c_num, int time_hour, int time_min,
                               int time_sec);
esp_err_t i2c_RV3029_readDay(i2c_port_t i2c_num, unsigned int *buffer);
esp_err_t i2c_RV3029_writeDay(i2c_port_t i2c_num, int time_year, int time_mon,
                              int time_day, int time_weekday);
esp_err_t i2c_BCT3253_writeREG(i2c_port_t i2c_num, int offset_address,
                               int value);
esp_err_t i2c_MCP23016_writeREG(i2c_port_t i2c_num, uint8_t offset_address,
                                uint8_t value);
esp_err_t i2c_MCP23016_readREG(i2c_port_t i2c_num, uint8_t offset_address,
                               uint8_t *buffer);

/**
 * @brief Get RESET_KEY (IOE P1.3) status
 *
 * @retval false if release
 * @retval true if press
 */
bool board_get_key_status(void);

esp_err_t board_led_ctrl(led_type_e led, bool enable);
esp_err_t board_init(void);

#ifdef __cplusplus
}
#endif

#endif // end of _BOARD_DRIVER_H_
