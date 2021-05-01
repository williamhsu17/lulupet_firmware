#ifndef _BOARD_DRIVER_H_
#define _BOARD_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

esp_err_t i2c_mcp3221_readADC(i2c_port_t i2c_num, unsigned int *buffer);

#ifdef __cplusplus
}
#endif

#endif // end of _BOARD_DRIVER_H_