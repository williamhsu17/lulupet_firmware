#ifndef _UTIL_H_
#define _UTIL_H_

#ifdef __cplusplus
extern "C" {
#endif

// Version
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 0

// I2C
#define I2C_MASTER_SCL_IO 13         /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 4          /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(1) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000    /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0  /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0  /*!< I2C master doesn't need buffer */

// Function
#define FUNC_CMD_TASK 1 // 1: run console command line interface

#ifdef __cplusplus
}
#endif

#endif // end of _UTIL_H_
