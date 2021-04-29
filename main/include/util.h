#ifndef _UTIL_H_
#define _UTIL_H_

#ifdef __cplusplus
extern "C" {
#endif

// Version
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 0

// I2C configuration
#define I2C_MASTER_SCL_IO 13         /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 4          /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(1) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000    /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0  /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0  /*!< I2C master doesn't need buffer */

// I2C setting
#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

// I2C comunication
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

// MCP3221 ADC setting
#define mcp3221_chip_addr 0x4b // 7-bit I2C address
#define mcp3221_data_addr 0x0
#define mcp3221_len 2

// Function
#define FUNC_CMD_TASK 1 // 1: run console command line interface

#ifdef __cplusplus
}
#endif

#endif // end of _UTIL_H_
