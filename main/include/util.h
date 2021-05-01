#ifndef _UTIL_H_
#define _UTIL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"

// Version
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 0

#define CAMERA_FRAME_SIZE FRAMESIZE_XGA

// CAM GPIO setting
#define GPIO_OUTPUT_CAMPWR 12
#define GPIO_OUTPUT_CAMPWR_PIN_SEL (1ULL << GPIO_OUTPUT_CAMPWR)

// CAM GPIO setting
#define GPIO_INPUT_PIR 2
#define GPIO_INPUT_PIR_PIN_SEL (1ULL << GPIO_INPUT_PIR)
#define GPIO_OUTPUT_PIRPWR 33
#define GPIO_OUTPUT_PIRPWR_PIN_SEL (1ULL << GPIO_OUTPUT_PIRPWR)
#define ESP_INTR_FLAG_DEFAULT 0

#if CONFIG_CAMERA_MODEL_WROVER_KIT
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 21
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 19
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 5
#define Y2_GPIO_NUM 4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#elif CONFIG_CAMERA_MODEL_ESP_EYE
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 4
#define SIOD_GPIO_NUM 18
#define SIOC_GPIO_NUM 23

#define Y9_GPIO_NUM 36
#define Y8_GPIO_NUM 37
#define Y7_GPIO_NUM 38
#define Y6_GPIO_NUM 39
#define Y5_GPIO_NUM 35
#define Y4_GPIO_NUM 14
#define Y3_GPIO_NUM 13
#define Y2_GPIO_NUM 34
#define VSYNC_GPIO_NUM 5
#define HREF_GPIO_NUM 27
#define PCLK_GPIO_NUM 25

#elif CONFIG_CAMERA_MODEL_M5STACK_PSRAM
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM 27
#define SIOD_GPIO_NUM 25
#define SIOC_GPIO_NUM 23

#define Y9_GPIO_NUM 19
#define Y8_GPIO_NUM 36
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 39
#define Y5_GPIO_NUM 5
#define Y4_GPIO_NUM 34
#define Y3_GPIO_NUM 35
#define Y2_GPIO_NUM 32
#define VSYNC_GPIO_NUM 22
#define HREF_GPIO_NUM 26
#define PCLK_GPIO_NUM 21

#elif CONFIG_CAMERA_MODEL_M5STACK_WIDE
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM 27
#define SIOD_GPIO_NUM 22
#define SIOC_GPIO_NUM 23

#define Y9_GPIO_NUM 19
#define Y8_GPIO_NUM 36
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 39
#define Y5_GPIO_NUM 5
#define Y4_GPIO_NUM 34
#define Y3_GPIO_NUM 35
#define Y2_GPIO_NUM 32
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 26
#define PCLK_GPIO_NUM 21

#elif CONFIG_CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#elif CONFIG_CAMERA_MODEL_CUSTOM
#define PWDN_GPIO_NUM CONFIG_CAMERA_PIN_PWDN
#define RESET_GPIO_NUM CONFIG_CAMERA_PIN_RESET
#define XCLK_GPIO_NUM CONFIG_CAMERA_PIN_XCLK
#define SIOD_GPIO_NUM CONFIG_CAMERA_PIN_SIOD
#define SIOC_GPIO_NUM CONFIG_CAMERA_PIN_SIOC

#define Y9_GPIO_NUM CONFIG_CAMERA_PIN_Y9
#define Y8_GPIO_NUM CONFIG_CAMERA_PIN_Y8
#define Y7_GPIO_NUM CONFIG_CAMERA_PIN_Y7
#define Y6_GPIO_NUM CONFIG_CAMERA_PIN_Y6
#define Y5_GPIO_NUM CONFIG_CAMERA_PIN_Y5
#define Y4_GPIO_NUM CONFIG_CAMERA_PIN_Y4
#define Y3_GPIO_NUM CONFIG_CAMERA_PIN_Y3
#define Y2_GPIO_NUM CONFIG_CAMERA_PIN_Y2
#define VSYNC_GPIO_NUM CONFIG_CAMERA_PIN_VSYNC
#define HREF_GPIO_NUM CONFIG_CAMERA_PIN_HREF
#define PCLK_GPIO_NUM CONFIG_CAMERA_PIN_PCLK
#endif

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

// RV3029 RTC setting
#define rv3029_chip_addr 0x56 // 7-bit I2C address
#define rv3029_time_addr 0x08 // offset to start of time registers
#define rv3029_date_addr 0x0B // offset to start of Date registers

// BCT3253 RGB LED Driver
#define bct3253_chip_addr 0x30

// MCP23016 IO Extender
#define mcp23016_chip_addr 0x24
#define mcp23016_GPIO0_addr 0x00
#define mcp23016_GPIO1_addr 0x01
#define mcp23016_OLAT0_addr 0x02
#define mcp23016_OLAT1_addr 0x03
#define MCP23016_IODIR0_ADDR 0x06
#define mcp23016_IODIR1_addr 0x07
#define mcp23016_INTCAP0_addr 0x08
#define mcp23016_INTCAP1_addr 0x09

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
