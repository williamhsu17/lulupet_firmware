#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#include "include/board_driver.h"
#include "include/util.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define TAG "board_drv"

esp_err_t i2c_mcp3221_readADC(i2c_port_t i2c_num, unsigned int *buffer) {
    esp_err_t ret;
    uint8_t value_hi;
    uint8_t value_lo;
    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, mcp3221_chip_addr << 1 | WRITE_BIT,
                          ACK_CHECK_EN);
    i2c_master_write_byte(cmd, mcp3221_data_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(30 / portTICK_RATE_MS); // TODO: chekc datasheet

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, mcp3221_chip_addr << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, &value_hi, 1, ACK_VAL);
    i2c_master_read(cmd, &value_lo, 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    (*buffer) = (unsigned int)((((unsigned int)value_hi) << 8) | (value_lo));

    ESP_LOGD(TAG, "I2C ADC Read:%x %x %d", value_hi, value_lo, (*buffer));

    return ret;
}