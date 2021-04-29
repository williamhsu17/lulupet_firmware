#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "include/util.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define TAG "weight_task"

#define WEIGHT_TASK_BUFFER_SIZE 100

enum weight_task_stat {
    WEIGHT_TASK_STAT_STANDBY = 0,
    WEIGHT_TASK_STAT_JUMP,
    WEIGHT_TASK_STAT_BIG_JUMP,
};

typedef struct {
    // weight state machine
    enum weight_task_stat now_stat;
    enum weight_task_stat pre_stat;

    // weight data
    unsigned int ring_buffer[WEIGHT_TASK_BUFFER_SIZE]; // TODO: the buffer just
                                                       // for verifying
    unsigned int ref_weight_sum;
    bool ring_buffer_loop;
    uint32_t ring_buffer_idx;
    float ref_weight;
    float adc_weight;

} weight_task_cb;

static weight_task_cb w_task_cb;

// TODO: move to bsp
static esp_err_t i2c_mcp3221_readADC(i2c_port_t i2c_num, unsigned int *buffer) {
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

static void weight_task(void *pvParameter) {
    // inital cb
    w_task_cb.now_stat = w_task_cb.pre_stat = WEIGHT_TASK_STAT_STANDBY;
    w_task_cb.ring_buffer_idx = 0;
    w_task_cb.ring_buffer_loop = false;
    w_task_cb.ref_weight_sum = 0;
    unsigned int tmp_adc;

    for (;;) {

        // measure process
        if (i2c_mcp3221_readADC(I2C_MASTER_NUM, &tmp_adc) == ESP_OK) {
            w_task_cb.adc_weight = 1.0 * tmp_adc; // calculate latest weight

            w_task_cb.ref_weight_sum += tmp_adc;
            if (w_task_cb.ring_buffer_loop) {
                w_task_cb.ref_weight_sum -=
                    w_task_cb.ring_buffer
                        [w_task_cb.ring_buffer_idx]; // minus orignal value
                w_task_cb.ring_buffer[w_task_cb.ring_buffer_idx] =
                    tmp_adc; // save adc data into buffer
                w_task_cb.ref_weight =
                    1.0 * w_task_cb.ref_weight_sum /
                    (1.0 *
                     WEIGHT_TASK_BUFFER_SIZE); // calculate reference weight
            } else {
                w_task_cb.ring_buffer[w_task_cb.ring_buffer_idx] =
                    tmp_adc; // save adc data into buffer
                w_task_cb.ref_weight =
                    1.0 * w_task_cb.ref_weight_sum /
                    (1.0 * (w_task_cb.ring_buffer_idx +
                            1)); // calculate reference weight
            }

            ++w_task_cb.ring_buffer_idx;
            if (w_task_cb.ring_buffer_idx == WEIGHT_TASK_BUFFER_SIZE) {
                w_task_cb.ring_buffer_idx = 0;
                w_task_cb.ring_buffer_loop = true;
            }

            ESP_LOGD(TAG, "now [%f] ref [%f]", w_task_cb.adc_weight,
                     w_task_cb.ref_weight);
        }

        vTaskDelay(50 / portTICK_PERIOD_MS); // run every 5 msec
    }
}

void app_weight_main(void) {
    ESP_LOGD(TAG, "app_weight_main start");

    xTaskCreate(&weight_task, "weight_task", 2048, NULL, 5, NULL);
}
