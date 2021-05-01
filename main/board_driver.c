#include "app_camera.h"
#include "esp_camera.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"

#include "include/board_driver.h"
#include "include/util.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define TAG "board_drv"

static esp_err_t board_i2c_master_init(void);
static esp_err_t board_driver_init(void);
static esp_err_t board_cam_init_gpio(void);
static esp_err_t board_cam_init(void);

/**
 * @brief i2c master initialization
 */
static esp_err_t board_i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t board_pir_init_gpio(void) {
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIRPWR_PIN_SEL;
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 1;
    // configure GPIO with the given settings
    gpio_config(&io_conf);

    // interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    // bit mask of the pins, use GPIO2 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIR_PIN_SEL;
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_set_level(GPIO_OUTPUT_PIRPWR, 1);

    return ESP_OK;
}

static esp_err_t board_driver_init(void) {
    esp_err_t err;

    // Init the I2C
    err = board_i2c_master_init();

    // Init the GPIO for IR detection
    err |= board_pir_init_gpio();

    // Init RGB LED and Light ON Max white
    err |= i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x00, 0x01); // chip RESET
    err |= i2c_BCT3253_writeREG(
        I2C_MASTER_NUM, 0x02,
        0x40); // set overall brightness Max value 25.50mA, Step value 0.10mA
    err |= i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x03,
                                0x0); // set brightness 20mA (R)
    err |= i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x04,
                                0x0); // set brightness 20mA (B)
    err |= i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x05,
                                0x0); // set brightness 20mA (G)
    err |=
        i2c_BCT3253_writeREG(I2C_MASTER_NUM, 0x01, 0x07); // enable both LEDs ON

    // Init GPIO Extender
    err |= i2c_MCP23016_writeREG(I2C_MASTER_NUM, MCP23016_IODIR0_ADDR, 0xFF);
    err |= i2c_MCP23016_writeREG(I2C_MASTER_NUM, MCP23016_IODIR1_ADDR, 0xF8);
    // LED All ON, IR ON
    err |= i2c_MCP23016_writeREG(I2C_MASTER_NUM, MCP23016_GPIO1_ADDR, 0x06);
    err |= i2c_MCP23016_writeREG(I2C_MASTER_NUM, MCP23016_OLAT1_ADDR, 0x06);

    return ESP_OK;
}

static esp_err_t board_cam_init_gpio(void) {
    esp_err_t err;
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins
    io_conf.pin_bit_mask = GPIO_OUTPUT_CAMPWR_PIN_SEL;
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 1;
    // configure GPIO with the given settings
    err = gpio_config(&io_conf);
    ESP_ERROR_CHECK(err);

    err = gpio_set_level(GPIO_OUTPUT_CAMPWR, 1);
    ESP_ERROR_CHECK(err);

    return err;
}

static esp_err_t board_cam_init(void) {
    esp_err_t err;

    err = board_cam_init_gpio();
    ESP_ERROR_CHECK(err);

#if CONFIG_CAMERA_MODEL_ESP_EYE
    /* IO13, IO14 is designed for JTAG by default,
     * to use it as generalized input,
     * firstly declair it as pullup input */
    gpio_config_t conf;
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = GPIO_PULLUP_ENABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    conf.pin_bit_mask = 1LL << 13;
    gpio_config(&conf);
    conf.pin_bit_mask = 1LL << 14;
    gpio_config(&conf);
#endif

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    // reduce camera interface spped for stability
    // config.xclk_freq_hz = 20000000;
    // config.xclk_freq_hz = 10000000;
    config.xclk_freq_hz = 5000000;
    config.pixel_format = PIXFORMAT_JPEG;
    // init with high specs to pre-allocate larger buffers
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;

    // camera init
    err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);       // flip it back
        s->set_brightness(s, 1);  // up the blightness just a bit
        s->set_saturation(s, -2); // lower the saturation
    }
    // drop down frame size for higher initial frame rate
    s->set_framesize(s, CAMERA_FRAME_SIZE);

    return err;
}

esp_err_t i2c_RV3029_readTIME(i2c_port_t i2c_num, unsigned int *buffer) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd;
    int time_sec, time_min, time_hour;
    uint8_t *reg_sec = (uint8_t *)malloc(sizeof(uint8_t));
    uint8_t *reg_min = (uint8_t *)malloc(sizeof(uint8_t));
    uint8_t *reg_hour = (uint8_t *)malloc(sizeof(uint8_t));
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, RV3029_CHIP_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, RV3029_TIME_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        free(reg_sec);
        free(reg_min);
        free(reg_hour);
        return ret;
    }

    vTaskDelay(30 / portTICK_RATE_MS);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, RV3029_CHIP_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, reg_sec, 1, ACK_VAL);
    i2c_master_read(cmd, reg_min, 1, ACK_VAL);
    i2c_master_read(cmd, reg_hour, 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        free(reg_sec);
        free(reg_min);
        free(reg_hour);
        return ret;
    }
    time_sec = ((*reg_sec) >> 4) * 10 + (0xf & (*reg_sec));
    time_min = ((*reg_min) >> 4) * 10 + (0xf & (*reg_min));
    if (((*reg_hour) & 0x40) == 0) { // 24-hour mode
        time_hour = ((*reg_hour & 0x3F) >> 4) * 10 + (0xf & (*reg_hour & 0x3F));
    } else { // 12-hour AM-PM mode
        time_hour =
            (((*reg_hour & 0x1F) >> 4) * 10 + (0xf & (*reg_hour & 0x1F))) +
            ((*reg_hour) & 0x20) * 12;
    }
    // ESP_LOGE(TAG, "I2C RTC Read:%d %d %d", time_hour, time_min, time_sec);
    *buffer = time_sec + time_min * 100 + time_hour * 10000;
    free(reg_sec);
    free(reg_min);
    free(reg_hour);
    return ret;
}

esp_err_t i2c_RV3029_writeTIME(i2c_port_t i2c_num, int time_hour, int time_min,
                               int time_sec) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, RV3029_CHIP_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, RV3029_TIME_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, time_sec, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, time_min, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, time_hour, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    // ESP_LOGE(TAG, "I2C RTC Write:%x %x %x", time_hour, time_min, time_sec);
    return ret;
}

esp_err_t i2c_RV3029_readDay(i2c_port_t i2c_num, unsigned int *buffer) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd;
    int time_day, time_mon, time_year;
    uint8_t *reg_day = (uint8_t *)malloc(sizeof(uint8_t));
    uint8_t *reg_weekday = (uint8_t *)malloc(sizeof(uint8_t));
    uint8_t *reg_mon = (uint8_t *)malloc(sizeof(uint8_t));
    uint8_t *reg_year = (uint8_t *)malloc(sizeof(uint8_t));
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, RV3029_CHIP_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, RV3029_DATE_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        free(reg_day);
        free(reg_weekday);
        free(reg_mon);
        free(reg_year);
        return ret;
    }

    vTaskDelay(30 / portTICK_RATE_MS);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, RV3029_CHIP_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, reg_day, 1, ACK_VAL);
    i2c_master_read(cmd, reg_weekday, 1, ACK_VAL);
    i2c_master_read(cmd, reg_mon, 1, ACK_VAL);
    i2c_master_read(cmd, reg_year, 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        free(reg_day);
        free(reg_weekday);
        free(reg_mon);
        free(reg_year);
        return ret;
    }
    time_day = ((*reg_day & 0x3F) >> 4) * 10 + (0xf & (*reg_day & 0x3F));
    time_mon = ((*reg_mon & 0x1F) >> 4) * 10 + (0xf & (*reg_mon & 0x1F));
    time_year = ((*reg_year & 0x7F) >> 4) * 10 + (0xf & (*reg_year & 0x7F));
    // ESP_LOGE(TAG, "I2C RTC Read:%d %d %d", time_year, time_mon, time_day);
    *buffer = time_day + time_mon * 100 + (time_year + 2000) * 10000;
    free(reg_day);
    free(reg_weekday);
    free(reg_mon);
    free(reg_year);
    return ret;
}

esp_err_t i2c_RV3029_writeDay(i2c_port_t i2c_num, int time_year, int time_mon,
                              int time_day, int time_weekday) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, RV3029_CHIP_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, RV3029_DATE_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, time_day, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, time_weekday, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, time_mon, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, time_year, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    // ESP_LOGE(TAG, "I2C RTC Write:%x %x %x %x", time_year, time_mon, time_day,
    // time_weekday);
    return ret;
}

esp_err_t i2c_BCT3253_writeREG(i2c_port_t i2c_num, int offset_address,
                               int value) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BCT3253_CHIP_ADDR << 1 | WRITE_BIT,
                          ACK_CHECK_EN);
    i2c_master_write_byte(cmd, offset_address, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    return ret;
}

esp_err_t i2c_MCP23016_writeREG(i2c_port_t i2c_num, int offset_address,
                                int value) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MCP23016_CHIP_ADDR << 1 | WRITE_BIT,
                          ACK_CHECK_EN);
    i2c_master_write_byte(cmd, offset_address, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    return ret;
}

esp_err_t i2c_MCP23016_readREG(i2c_port_t i2c_num, int offset_address,
                               unsigned int *buffer) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd;
    uint8_t *reg_read = (uint8_t *)malloc(sizeof(uint8_t));
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MCP23016_CHIP_ADDR << 1 | WRITE_BIT,
                          ACK_CHECK_EN);
    i2c_master_write_byte(cmd, offset_address, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        free(reg_read);
        return ret;
    }

    vTaskDelay(30 / portTICK_RATE_MS);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MCP23016_CHIP_ADDR << 1 | READ_BIT,
                          ACK_CHECK_EN);
    i2c_master_read(cmd, reg_read, 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        free(reg_read);
        return ret;
    }
    *buffer = *reg_read;
    free(reg_read);
    return ret;
}

esp_err_t i2c_mcp3221_readADC(i2c_port_t i2c_num, unsigned int *buffer) {
    esp_err_t ret;
    uint8_t value_hi;
    uint8_t value_lo;
    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MCP3221_CHIP_ADDR << 1 | WRITE_BIT,
                          ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MCP3221_DATA_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(30 / portTICK_RATE_MS); // TODO: chekc datasheet

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MCP3221_CHIP_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
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

esp_err_t board_init(void) {
    ESP_ERROR_CHECK(board_cam_init());
    ESP_ERROR_CHECK(board_driver_init());

    return ESP_OK;
}
