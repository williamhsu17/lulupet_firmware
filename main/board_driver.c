#include "app_camera.h"
#include "esp_camera.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "include/app_weight.h"
#include "include/board_driver.h"
#include "include/util.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define TAG "board_drv"

#define BOARD_DRIVER_I2C0_TIMEOUT_MS 3000

typedef struct {
    uint8_t address;
    uint8_t curr_val;
} reg_led_cb_t;

typedef struct {
    reg_led_cb_t *rgb_led;
    bool ioe_ir_led;
    bool ioe_w_led;
} board_cb_t;

typedef enum {
    GPIO_OUTPUT_LOW = 0,
    GPIO_OUTPUT_HIGH = 1,
} gpio_output_level_e;

SemaphoreHandle_t i2c0_mutex;

static reg_led_cb_t reg_led_cb[] = {
    {
        .address = RGB_LED_R_ADDR,
    },
    {
        .address = RGB_LED_G_ADDR,
    },
    {
        .address = RGB_LED_B_ADDR,
    },
};

static board_cb_t board_cb = {
    .rgb_led = reg_led_cb,
};

static esp_err_t board_ioe_output_en(uint8_t port, uint8_t pin,
                                     gpio_output_level_e level);
static esp_err_t board_i2c_master_init(void);
static esp_err_t board_driver_init(void);
static esp_err_t board_cam_init_gpio(void);
static esp_err_t board_cam_init(void);

static esp_err_t board_ioe_output_en(uint8_t port, uint8_t pin,
                                     gpio_output_level_e level) {
    uint8_t port_addr;
    // uint8_t latch_addr;
    uint8_t port_val;
    // uint8_t latch_val;
    esp_err_t esp_err;

    if (port == 0) {
        port_addr = MCP23016_GPIO0_ADDR;
        // latch_addr = MCP23016_OLAT0_ADDR;
    } else if (port == 1) {
        port_addr = MCP23016_GPIO1_ADDR;
        // latch_addr = MCP23016_OLAT1_ADDR;
    } else {
        return ESP_FAIL;
    }

    for (int retry = 0; retry < 3; ++retry) {
        esp_err = i2c_MCP23016_readREG(I2C_MASTER_NUM, port_addr, &port_val);
        // esp_err |= i2c_MCP23016_readREG(I2C_MASTER_NUM, latch_addr,
        // &latch_val);
        ESP_LOGI(TAG, "rd ioe_port%d: 0x%02X", port, port_val);
        // ESP_LOGD(TAG, "rd ioe_latch%d: 0x%02X", port, latch_val);

        if (esp_err == ESP_OK) {
            break;
        } else {
            ESP_LOGE(TAG, "err: retry[%d] get port[%d] pin[%d] level[%d] L%d",
                     retry, port, pin, level, __LINE__);
        }
    }

    if (esp_err != ESP_OK) {
        return esp_err;
    }

    if (level) {
        BIT_SET(port_val, pin);
        // BIT_SET(latch_val, pin);
    } else {
        BIT_CLEAR(port_val, pin);
        // BIT_CLEAR(latch_val, pin);
    }

    for (int retry = 0; retry < 3; ++retry) {
        ESP_LOGI(TAG, "wr ioe_port%d: 0x%02X", port, port_val);
        // ESP_LOGD(TAG, "wr ioe_latch%d: 0x%02X", port, latch_val);
        esp_err = i2c_MCP23016_writeREG(I2C_MASTER_NUM, port_addr, port_val);
        // esp_err |= i2c_MCP23016_writeREG(I2C_MASTER_NUM, latch_addr,
        // latch_val);

        if (esp_err == ESP_OK) {
            break;
        } else {
            ESP_LOGE(TAG, "err: retry[%d] set port[%d] pin[%d] level[%d] L%d",
                     retry, port, pin, level, __LINE__);
        }
    }

    return ESP_OK;
}

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

static esp_err_t i2c_write_sacn(i2c_port_t i2c_num, uint16_t addr,
                                uint32_t timeout_ms) {
    i2c_cmd_handle_t *cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t esp_err =
        i2c_master_cmd_begin(i2c_num, cmd, timeout_ms / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return esp_err;
}

static void i2c_scan(i2c_port_t i2c_num) {
    printf("i2c_%d\n", i2c_num);
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
    // Scan the existing i2c slave devices except the preserved address. Please
    // refer to https://www.nxp.com/docs/en/user-guide/UM10204.pdf 3.2.9.

    for (int i = 0; i < 8; i++) {
        if (i % 16 == 0)
            printf("\n%.2x:", i);
        printf(" --");
    }

    for (int i = 8; i < 0x78; i++) {
        if (i % 16 == 0)
            printf("\n%.2x:", i);
        if (i2c_write_sacn(i2c_num, i, 10) == ESP_OK) {
            printf(" %.2x", i);
        } else {
            printf(" --");
        }
    }
    printf("\n");
}

static esp_err_t board_driver_init(void) {
    esp_err_t err;

    // Init the I2C
    err = board_i2c_master_init();

    // Init the GPIO for IR detection
    err |= board_pir_init_gpio();

    // I2C scan
    i2c_scan(I2C_MASTER_NUM);

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
    // set P1.0, P1.1 as output pin.
    uint8_t port1_ctrl;
    i2c_MCP23016_readREG(I2C_MASTER_NUM, MCP23016_IODIR1_ADDR, &port1_ctrl);
    BIT_CLEAR(port1_ctrl, MCP23016_IR_LED_BIT);
    BIT_CLEAR(port1_ctrl, MCP23016_W_LED_BIT);
    i2c_MCP23016_writeREG(I2C_MASTER_NUM, MCP23016_IODIR1_ADDR, port1_ctrl);
    i2c_MCP23016_readREG(I2C_MASTER_NUM, MCP23016_IODIR1_ADDR, &port1_ctrl);

    // test weight sensor
    float adc;
    float g;
    int retry = 3;
    esp_err_t esp_err = ESP_OK;
    while (retry) {
        esp_err = board_get_weight(1, &adc, &g);
        ESP_LOGI(TAG, "read weight sensor[%d]: %.3f, %.3f", retry, adc, g);
        retry--;
    }

    if (esp_err != ESP_OK) {
        ESP_LOGE(TAG, "read weight sensor failed L%d", __LINE__);
        board_set_rgb_led(true, false, false);
        while (1) {
            vTaskDelay(100);
        }
    }

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

esp_err_t board_deinit_gpio(void) {
    esp_err_t err;
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_DISABLE;
    io_conf.pin_bit_mask = GPIO_OUTPUT_CAMPWR_PIN_SEL |
                           GPIO_OUTPUT_PIRPWR_PIN_SEL | GPIO_INPUT_PIR_PIN_SEL |
                           GPIO_OUTPUT_VSYNC_GPIO_NUM;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    err = gpio_config(&io_conf);

    return err;
}

esp_err_t board_deinit_gpio2(void) {
    esp_err_t err;
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_DISABLE;
    io_conf.pin_bit_mask = (1 << 4) | (1 << 13) | (1 << 14);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    err = gpio_config(&io_conf);

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
    config.frame_size = FRAMESIZE_XGA;
    config.jpeg_quality = 12;
    config.fb_count = CAM_RING_BUF_SIZE;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    // CAMERA_GRAB_LATEST; // Note: that will cause ota failed: Guru Meditation
    // Error: Core  0 panic'ed (Cache disabled but
    // cached memory region accessed)

    // camera init
    err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        ESP_LOGW(TAG, "Restarting");
        esp_restart();
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);       // flip it back
        s->set_brightness(s, 1);  // up the blightness just a bit
        s->set_saturation(s, -2); // lower the saturation
    }

    return err;
}

esp_err_t board_cam_deinit(void) { return esp_camera_deinit(); }

esp_err_t i2c_RV3029_readTIME(i2c_port_t i2c_num, unsigned int *buffer) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd;
    int time_sec, time_min, time_hour;
    uint8_t *reg_sec = (uint8_t *)malloc(sizeof(uint8_t));
    uint8_t *reg_min = (uint8_t *)malloc(sizeof(uint8_t));
    uint8_t *reg_hour = (uint8_t *)malloc(sizeof(uint8_t));

    xSemaphoreTake(i2c0_mutex, portMAX_DELAY);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, RV3029_CHIP_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, RV3029_TIME_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd,
                               BOARD_DRIVER_I2C0_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "err: %s L%d", esp_err_to_name(ret), __LINE__);
        goto _end;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, RV3029_CHIP_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, reg_sec, 1, ACK_VAL);
    i2c_master_read(cmd, reg_min, 1, ACK_VAL);
    i2c_master_read(cmd, reg_hour, 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd,
                               BOARD_DRIVER_I2C0_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "err: %s L%d", esp_err_to_name(ret), __LINE__);
        goto _end;
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

_end:
    free(reg_sec);
    free(reg_min);
    free(reg_hour);
    xSemaphoreGive(i2c0_mutex);
    return ret;
}

esp_err_t i2c_RV3029_writeTIME(i2c_port_t i2c_num, int time_hour, int time_min,
                               int time_sec) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd;

    xSemaphoreTake(i2c0_mutex, portMAX_DELAY);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, RV3029_CHIP_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, RV3029_TIME_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, time_sec, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, time_min, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, time_hour, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd,
                               BOARD_DRIVER_I2C0_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "err: %s L%d", esp_err_to_name(ret), __LINE__);
    }

    xSemaphoreGive(i2c0_mutex);

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

    xSemaphoreTake(i2c0_mutex, portMAX_DELAY);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, RV3029_CHIP_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, RV3029_DATE_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd,
                               BOARD_DRIVER_I2C0_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "err: %s L%d", esp_err_to_name(ret), __LINE__);
        goto _end;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, RV3029_CHIP_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, reg_day, 1, ACK_VAL);
    i2c_master_read(cmd, reg_weekday, 1, ACK_VAL);
    i2c_master_read(cmd, reg_mon, 1, ACK_VAL);
    i2c_master_read(cmd, reg_year, 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd,
                               BOARD_DRIVER_I2C0_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "err: %s L%d", esp_err_to_name(ret), __LINE__);
        goto _end;
    }
    time_day = ((*reg_day & 0x3F) >> 4) * 10 + (0xf & (*reg_day & 0x3F));
    time_mon = ((*reg_mon & 0x1F) >> 4) * 10 + (0xf & (*reg_mon & 0x1F));
    time_year = ((*reg_year & 0x7F) >> 4) * 10 + (0xf & (*reg_year & 0x7F));
    // ESP_LOGE(TAG, "I2C RTC Read:%d %d %d", time_year, time_mon, time_day);
    *buffer = time_day + time_mon * 100 + (time_year + 2000) * 10000;

_end:
    free(reg_day);
    free(reg_weekday);
    free(reg_mon);
    free(reg_year);
    xSemaphoreGive(i2c0_mutex);
    return ret;
}

esp_err_t i2c_RV3029_writeDay(i2c_port_t i2c_num, int time_year, int time_mon,
                              int time_day, int time_weekday) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd;

    xSemaphoreTake(i2c0_mutex, portMAX_DELAY);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, RV3029_CHIP_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, RV3029_DATE_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, time_day, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, time_weekday, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, time_mon, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, time_year, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd,
                               BOARD_DRIVER_I2C0_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "err: %s L%d", esp_err_to_name(ret), __LINE__);
    }

    xSemaphoreGive(i2c0_mutex);

    return ret;
}

esp_err_t i2c_BCT3253_writeREG(i2c_port_t i2c_num, int offset_address,
                               int value) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd;

    xSemaphoreTake(i2c0_mutex, portMAX_DELAY);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BCT3253_CHIP_ADDR << 1 | WRITE_BIT,
                          ACK_CHECK_EN);
    i2c_master_write_byte(cmd, offset_address, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd,
                               BOARD_DRIVER_I2C0_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "err: %s L%d", esp_err_to_name(ret), __LINE__);
    }

    xSemaphoreGive(i2c0_mutex);

    return ret;
}

esp_err_t i2c_MCP23016_writeREG(i2c_port_t i2c_num, uint8_t offset_address,
                                uint8_t value) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd;

    xSemaphoreTake(i2c0_mutex, portMAX_DELAY);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MCP23016_CHIP_ADDR << 1 | WRITE_BIT,
                          ACK_CHECK_EN);
    i2c_master_write_byte(cmd, offset_address, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd,
                               BOARD_DRIVER_I2C0_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "err: %s L%d", esp_err_to_name(ret), __LINE__);
    }

    xSemaphoreGive(i2c0_mutex);

    return ret;
}

esp_err_t i2c_MCP23016_readREG(i2c_port_t i2c_num, uint8_t offset_address,
                               uint8_t *buffer) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd;

    xSemaphoreTake(i2c0_mutex, portMAX_DELAY);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MCP23016_CHIP_ADDR << 1 | WRITE_BIT,
                          ACK_CHECK_EN);
    i2c_master_write_byte(cmd, offset_address, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd,
                               BOARD_DRIVER_I2C0_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "error: %s[%d] L%d", esp_err_to_name(ret), ret, __LINE__);
        goto _end;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MCP23016_CHIP_ADDR << 1 | READ_BIT,
                          ACK_CHECK_EN);
    i2c_master_read(cmd, buffer, 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd,
                               BOARD_DRIVER_I2C0_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "error: %s[%d] L%d", esp_err_to_name(ret), ret, __LINE__);
        goto _end;
    }

_end:
    xSemaphoreGive(i2c0_mutex);
    return ret;
}

esp_err_t i2c_mcp3221_readADC(i2c_port_t i2c_num, unsigned int *buffer) {
    esp_err_t ret;
    uint8_t value_hi;
    uint8_t value_lo;
    i2c_cmd_handle_t cmd;

    xSemaphoreTake(i2c0_mutex, portMAX_DELAY);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MCP3221_CHIP_ADDR << 1 | WRITE_BIT,
                          ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MCP3221_DATA_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd,
                               BOARD_DRIVER_I2C0_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "err: %s L%d", esp_err_to_name(ret), __LINE__);
        goto _end;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MCP3221_CHIP_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, &value_hi, 1, ACK_VAL);
    i2c_master_read(cmd, &value_lo, 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd,
                               BOARD_DRIVER_I2C0_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "err: %s L%d", esp_err_to_name(ret), __LINE__);
        goto _end;
    }
    (*buffer) = (unsigned int)((((unsigned int)value_hi) << 8) | (value_lo));

    ESP_LOGD(TAG, "I2C ADC Read:%x %x %d", value_hi, value_lo, (*buffer));

_end:
    xSemaphoreGive(i2c0_mutex);
    return ret;

    return ret;
}

esp_err_t board_set_rgb_led(bool r, bool g, bool b) {
    esp_err_t err = ESP_OK;

    board_cb.rgb_led[LED_TYPE_R].curr_val =
        (r) ? RGB_LED_ON_VAL : RGB_LED_OFF_VAL;
    board_cb.rgb_led[LED_TYPE_G].curr_val =
        (g) ? RGB_LED_ON_VAL : RGB_LED_OFF_VAL;
    board_cb.rgb_led[LED_TYPE_B].curr_val =
        (b) ? RGB_LED_ON_VAL : RGB_LED_OFF_VAL;

    for (int retry = 0; retry < 3; ++retry) {
        err = ESP_OK;
        for (int i = 0; i < sizeof(reg_led_cb) / sizeof(reg_led_cb[0]); ++i) {
            err |= i2c_BCT3253_writeREG(I2C_MASTER_NUM,
                                        board_cb.rgb_led[i].address,
                                        board_cb.rgb_led[i].curr_val);
        }
        if (err == ESP_OK) {
            break;
        } else {
            ESP_LOGE(TAG, "err: retry[%d] set r[%d] g[%d] b[%d] L%d", retry, r,
                     g, b, __LINE__);
        }
    }

    return err;
}

esp_err_t board_get_weight(uint8_t repeat, float *adc, float *g) {
    unsigned int adc_tmp;
    unsigned int adc_sum = 0;
    esp_err_t err = ESP_OK;

    for (uint8_t i = 0; i < repeat; ++i) {
        err |= i2c_mcp3221_readADC(I2C_MASTER_NUM, &adc_tmp);
        if (err != ESP_OK) {
            continue;
        }
        adc_sum += adc_tmp;
    };

    if (repeat == 0) {
        *adc = 0.0;
    } else {
        *adc = (1.0 * adc_sum) / (1.0 * repeat);
    }

    *g = weight_calculate(*adc, WEIGHT_COEFFICIENT);

    return err;
}

esp_err_t board_set_pir_pwr(bool enable) {
    esp_err_t err = gpio_set_level(GPIO_OUTPUT_PIRPWR, enable);
    return err;
}

bool board_get_pir_status(void) { return (bool)gpio_get_level(GPIO_INPUT_PIR); }

bool board_get_key_status(void) {
    uint8_t port_val = 0;

    if (i2c_MCP23016_readREG(I2C_MASTER_NUM, MCP23016_GPIO1_ADDR, &port_val) !=
        ESP_OK) {
        ESP_LOGE(TAG, "MCP23016 read failed port_val[0x%02x]", port_val);
        return false;
    }

    ESP_LOGD(TAG, "port_val: 0x%X\n", port_val);

    if (((BIT_CHECK(port_val, MCP23016_RESET_KEY_BIT)) >>
         MCP23016_RESET_KEY_BIT)) {
        return false;
    } else {
        return true;
    }
}

esp_err_t board_led_ctrl(led_type_e led, bool enable) {
    ESP_LOGI(TAG, "led: %d, enable: %d", led, enable);

    switch (led) {
    case LED_TYPE_W:
        if (enable) {
            board_set_rgb_led(true, true, true);
        } else {
            board_set_rgb_led(false, false, false);
        }
        break;

    case LED_TYPE_R:
        if (enable) {
            board_set_rgb_led(true, false, false);
        } else {
            board_set_rgb_led(false, false, false);
        }
        break;

    case LED_TYPE_G:
        if (enable) {
            board_set_rgb_led(false, true, false);
        } else {
            board_set_rgb_led(false, false, false);
        }
        break;

    case LED_TYPE_B:
        if (enable) {
            board_set_rgb_led(false, false, true);
        } else {
            board_set_rgb_led(false, false, false);
        }
        break;

    case LED_TYPE_IR:
        if (enable) {
            board_ioe_output_en(MCP23016_GPIO1_ADDR, MCP23016_IR_LED_BIT,
                                GPIO_OUTPUT_HIGH);
        } else {
            board_ioe_output_en(MCP23016_GPIO1_ADDR, MCP23016_IR_LED_BIT,
                                GPIO_OUTPUT_LOW);
        }
        break;
    case LED_TYPE_BD_W:
        if (enable) {
            board_ioe_output_en(MCP23016_GPIO1_ADDR, MCP23016_W_LED_BIT,
                                GPIO_OUTPUT_HIGH);
        } else {
            board_ioe_output_en(MCP23016_GPIO1_ADDR, MCP23016_W_LED_BIT,
                                GPIO_OUTPUT_LOW);
        }
        break;

    default:
        break;
    }

    return ESP_OK;
}

esp_err_t board_init(void) {
    esp_err_t esp_err;

    i2c0_mutex = xSemaphoreCreateMutex();
    esp_err = board_cam_init();
    esp_err |= board_driver_init();

    return esp_err;
}

void sntp_show_time(time_t sec) {
    struct tm timeinfo;
    char strftime_buf[64];

    localtime_r(&sec, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time is: %s", strftime_buf);
}
