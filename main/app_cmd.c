#include "app_cmd.h"
#include "argtable3/argtable3.h"
#include "driver/uart.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "include/app_weight.h"
#include "include/board_driver.h"
#include "include/util.h"
#include "linenoise/linenoise.h"
#include "sys/queue.h"
#include <stdio.h>
#include <string.h>

#define TAG "cmd"

#define BANNER                                                                 \
    "\n =================================================="                    \
    "\n |                                                |"                    \
    "\n |        LuluPet AI Litter Box v%d.%d.%d            |"                 \
    "\n |                                                |"                    \
    "\n |    Print 'help' to gain overview of commands   |"                    \
    "\n |                                                |"                    \
    "\n =================================================="                    \
    "\n"

static const char *prompt;

#define PARSE_ARG(args)                                                        \
    do {                                                                       \
        int err = arg_parse(argc, argv, (void **)&(args));                     \
        if (err != 0) {                                                        \
            arg_print_errors(stderr, args.end, argv[0]);                       \
            return ESP_ERR_INVALID_ARG;                                        \
        }                                                                      \
    } while (0)

static int cmd_led_set(int argc, char **argv);
static esp_err_t register_led_command(void);
#if (FUNC_WEIGHT_FAKE)
static int cmd_weight_condition_set(int argc, char **argv);
#endif
static int cmd_weight_get_param(int argc, char **argv);
static esp_err_t register_weight_command(void);

static int cmd_button_status(int argc, char **argv);
static esp_err_t register_button_command(void);

static int cmd_button_status(int argc, char **argv) {

    uint8_t port_val;

    i2c_MCP23016_readREG(I2C_MASTER_NUM, MCP23016_GPIO1_ADDR, &port_val);

    ESP_LOGD(TAG, "port_val: 0x%X\n", port_val);
    printf("button: %s\n",
           ((BIT_CHECK(port_val, MCP23016_BUTTON_BIT)) >> MCP23016_BUTTON_BIT)
               ? "release"
               : "press");

    return 0;
}

static esp_err_t register_button_command(void) {

    const esp_console_cmd_t cmds[] = {
        {
            .command = "btn_status",
            .help = "button status",
            .hint = NULL,
            .func = &cmd_button_status,
            .argtable = NULL,
        },
    };

    for (int i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }

    return ESP_OK;
}

struct {
    struct arg_str *led_type;
    struct arg_int *en;
    struct arg_end *end;
} cmd_led_set_args;

extern weight_task_cb w_task_cb;

static int cmd_led_set(int argc, char **argv) {
    esp_err_t err = ESP_OK;

    PARSE_ARG(cmd_led_set_args);

    if (cmd_led_set_args.led_type->count == 0 ||
        cmd_led_set_args.en->count == 0) {
        printf("param err");
        goto cmd_led_set_err;
    }

    if (cmd_led_set_args.en->ival[0] != 0 &&
        cmd_led_set_args.en->ival[0] != 1) {
        printf("en <0|1>\n");
        goto cmd_led_set_err;
    }

    if (strcmp(cmd_led_set_args.led_type->sval[0], "w") == 0)
        err = board_led_ctrl(LED_TYPE_W, (bool)cmd_led_set_args.en->ival[0]);
    else if (strcmp(cmd_led_set_args.led_type->sval[0], "r") == 0)
        err = board_led_ctrl(LED_TYPE_R, (bool)cmd_led_set_args.en->ival[0]);
    else if (strcmp(cmd_led_set_args.led_type->sval[0], "g") == 0)
        err = board_led_ctrl(LED_TYPE_G, (bool)cmd_led_set_args.en->ival[0]);
    else if (strcmp(cmd_led_set_args.led_type->sval[0], "b") == 0)
        err = board_led_ctrl(LED_TYPE_B, (bool)cmd_led_set_args.en->ival[0]);
    else if (strcmp(cmd_led_set_args.led_type->sval[0], "IR") == 0)
        err = board_led_ctrl(LED_TYPE_IR, (bool)cmd_led_set_args.en->ival[0]);
    else if (strcmp(cmd_led_set_args.led_type->sval[0], "W") == 0)
        err = board_led_ctrl(LED_TYPE_BD_W, (bool)cmd_led_set_args.en->ival[0]);
    else
        printf("led_type <w|r|g|b|IR|W>\n");

    if (err == ESP_OK)
        printf("ok\n");
    else
        printf("err: %s\n", esp_err_to_name(err));

    return 0;

cmd_led_set_err:
    arg_print_errors(stderr, cmd_led_set_args.end, argv[0]);
    return -1;
}

static esp_err_t register_led_command(void) {
    cmd_led_set_args.led_type =
        arg_str1("t", "led_type", "<w|r|g|b|IR|W>", "led type");
    cmd_led_set_args.en = arg_int0("e", "enable", "<0|1>", "enable led");
    cmd_led_set_args.end = arg_end(2);

    const esp_console_cmd_t cmds[] = {
        {
            .command = "led_set",
            .help = "led set",
            .hint = NULL,
            .func = &cmd_led_set,
            .argtable = &cmd_led_set_args,
        },
    };

    for (int i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }

    return ESP_OK;
}

#if (FUNC_WEIGHT_FAKE)
struct {
    struct arg_str *condition_type;
    struct arg_int *value;
    struct arg_end *end;
} cmd_weight_condition_set_args;

static int cmd_weight_condition_set(int argc, char **argv) {
    PARSE_ARG(cmd_weight_condition_set_args);

    if (cmd_weight_condition_set_args.condition_type->count == 0 ||
        cmd_weight_condition_set_args.value->count == 0) {
        printf("param err");
        goto cmd_weight_condition_set_err;
    }

    if (strcmp(cmd_weight_condition_set_args.condition_type->sval[0],
               "adc_w") == 0)
        w_task_cb.now_weight =
            cmd_weight_condition_set_args.value->ival[0] * 1.0;
    else if (strcmp(cmd_weight_condition_set_args.condition_type->sval[0],
                    "ref_w") == 0)
        w_task_cb.ref_weight =
            cmd_weight_condition_set_args.value->ival[0] * 1.0;
    else if (strcmp(cmd_weight_condition_set_args.condition_type->sval[0],
                    "pir") == 0)
        w_task_cb.pir_level = cmd_weight_condition_set_args.value->ival[0];
    else
        printf("weight_condition <adc_w|ref_w|pir>\n");

    return 0;

cmd_weight_condition_set_err:
    arg_print_errors(stderr, cmd_weight_condition_set_args.end, argv[0]);
    return -1;
}
#endif

struct {
    struct arg_str *list;
    struct arg_end *end;
} cmd_weight_get_param_args;

static int cmd_weight_get_param(int argc, char **argv) {
    PARSE_ARG(cmd_weight_get_param_args);

    printf("active_weight: %.2f\n", w_task_cb.active_weight);
    printf("cat_weight: %.2f\n", w_task_cb.cat_weight);
    printf("jump_to_standby_chk: %d\n", w_task_cb.jump_to_standby_chk);
    printf("jump_to_bigjump_chk: %d\n", w_task_cb.jump_to_bigjump_chk);
    printf("jump_chk: %d\n", w_task_cb.jump_chk);
    printf("postevnet_chk: %d\n", w_task_cb.postevnet_chk);
    printf("jump_pause_times: %d\n", w_task_cb.jump_pause_times);
    printf("standby_period_ms: %d\n", w_task_cb.standby_period_ms);
    printf("jump_period_ms: %d\n", w_task_cb.jump_period_ms);
    printf("bugjump_period_ms: %d\n", w_task_cb.bugjump_period_ms);
    printf("postevent_period_ms: %d\n", w_task_cb.postevent_period_ms);
    printf("pir_level: %d\n", gpio_get_level(GPIO_INPUT_PIR));

    return 0;
}

static esp_err_t register_weight_command(void) {

#if (FUNC_WEIGHT_FAKE)
    cmd_weight_condition_set_args.condition_type = arg_str1(
        "c", "weight_condition", "<adc_w|ref_w|pir>", "weight condition type");
    cmd_weight_condition_set_args.value =
        arg_int0("v", "value", "", "condition value");
    cmd_weight_condition_set_args.end = arg_end(2);
#endif

    cmd_weight_get_param_args.list =
        arg_str1("l", "weight_get_param", "<1>", "weight list parameters");
    cmd_weight_get_param_args.end = arg_end(1);

    const esp_console_cmd_t cmds[] = {
#if (FUNC_WEIGHT_FAKE)
        {
            .command = "weight_condition_set",
            .help = "weight condition set",
            .hint = NULL,
            .func = &cmd_weight_condition_set,
            .argtable = &cmd_weight_condition_set_args,
        },
#endif
        {
            .command = "weight_get",
            .help = "weight get",
            .hint = NULL,
            .func = &cmd_weight_get_param,
            .argtable = &cmd_weight_get_param_args,
        },
    };

    for (int i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }

    return ESP_OK;
}

static void cmd_task(void *pvParameter) {
    ESP_ERROR_CHECK(esp_console_register_help_command());
    ESP_ERROR_CHECK(register_led_command());
    ESP_ERROR_CHECK(register_weight_command());
    ESP_ERROR_CHECK(register_button_command());

    for (;;) {
        /* Main loop */
        while (true) {
            /* Get a line using linenoise.
             * The line is returned when ENTER is pressed.
             */
            char *line = linenoise(prompt);
            if (line == NULL) { /* Ignore empty lines */
                continue;
            }
            /* Add the command to the history */
            linenoiseHistoryAdd(line);

            /* Try to run the command */
            int ret;
            esp_err_t err = esp_console_run(line, &ret);
            if (err == ESP_ERR_NOT_FOUND) {
                printf("Unrecognized command\n");
            } else if (err == ESP_OK && ret != ESP_OK) {
                printf("Command returned non-zero error code: 0x%x\n", ret);
            } else if (err != ESP_OK) {
                printf("Internal error: %s\n", esp_err_to_name(err));
            }
            /* linenoise allocates line buffer on the heap, so need to free it
             */
            linenoiseFree(line);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS); // run every 0..05sec
    }
}

static void console_init() {
    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK(
        uart_driver_install(CONFIG_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CONFIG_CONSOLE_UART_NUM);

    /* Initialize the console */
    esp_console_config_t console_config = {
        .max_cmdline_args = 32,
        .max_cmdline_length = 256,
#if CONFIG_LOG_COLORS
        .hint_color = atoi(LOG_COLOR_CYAN)
#endif
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    /* Configure linenoise line completion library */
    /* Enable multiline editing. If not set, long commands will scroll within
     * single line.
     */
    linenoiseSetMultiLine(1);

    /* Tell linenoise where to get command completions and hints */
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback *)&esp_console_get_hint);

    /* Set command history size */
    linenoiseHistorySetMaxLen(100);
}

void app_cmd_main(void) {
    console_init();

    printf(BANNER, VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);

    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    prompt = LOG_COLOR_I "lulupet> " LOG_RESET_COLOR;

    /* Figure out if the terminal supports escape sequences */
    int probe_status = linenoiseProbe();
    if (probe_status) { /* zero indicates success */
        printf("\n"
               "Your terminal application does not support escape sequences.\n"
               "Line editing and history features are disabled.\n"
               "On Windows, try using Putty instead.\n");
        linenoiseSetDumbMode(1);
#if CONFIG_LOG_COLORS
        /* Since the terminal doesn't support escape sequences,
         * don't use color codes in the prompt.
         */
        prompt = "lulupet> ";
#endif // CONFIG_LOG_COLORS
    }

    xTaskCreate(&cmd_task, "cmd_task", 4096, NULL, 5, NULL);
}
