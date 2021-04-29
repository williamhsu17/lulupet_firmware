#include "app_cmd.h"
#include "argtable3/argtable3.h"
#include "driver/uart.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "linenoise/linenoise.h"
#include <stdio.h>
#include <string.h>

static const char *prompt;

static void cmd_task(void *pvParameter) {
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

static void initialize_console() {
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
    initialize_console();

    printf("\n ==================================================\n");
    printf(" |        LuluPet AI Litter Box v%d.%d.%d            |\n",
           VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
    printf(" |                                                |\n");
    printf(" |    Print 'help' to gain overview of commands   |\n");
    printf(" |                                                |\n");
    printf(" ==================================================\n\n");
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
