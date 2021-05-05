#ifndef _APP_WEIGHT_H_
#define _APP_WEIGHT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#define WEIGHT_TASK_BUFFER_SIZE 100
#define WEIGHT_ACTIVE_VAL 200.0 // unit:g
#define WEIGHT_CAT_VAL 2000.0   // unit:g
#define WEIGHT_COEFFICIENT 20000.0 / 4096.0
#define WEIGHT_STANDBY_PERIOD 10     // ms
#define WEIGHT_JUMP_PERIOD 1000      // ms
#define WEIGHT_BIGJUMP_PERIOD 1000   // ms
#define WEIGHT_POSTEVENT_PERIOD 1000 // ms
#define WEIGHT_JUMP_TO_STANDBY_CHECK_TIMES 4
#define WEIGHT_JUMP_TO_BIGJUMP_CHECK_TIMES 4
#define WEIGHT_JUMP_CHECK_TIMES 25
#define WEIGHT_POSTEVENT_CHECK_TIMES 1

enum weight_task_fsm {
    WEIGHT_TASK_STAT_START = 0,
    WEIGHT_TASK_STAT_STANDBY,
    WEIGHT_TASK_STAT_JUMP,
    WEIGHT_TASK_STAT_BIGJUMP,
    WEIGHT_TASK_STAT_POSTEVENT,
};

typedef struct {
    // weight state machine
    enum weight_task_fsm now_stat;
    enum weight_task_fsm pre_stat;

    // weight data
    unsigned int ring_buffer[WEIGHT_TASK_BUFFER_SIZE]; // TODO: the buffer just
                                                       // for verifying
    uint32_t weight_pause_times;
    unsigned int ref_adc_sum;
    bool ring_buffer_loop;
    uint32_t ring_buffer_idx;
    float ref_adc;
    float latest_adc;
    float weight_coefficent;

    float ref_weight; // unit:g
    float now_weight; // unit:g

    bool ref_adc_exec;

    float active_weight; // param can be set by user
    float cat_weight;    // param can be set by user

    uint8_t jump_to_standby_num;
    uint8_t jump_to_standby_cnt;
    uint8_t jump_to_standby_chk; // param can be set by user
    uint8_t jump_to_bigjump_num;
    uint8_t jump_to_bigjump_cnt;
    uint8_t jump_to_bigjump_chk; // param can be set by user

    uint8_t jump_num;
    uint8_t jump_cnt;
    uint8_t jump_chk; // param can be set by user

    uint8_t postevnet_num;
    uint8_t postevnet_cnt;
    uint8_t postevnet_chk; // param can be set by user

    uint32_t jump_pause_times;    // param can be set by user
    uint32_t standby_period_ms;   // param can be set by user
    uint32_t jump_period_ms;      // param can be set by user
    uint32_t bugjump_period_ms;   // param can be set by user
    uint32_t postevent_period_ms; // param can be set by user

    uint32_t period_ms;
    uint32_t period_cnt;

    int pir_level;
} weight_task_cb;

void app_weight_main(void);

#ifdef __cplusplus
}
#endif

#endif // end of _APP_WEIGHT_H_
