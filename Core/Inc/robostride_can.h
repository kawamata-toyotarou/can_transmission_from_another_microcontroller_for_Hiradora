#ifndef ROBOSTRIDE_CAN_H
#define ROBOSTRIDE_CAN_H

#include <stdbool.h>
#include "main.h"

#define RS_PARAM_RUN_MODE   0x7005u
#define RS_PARAM_SPD_REF    0x700Au
#define RS_PARAM_LOC_REF    0x7016u
#define RS_PARAM_LIMIT_SPD  0x7017u
#define RS_PARAM_LIMIT_CUR  0x7018u
#define RS_PARAM_MECH_POS   0x7019u
#define RS_PARAM_MECH_SPEED 0x701Bu

#define STEER_FEEDBACK_TIMEOUT_MS 60U

typedef struct {
    float angle_rad;    /* RobStride出力軸の多回転角度 */
    uint32_t rx_ms;
    bool valid;
    bool fault;
} SteeringFeedback;

void robostride_can_start(void);
void robostride_receive(void);
bool robostride_tx_ready(void);
bool robostride_wait_idle(void);

HAL_StatusTypeDef robostride_enable(uint8_t id);
HAL_StatusTypeDef robostride_stop(uint8_t id, bool clear_fault);
HAL_StatusTypeDef robostride_write_float(uint8_t id, uint16_t index, float value);
HAL_StatusTypeDef robostride_write_u8(uint8_t id, uint16_t index, uint8_t value);

/* 同期読み出しは起動時の原点復帰専用。走行中は角度要求と返信の処理を分ける。 */
bool robostride_read_parameter(uint8_t id, uint16_t index, void *value,
                               uint8_t size, uint32_t timeout_ms);
HAL_StatusTypeDef robostride_request_angle(uint8_t wheel, uint32_t now);
const SteeringFeedback *robostride_feedback(uint8_t wheel);
void robostride_set_homed(uint8_t wheel);

void robostride_scan_ids(void);
void robostride_print_startup_errors(void);

#endif
