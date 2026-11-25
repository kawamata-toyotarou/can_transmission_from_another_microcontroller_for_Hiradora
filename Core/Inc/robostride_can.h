#ifndef ROBOSTRIDE_CAN_H
#define ROBOSTRIDE_CAN_H

#include <stdbool.h>
#include "main.h"

/* robostride_receive()が返信の data[4]〜data[7]から値を取り出し */
#define RS_PARAM_RUN_MODE   0x7005u  // 制御モード。2 = 速度制御、5 = 位置制御（CSP）
#define RS_PARAM_SPD_REF    0x700Au  // 目標速度 [rad/s]。原点探索で回す速度を指定する
#define RS_PARAM_LOC_REF    0x7016u  // 目標角度 [rad]。位置制御で出力軸を動かす先を指定する
#define RS_PARAM_LIMIT_SPD  0x7017u  // 位置制御中の速度上限 [rad/s]。目標角度へ動く速さを制限する
#define RS_PARAM_LIMIT_CUR  0x7018u  // 電流上限 [A]。原点探索中などに出せるトルクを抑える
#define RS_PARAM_MECH_POS   0x7019u  // 実測の多回転角度 [rad]。探索中の移動量と原点位置を確認する
#define RS_PARAM_MECH_SPEED 0x701Bu  // 実測の角速度 [rad/s]。速度0の指令後に停止を確認する

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
