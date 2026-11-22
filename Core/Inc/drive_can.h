#ifndef DRIVE_CAN_H
#define DRIVE_CAN_H

#include "main.h"
#include "swerve_config.h"

/* generic-bldc-control の CAN 受信が許容するN5065軸速度。 */
#define DRIVE_MOTOR_MAX_RPM 2000.0f

/* 右前、左前、後ろの順。角度は車輪側、rpmはN5065軸側。 */
typedef struct {
    float steer_rad;
    float motor_rpm;
} WheelCommand;

typedef struct {
    WheelCommand wheels[SWERVE_WHEEL_COUNT];
    uint32_t rx_ms;
    uint32_t count;
} DriveCommand;

void drive_can_start(void);
DriveCommand drive_can_latest_command(void);
HAL_StatusTypeDef drive_can_send_rpm(uint8_t wheel, float motor_rpm,
                                   uint8_t pid_mode, uint8_t control_mode);

#endif
