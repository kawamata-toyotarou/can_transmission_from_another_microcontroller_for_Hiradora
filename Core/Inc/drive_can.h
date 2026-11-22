#ifndef DRIVE_CAN_H
#define DRIVE_CAN_H

#include "main.h"
#include "swerve_config.h"

/* generic-bldc-control の CAN 受信が許容するN5065軸速度。 */
#define DRIVE_MOTOR_MAX_RPM 2000.0f

/* tourobo2026_wsの /cmd_vel から受け取る車体速度。 */
typedef struct {
    float vx_m_s;
    float vy_m_s;
    float omega_rad_s;
    uint32_t rx_ms;
    uint32_t count;
} DriveCommand;

void drive_can_start(void);
DriveCommand drive_can_latest_command(void);
HAL_StatusTypeDef drive_can_send_rpm(uint8_t wheel, float motor_rpm,
                                   uint8_t pid_mode, uint8_t control_mode);

#endif
