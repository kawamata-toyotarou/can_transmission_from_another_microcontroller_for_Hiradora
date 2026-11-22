#include "drive_can.h"

#include <math.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan3;

#define CAN_VELOCITY_CMD_ID 0x100u
#define CAN_MOTOR_CMD_BASE_ID 0x301u
#define MAX_BODY_SPEED_M_S 4.0f
#define MAX_BODY_OMEGA_RAD_S 6.0f

typedef struct __attribute__((packed)) {
    float   speed_target;                 /* byte0-3: N5065軸rpm、float32リトルエンディアン */
    uint8_t pid_mode;                     /* byte4: 0=locate_pid, 1=speed_pid */
    uint8_t control_motor_mode;           /* byte5: 0=電圧制御, 1=電流制御 */
    uint8_t reserved[2];                  /* byte6-7 */
} can_motor_cmd_t;

_Static_assert(sizeof(can_motor_cmd_t) == 8U, "Hiradora command must be 8 bytes");

/* FDCAN1割り込みが更新。メインループは1フレーム分をまとめて読む。 */
static volatile DriveCommand latest_command;

DriveCommand drive_can_latest_command(void)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const DriveCommand command = latest_command;
    __set_PRIMASK(mask);
    return command;
}

static inline float be_bytes_to_float(const uint8_t *p)
{
    union { uint32_t u; float f; } conv;
    conv.u = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
           | ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
    return conv.f;
}

void drive_can_start(void)
{
    FDCAN_FilterTypeDef sFilterConfig = {0};

    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 0;
    sFilterConfig.FilterType   = FDCAN_FILTER_DUAL;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = CAN_VELOCITY_CMD_ID;
    sFilterConfig.FilterID2    = CAN_VELOCITY_CMD_ID;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                      FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)  // HAL_FDCAN_RxFifo0Callbackが呼ばれる
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t events)
{
    /* FDCAN1の新着受信か確認 */
    if (hfdcan->Instance != FDCAN1 || (events & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];  /* HALはヘッダーのDLCに従い、最大64バイトを書き込む。 */

    /* フレームを1件取り出し、車体速度指令の形式を確認する。 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK || header.Identifier != CAN_VELOCITY_CMD_ID || header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME || header.FDFormat != FDCAN_FD_CAN || header.BitRateSwitch != FDCAN_BRS_ON || header.DataLength != FDCAN_DLC_BYTES_12) {
        return;
    }

    /* 4バイトずつ、vx [m/s]、vy [m/s]、omega [rad/s]。すべてBE float32。 */
    const float vx_m_s = be_bytes_to_float(&data[0]);
    const float vy_m_s = be_bytes_to_float(&data[4]);
    const float omega_rad_s = be_bytes_to_float(&data[8]);
    if (!isfinite(vx_m_s) || !isfinite(vy_m_s) || !isfinite(omega_rad_s) || hypotf(vx_m_s, vy_m_s) > MAX_BODY_SPEED_M_S || fabsf(omega_rad_s) > MAX_BODY_OMEGA_RAD_S) {
        return;
    }

    /* 正常な指令だけを保存し、受信時刻を更新する。 */
    latest_command.vx_m_s = vx_m_s;
    latest_command.vy_m_s = vy_m_s;
    latest_command.omega_rad_s = omega_rad_s;
    latest_command.rx_ms = HAL_GetTick();
    latest_command.count++;
}

HAL_StatusTypeDef drive_can_send_rpm(uint8_t wheel, float motor_rpm,
                                  uint8_t pid_mode, uint8_t control_mode)
{
    FDCAN_TxHeaderTypeDef TxHeader = {0};
    can_motor_cmd_t cmd;

    if (wheel >= SWERVE_WHEEL_COUNT || !isfinite(motor_rpm) ||
        fabsf(motor_rpm) > DRIVE_MOTOR_MAX_RPM || pid_mode > 1U || control_mode > 1U) {
        return HAL_ERROR;
    }

    cmd.speed_target = motor_rpm;
    cmd.pid_mode = pid_mode;
    cmd.control_motor_mode = control_mode;
    cmd.reserved[0] = 0;
    cmd.reserved[1] = 0;

    TxHeader.Identifier = CAN_MOTOR_CMD_BASE_ID + wheel;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, (uint8_t *)&cmd);
}
