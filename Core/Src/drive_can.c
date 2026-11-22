#include "drive_can.h"

#include <math.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan3;

#define CAN_WHEEL_CMD_ID  0x110u
#define CAN_MOTOR_CMD_BASE_ID 0x301u
/* 正規化した角度は±π。送信元は6要素を1つのCAN FDフレームにする。 */
#define COMMAND_MAX_STEER_RAD 3.14159265359f

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
    sFilterConfig.FilterID1    = CAN_WHEEL_CMD_ID;
    sFilterConfig.FilterID2    = CAN_WHEEL_CMD_ID;
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
    if (hfdcan->Instance != FDCAN1 || (events & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];  /* HALはヘッダーのDLCに従い、最大64バイトを書き込む。 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK ||
        header.Identifier != CAN_WHEEL_CMD_ID || header.IdType != FDCAN_STANDARD_ID ||
        header.RxFrameType != FDCAN_DATA_FRAME || header.FDFormat != FDCAN_FD_CAN ||
        header.BitRateSwitch != FDCAN_BRS_ON || header.DataLength != FDCAN_DLC_BYTES_24) {
        return;
    }

    /* 8バイトごとに車輪側角度[rad]、N5065軸速度[rpm]。どちらもBE float32。 */
    WheelCommand wheels[SWERVE_WHEEL_COUNT];
    for (uint8_t wheel = 0; wheel < SWERVE_WHEEL_COUNT; ++wheel) {
        const float steer_rad = be_bytes_to_float(&data[8U * wheel]);
        const float motor_rpm = be_bytes_to_float(&data[8U * wheel + 4U]);
        if (!isfinite(steer_rad) || fabsf(steer_rad) > COMMAND_MAX_STEER_RAD ||
            !isfinite(motor_rpm) || fabsf(motor_rpm) > DRIVE_MOTOR_MAX_RPM) {
            return;
        }
        wheels[wheel] = (WheelCommand){steer_rad, motor_rpm};
    }

    /* 全輪が正常な場合だけ、1フレーム分の指令と受信時刻を更新する。 */
    for (uint8_t wheel = 0; wheel < SWERVE_WHEEL_COUNT; ++wheel) {
        latest_command.wheels[wheel] = wheels[wheel];
    }
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
