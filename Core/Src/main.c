/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) {
    float vx;      /* byte0-3: m/s (そのまま) */
    float vy;      /* byte4-7: m/s (そのまま) */
    float omega;   /* byte8-11: rad/s (そのまま) */
} can_velocity_cmd_t;

typedef struct __attribute__((packed)) {
    float   speed_target;         /* byte0-3 */
    uint8_t pid_mode;             /* byte4: 0=locate_pid, 1=speed_pid */
    uint8_t control_motor_mode;   /* byte5: 0=電圧制御, 1=電流制御 */
    uint8_t reserved[2];          /* byte6-7 */
} can_motor_cmd_t;

/* CAN3 startup error diagnostics: no CAN transmission or automatic recovery. */
typedef struct {
    uint32_t tick;
    uint32_t psr;
    uint32_t ecr;
    uint32_t hal_error;
} CAN3_DiagEvent;

/* robostride_motor_id[] と同じ順番 */
const uint16_t home_sensor_pin[3] = {
    GPIO_PIN_3,  // ID 1
    GPIO_PIN_0,  // ID 2
    GPIO_PIN_2   // ID 127
};

/* 探索方向。逆方向にしたければ、その軸だけ負の値にする */
const float home_speed[3] = {1.5f, 1.5f, 1.5f};

float home_position[3] = {0};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_VELOCITY_CMD_ID  0x100u

#define CAN_MOTOR_CMD_BASE_ID 0x301u

#define WHEEL_RADIUS_M      0.037f  // ホイール半径[m] ← 実機に合わせて変更
#define ROBOT_RADIUS_M      0.31f   // 中心からホイールまでの距離[m] ← 実機に合わせて変更

#define WHEEL0_ANGLE_RAD    (-(float)M_PI / 3.0f)          //  60°  
#define WHEEL1_ANGLE_RAD    ((float)M_PI / 3.0f)          // -60°  
#define WHEEL2_ANGLE_RAD    ((float)M_PI)  

#define MAX_WHEEL_RPM 5000.0f        // doc1/2側のクリップ値と合わせる

#define ROBOSTRIDE_HOST_ID   0xFDu   // stm32側のID

#define ROBOSTRIDE_MOTOR_ID_1  1   // robostride側のID
#define ROBOSTRIDE_MOTOR_ID_2  2
#define ROBOSTRIDE_MOTOR_ID_3  127

#define RS_TYPE_ENABLE       0x03u   // モーターを有効化する命令
#define RS_TYPE_STOP         0x04u   // モーター停止命令
#define RS_TYPE_WRITE_PARAM  0x12u   // パラメータを書き換える命令

#define RS_PARAM_RUN_MODE    0x7005u // 制御モードを指定するパラメータ番号
#define RS_PARAM_LIMIT_CUR   0x7018u // 電流制限のパラメータ番号
#define RS_PARAM_ACC_RAD     0x7022u // 加速度制限のパラメータ番号
#define RS_PARAM_SPD_REF     0x700Au // 目標速度のパラメータ番号

#define RS_PARAM_LOC_REF    0x7016u  // 目標位置
#define RS_PARAM_LIMIT_SPD  0x7017u  // 位置制御中の最大速度制限

#define RS_ID_SCAN_ONLY  0  // 1: ID確認、0: 通常動作

#define motor_0_Pin        GPIO_PIN_3
#define motor_0_GPIO_Port  GPIOC

#define CAN3_DIAG_CAPACITY 16U

/* 停止開始直後の位置と、停止完了位置の差を許容する */
#define HOME_POSITION_TOLERANCE_RAD  0.12f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan3;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t tx_success_count = 0;
volatile uint32_t tx_fail_count = 0;
volatile HAL_StatusTypeDef rs_last_tx_status = HAL_OK;
volatile uint32_t rs_last_tx_id = 0;

volatile float goal_speed_target = 300;
volatile uint8_t goal_pid_mode = 0;
volatile uint8_t goal_control_motor_mode = 0;

volatile float rx_vx    = 0.0f;   /* m/s */
volatile float rx_vy    = 0.0f;   /* m/s */
volatile float rx_omega = 0.0f;   /* rad/s */
volatile uint32_t velocity_rx_count = 0;

const uint8_t robostride_motor_id[3] = {
    ROBOSTRIDE_MOTOR_ID_1,
    ROBOSTRIDE_MOTOR_ID_2,
    ROBOSTRIDE_MOTOR_ID_3
};

volatile int8_t control_mode[3] = {0, 0, 0};   // 0=速度, 1=位置
volatile float robostride_spd_ref[3] = {2.0f, 2.0f, 2.0f};
volatile float robostride_pos_ref[3] = {0.1f, 0.1f, 0.1f};
volatile float wheel_speed_target[3] = {0.0f, 0.0f, 0.0f};
volatile uint16_t rs_last_param_index = 0;

static volatile CAN3_DiagEvent can3_diag_events[CAN3_DIAG_CAPACITY];
static volatile uint32_t can3_diag_count;
static volatile uint32_t can3_diag_busoff_seen;
static volatile uint32_t can3_diag_busoff_psr;
static volatile uint32_t can3_diag_busoff_ecr;

float home_detect_position[3] = {0};
uint8_t home_done[3] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN3_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void FDCAN1_ConfigFilterAndStart(void);
static void compute_wheel_targets(float vx, float vy, float omega);

static HAL_StatusTypeDef robostride_send(uint8_t type, uint16_t data_area, uint8_t motor_id, uint8_t data[8]);
static void put_u16_le(uint8_t *p, uint16_t v);
static void put_float_le(uint8_t *p, float v);
static inline float be_bytes_to_float(const uint8_t *p);
static HAL_StatusTypeDef robostride_write_float(uint8_t motor_id, uint16_t index, float value);
static HAL_StatusTypeDef robostride_write_u8(uint8_t motor_id, uint16_t index, uint8_t value);
static HAL_StatusTypeDef robostride_enable(uint8_t motor_id);
static HAL_StatusTypeDef robostride_stop(uint8_t motor_id);
static void robostride_scan_ids(void);
static int home_read_param(uint8_t id, uint16_t index, void *value, uint8_t size);
static int home_send_ok(HAL_StatusTypeDef status);
static int home_one_motor(uint8_t i);
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 送信要求の間隔を空ける。HAL_OK自体は変更完了の証明ではない */
static int home_send_ok(HAL_StatusTypeDef status)
{
    if (status != HAL_OK) return 0;

    HAL_Delay(10);

    return (hfdcan3.Instance->PSR & FDCAN_PSR_BO) == 0U;
}

static int home_one_motor(uint8_t i)
{
    uint8_t id = robostride_motor_id[i];
    uint8_t mode;
    uint8_t stop_data[8] = {0};
    home_done[i] = 0;

    /* 停止 → 速度モード → 電流制限 → 速度0 */
    if (!home_send_ok(robostride_send(
            RS_TYPE_STOP, ROBOSTRIDE_HOST_ID, id, stop_data)) ||
        !home_send_ok(robostride_write_u8(
            id, RS_PARAM_RUN_MODE, 2)) ||
        !home_send_ok(robostride_write_float(
            id, RS_PARAM_LIMIT_CUR, 1.0f)) ||
        !home_send_ok(robostride_write_float(
            id, RS_PARAM_SPD_REF, 0.0f))) {
        return 0;
    }

    /* 速度モードに切り替わったことを確認 */
    if (!home_read_param(id, RS_PARAM_RUN_MODE, &mode, 1) ||
        mode != 2) {
        return 0;
    }

    if (!home_send_ok(robostride_enable(id))) return 0;

    printf("HOME START id=%u\r\n", (unsigned int)id);
    fflush(stdout);

    uint32_t started = HAL_GetTick();

    /* 初めから0なら、回さず現在位置を原点として扱う */
    if (HAL_GPIO_ReadPin(GPIOC, home_sensor_pin[i])
        == GPIO_PIN_SET) {

        if (!home_send_ok(robostride_write_float(
                id, RS_PARAM_SPD_REF, home_speed[i]))) {
            return 0;
        }

        while (HAL_GPIO_ReadPin(GPIOC, home_sensor_pin[i])
               == GPIO_PIN_SET) {

            /* 見つからなければ45秒で中止 */
            if (HAL_GetTick() - started >= 45000U ||
                (hfdcan3.Instance->PSR & FDCAN_PSR_BO)) {
                return 0;
            }

            HAL_Delay(1);
        }
    }

    /* 0を検出したら速度指令を0にする */
    if (!home_send_ok(robostride_write_float(
            id, RS_PARAM_SPD_REF, 0.0f))) {
        return 0;
    }

    /* 停止指令直後の角度を記録 */
    if (!home_read_param(id, 0x7019u, &home_detect_position[i], 4) ||
      !isfinite(home_detect_position[i])) {
      return 0;
    }

    /* 速度が落ち着くまで待つ */
    uint32_t settling = HAL_GetTick();
    uint8_t still_count = 0;

    while (still_count < 3) {
        float speed;

        if (HAL_GetTick() - settling >= 1000U ||
            !home_read_param(id, 0x701Bu, &speed, 4) ||
            !isfinite(speed)) {
            return 0;
        }

        still_count = fabsf(speed) < 0.03f
                    ? (uint8_t)(still_count + 1) : 0;

        HAL_Delay(20);
    }


    if (!home_send_ok(robostride_send(
            RS_TYPE_STOP, ROBOSTRIDE_HOST_ID, id, stop_data))) {
        return 0;
    }

    /* 現在の多回転角度を読み取って保存 */
    if (!home_read_param(id, 0x7019u, &home_position[i], 4) ||
        !isfinite(home_position[i])) {
        return 0;
    }

    float position_error =
    fabsf(home_position[i] - home_detect_position[i]);

    if (position_error > HOME_POSITION_TOLERANCE_RAD) {
      printf("HOME FAILED: stop movement too large id=%u\r\n", (unsigned int)id);
      fflush(stdout);
      return 0;
    }

    /* 位置モードへ。目標は0ではなく、いま保存した角度 */
    if (!home_send_ok(robostride_write_u8(
            id, RS_PARAM_RUN_MODE, 5)) ||
        !home_send_ok(robostride_write_float(
            id, RS_PARAM_LIMIT_SPD, 0.2f)) ||
        !home_send_ok(robostride_write_float(
            id, RS_PARAM_LOC_REF, home_position[i]))) {
        return 0;
    }

    if (!home_read_param(id, RS_PARAM_RUN_MODE, &mode, 1) ||
        mode != 5) {
        return 0;
    }

    robostride_pos_ref[i] = home_position[i];
    robostride_spd_ref[i] = 0.0f;
    control_mode[i] = 1;

    if (!home_send_ok(robostride_enable(id))) return 0;

    printf("HOME DONE id=%u\r\n", (unsigned int)id);
    fflush(stdout);

    home_done[i] = 1;

    return 1;
}

static int home_read_param(uint8_t id, uint16_t index, void *value, uint8_t size)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[64];

    /* 古い応答を捨てる */
    while (HAL_FDCAN_GetRxFifoFillLevel(
               &hfdcan3, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(
                &hfdcan3, FDCAN_RX_FIFO0, &rx, data) != HAL_OK) {
            return 0;
        }
    }

    uint8_t request[8] = {0};
    put_u16_le(request, index);

    if (robostride_send(0x11u, ROBOSTRIDE_HOST_ID,
                        id, request) != HAL_OK) {
        return 0;
    }

    uint32_t started = HAL_GetTick();

    while (HAL_GetTick() - started < 200U) {
        if (hfdcan3.Instance->PSR & FDCAN_PSR_BO) {
            return 0;
        }

        if (HAL_FDCAN_GetRxFifoFillLevel(
                &hfdcan3, FDCAN_RX_FIFO0) > 0U) {
            if (HAL_FDCAN_GetRxMessage(
                    &hfdcan3, FDCAN_RX_FIFO0,
                    &rx, data) != HAL_OK) {
                return 0;
            }

            if (rx.IdType != FDCAN_EXTENDED_ID ||
                rx.FDFormat != FDCAN_CLASSIC_CAN ||
                rx.RxFrameType != FDCAN_DATA_FRAME ||
                rx.DataLength != FDCAN_DLC_BYTES_8) {
                continue;
            }

            uint8_t type = (rx.Identifier >> 24) & 0x1Fu;
            uint8_t source = (rx.Identifier >> 8) & 0xFFu;
            uint16_t received_index =
                data[0] | ((uint16_t)data[1] << 8);

            if (type == 0x11u &&
                source == id &&
                (rx.Identifier & 0xFFu) == ROBOSTRIDE_HOST_ID &&
                received_index == index) {

                /* bit23:16が0なら読み取り成功 */
                if (((rx.Identifier >> 16) & 0xFFu) != 0U) {
                    return 0;
                }

                memcpy(value, &data[4], size);
                return 1;
            }
        }

        HAL_Delay(1);
    }

    return 0;
}

static void CAN3_DiagCapture(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance != FDCAN3) {
        return;
    }

    /* Reading PSR acknowledges the last error code. Preserve this first read. */
    uint32_t psr = hfdcan->Instance->PSR;
    uint32_t ecr = hfdcan->Instance->ECR;
    uint32_t lec = psr & FDCAN_PSR_LEC;

    if ((psr & FDCAN_PSR_BO) != 0U && can3_diag_busoff_seen == 0U) {
        can3_diag_busoff_psr = psr;
        can3_diag_busoff_ecr = ecr;
        can3_diag_busoff_seen = 1U;
    }

    if (lec == 0U || lec == 7U) {
        return;
    }

    uint32_t index = can3_diag_count;
    if (index < CAN3_DIAG_CAPACITY) {
        can3_diag_events[index].tick = HAL_GetTick();
        can3_diag_events[index].psr = psr;
        can3_diag_events[index].ecr = ecr;
        can3_diag_events[index].hal_error = hfdcan->ErrorCode;
        can3_diag_count = index + 1U;
    }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    CAN3_DiagCapture(hfdcan);
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    (void)ErrorStatusITs;
    CAN3_DiagCapture(hfdcan);
}

/* Call after FDCAN3_ConfigAndStart(), before the first motor command. */
static HAL_StatusTypeDef CAN3_DiagEnable(void)
{
    return HAL_FDCAN_ActivateNotification(&hfdcan3,
        FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR |
        FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING,
        0U);
}

/* Print only from main context, including scan failure and idle logging. */
static void CAN3_DiagPrint(void)
{
    static uint32_t printed;
    static uint32_t busoff_printed;
    uint32_t count = can3_diag_count;

    while (printed < count) {
        uint32_t i = printed;
        uint32_t psr = can3_diag_events[i].psr;
        uint32_t ecr = can3_diag_events[i].ecr;
        printf("CAN3_FIRST[%lu] t=%lu LEC=%lu TEC=%lu REC=%lu "
               "PSR=0x%08lX ECR=0x%08lX HAL=0x%08lX\r\n",
               (unsigned long)i,
               (unsigned long)can3_diag_events[i].tick,
               (unsigned long)(psr & FDCAN_PSR_LEC),
               (unsigned long)((ecr & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos),
               (unsigned long)((ecr & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos),
               (unsigned long)psr, (unsigned long)ecr,
               (unsigned long)can3_diag_events[i].hal_error);
        printed++;
    }

    if (can3_diag_busoff_seen != 0U && busoff_printed == 0U) {
        printf("CAN3_BUSOFF PSR=0x%08lX ECR=0x%08lX\r\n",
               (unsigned long)can3_diag_busoff_psr,
               (unsigned long)can3_diag_busoff_ecr);
        busoff_printed = 1U;
    }
}

/*モーターを動かしてよい状態にする関数*/
static HAL_StatusTypeDef robostride_enable(uint8_t motor_id)
{
    uint8_t data[8] = {0};
    return robostride_send(RS_TYPE_ENABLE, ROBOSTRIDE_HOST_ID, motor_id, data);
}

/*モーターを停止させる関数*/
static HAL_StatusTypeDef robostride_stop(uint8_t motor_id)
{
    uint8_t data[8] = {0};
    data[0] = 1;
    return robostride_send(RS_TYPE_STOP, ROBOSTRIDE_HOST_ID, motor_id, data);
}

/*送信関数 引数typeはRoboStrideの通信タイプ data_areaはCAN拡張IDの中央部分に入れる値*/
static HAL_StatusTypeDef robostride_send(uint8_t type, uint16_t data_area, uint8_t motor_id, uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef tx = {0};

    tx.Identifier = ((uint32_t)type << 24) | ((uint32_t)data_area << 8) | motor_id; //RoboStride用の29bit拡張CANIDを作成
    tx.IdType = FDCAN_EXTENDED_ID;                                                  //ロボマスではstandardidだったがロボストライドでは拡張id
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_8;                                              //RoboStrideの命令は基本8バイト
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0;

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &tx, data);
    rs_last_tx_status = st;
    rs_last_tx_id = tx.Identifier;
    if (st == HAL_OK) {
        tx_success_count++;
    } else {
        tx_fail_count++;
    }
    return st;
}

/*16bitのデータを2つに分解する関数*/
static void put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = v & 0xFF;
    p[1] = (v >> 8) & 0xFF;
}

/*floatの値をRoboStrideに送る形に詰めるための関数 pにvの4バイトがコピーされている*/
static void put_float_le(uint8_t *p, float v)
{
    memcpy(p, &v, 4);
}

/*floatデータから送信データを作成し、送信する関数*/
static HAL_StatusTypeDef robostride_write_float(uint8_t motor_id, uint16_t index, float value)
{
    uint8_t data[8] = {0};

    put_u16_le(&data[0], index);
    put_float_le(&data[4], value);

    rs_last_param_index = index;

    return robostride_send(RS_TYPE_WRITE_PARAM, ROBOSTRIDE_HOST_ID, motor_id, data);
}

/*intデータから送信データを作成し、送信する関数*/
static HAL_StatusTypeDef robostride_write_u8(uint8_t motor_id, uint16_t index, uint8_t value)
{
    uint8_t data[8] = {0};

    put_u16_le(&data[0], index);
    data[4] = value;
    rs_last_param_index = index;

    return robostride_send(RS_TYPE_WRITE_PARAM, ROBOSTRIDE_HOST_ID, motor_id, data);
}

/*受信した4バイトのデータを float に戻す関数*/
static inline float be_bytes_to_float(const uint8_t *p)
{
    union { uint32_t u; float f; } conv;
    conv.u = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
           | ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
    return conv.f;
}

static void FDCAN1_ConfigFilterAndStart(void)
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

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* 3輪オムニの逆運動学: vx,vy[m/s], omega[rad/s] → 各ホイールrpm */
static void compute_wheel_targets(float vx, float vy, float omega)
{
    const float wheel_angle[3] = { WHEEL0_ANGLE_RAD, WHEEL1_ANGLE_RAD, WHEEL2_ANGLE_RAD };

    for (int i = 0; i < 3; i++)
    {
        // 接線方向速度成分 + 回転による寄与
        float v_wheel_mps = -vx * sinf(wheel_angle[i]) + vy * cosf(wheel_angle[i])
                             + ROBOT_RADIUS_M * omega;

        // m/s → rpm変換: rpm = v / (2*pi*r) * 60
        float rpm = v_wheel_mps / (2.0f * (float)M_PI * WHEEL_RADIUS_M) * 60.0f;

        if (rpm > MAX_WHEEL_RPM)  rpm = MAX_WHEEL_RPM;
        if (rpm < -MAX_WHEEL_RPM) rpm = -MAX_WHEEL_RPM;

        wheel_speed_target[i] = rpm;
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance != FDCAN1) return;
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;

    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[64];   // CAN FDは最大64byteなのでバッファを拡張

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        return;
    }

    if (RxHeader.IdType != FDCAN_STANDARD_ID) return;
    if (RxHeader.Identifier != CAN_VELOCITY_CMD_ID) return;
    if (RxHeader.FDFormat != FDCAN_FD_CAN ||
      RxHeader.BitRateSwitch != FDCAN_BRS_ON ||
      RxHeader.DataLength != FDCAN_DLC_BYTES_12)
    {
      return;
    }
    can_velocity_cmd_t cmd;
    
    cmd.vx    = be_bytes_to_float(&RxData[0]);
    cmd.vy    = be_bytes_to_float(&RxData[4]);
    cmd.omega = be_bytes_to_float(&RxData[8]);

    rx_vx    = cmd.vx;      
    rx_vy    = cmd.vy;
    rx_omega = cmd.omega;

    compute_wheel_targets(rx_vy, -rx_vx, rx_omega); 

    velocity_rx_count++;
}

// static void FDCAN3_ConfigAndStart(void)  
// {
//     FDCAN_FilterTypeDef sFilterConfig = {0};

//     /* 基本的に送信用のリポジトリなのでフィルタは全拒否のグローバル設定 */
//     HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT,
//                                   FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

//     if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)   //  hfdcan3
//     {
//         Error_Handler();
//     }
// }

static void robostride_scan_ids(void)
{
    uint8_t seen[256] = {0};
    uint16_t found_count = 0;

    printf("\r\n=== RobStride ID scan start ===\r\n");

    /* 古い受信データを捨てる */
    while (HAL_FDCAN_GetRxFifoFillLevel(
               &hfdcan3, FDCAN_RX_FIFO0) > 0U) {
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[64];

        if (HAL_FDCAN_GetRxMessage(
                &hfdcan3, FDCAN_RX_FIFO0,
                &header, data) != HAL_OK) {
            break;
        }
    }

    /* 8bitのID候補を順番に問い合わせる */
    for (uint16_t candidate = 0; candidate < 256; candidate++) {
        uint8_t request[8] = {0};

        /* 通信タイプ0：ID取得。ID変更やEnableは送らない */
        HAL_StatusTypeDef st = robostride_send(
            0x00,
            ROBOSTRIDE_HOST_ID,
            (uint8_t)candidate,
            request);

        if (st != HAL_OK) {
            CAN3_DiagPrint();
            printf("Scan TX failed: candidate=%u status=%d\r\n",
                   (unsigned int)candidate, (int)st);
            return;
        }

        uint32_t started = HAL_GetTick();

        /* 各IDについて約30ms応答を待つ */
        while ((uint32_t)(HAL_GetTick() - started) < 30U) {
            while (HAL_FDCAN_GetRxFifoFillLevel(
                       &hfdcan3, FDCAN_RX_FIFO0) > 0U) {
                FDCAN_RxHeaderTypeDef header;
                uint8_t data[64];

                if (HAL_FDCAN_GetRxMessage(
                        &hfdcan3, FDCAN_RX_FIFO0,
                        &header, data) != HAL_OK) {
                    break;
                }

                if (header.IdType != FDCAN_EXTENDED_ID ||
                    header.FDFormat != FDCAN_CLASSIC_CAN ||
                    header.RxFrameType != FDCAN_DATA_FRAME ||
                    header.DataLength != FDCAN_DLC_BYTES_8) {
                    continue;
                }

                uint8_t type =
                    (uint8_t)((header.Identifier >> 24) & 0x1FU);

                /*
                 * タイプ0のID応答：
                 * 下位8bitが0xFE、bit23:8にモーターID
                 */
                if (type != 0x00U ||
                    (header.Identifier & 0xFFU) != 0xFEU) {
                    continue;
                }

                uint16_t motor_id =
                    (uint16_t)((header.Identifier >> 8) & 0xFFFFU);

                if (motor_id > 255U || seen[motor_id]) {
                    continue;
                }

                seen[motor_id] = 1;
                found_count++;

                printf("FOUND: motor_id=%u (0x%02X)"
                       " CAN_ID=0x%08lX UID=",
                       (unsigned int)motor_id,
                       (unsigned int)motor_id,
                       (unsigned long)header.Identifier);

                for (uint8_t i = 0; i < 8; i++) {
                    printf("%02X", (unsigned int)data[i]);
                }

                printf("\r\n");
            }

            HAL_Delay(1);
        }
    }

    printf("=== Scan finished: %u distinct ID(s) ===\r\n",
           (unsigned int)found_count);
}

static void FDCAN3_ConfigAndStart(void)
{
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType       = FDCAN_EXTENDED_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0;
    filter.FilterID2    = 0;  // マスク0：すべての拡張IDを通す

    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &filter) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan3,
            FDCAN_REJECT, FDCAN_REJECT,
            FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK) {
        Error_Handler();
    }
}

HAL_StatusTypeDef send_motor_cmd(uint8_t motor_id, float speed_target,
                                  uint8_t pid_mode, uint8_t control_mode)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    can_motor_cmd_t cmd;

    cmd.speed_target = speed_target;
    cmd.pid_mode = pid_mode;
    cmd.control_motor_mode = control_mode;
    cmd.reserved[0] = 0;
    cmd.reserved[1] = 0;

    TxHeader.Identifier = CAN_MOTOR_CMD_BASE_ID + motor_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, (uint8_t*)&cmd);  // hfdcan3
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        //とりあえず今はロボストライド単体で動かすのでerrorメッセージを返してしまうからコメントアウト
        // for (uint8_t i = 0; i < 3; i++)
        // {
        //     HAL_StatusTypeDef st = send_motor_cmd(i, wheel_speed_target[i],
        //                                            goal_pid_mode, goal_control_motor_mode);
        //     if (st == HAL_OK) { tx_success_count++; } else { tx_fail_count++; }
        // }

        static uint32_t tim6_div = 0;

        if (++tim6_div >= 20)   // 20msごとくらい
        {
            tim6_div = 0;

          for (uint8_t i = 0; i < 3; i++) {
            uint8_t id = robostride_motor_id[i];

            if (control_mode[i] == 0) {
              robostride_write_float(id, RS_PARAM_SPD_REF, robostride_spd_ref[i]);
            }
            else if (control_mode[i] == 1) {
              robostride_write_float(id, RS_PARAM_LOC_REF, robostride_pos_ref[i]);
            }
          }

        }

    }
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_FDCAN3_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  FDCAN3_ConfigAndStart();   
  if (CAN3_DiagEnable() != HAL_OK) {
    Error_Handler();
  }
  FDCAN1_ConfigFilterAndStart();
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0); 
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

  if (RS_ID_SCAN_ONLY) {
    HAL_Delay(200);

    robostride_scan_ids();

    /* 後ろのEnable・目標送信・TIM6開始には進まない */
    while (1) {
        HAL_Delay(1000);
        CAN3_DiagPrint();
    }
  }

  uint8_t data[8] = {0};

  // for (uint8_t i = 0; i < 3; i++) {
  //   uint8_t id = robostride_motor_id[i];

  //   robostride_stop(id);
  //   HAL_Delay(100 );

  //   if (control_mode[i] == 0) {
  //       robostride_write_u8(id, RS_PARAM_RUN_MODE, 2);  // 速度
  //       HAL_Delay(100);

  //       robostride_write_float(id, RS_PARAM_LIMIT_CUR, 1.0f);
  //       HAL_Delay(50);
  //   } else if (control_mode[i] == 1) {
  //       robostride_write_u8(id, RS_PARAM_RUN_MODE, 5);  // 位置
  //       HAL_Delay(100);

  //       robostride_write_float(id, RS_PARAM_LIMIT_CUR, 1.0f);
  //       HAL_Delay(50);

  //       robostride_write_float(id, RS_PARAM_LIMIT_SPD, 1.0f);
  //       HAL_Delay(50);
  //   }

  //   robostride_enable(id);
  //   HAL_Delay(100);
  // }
  
  HAL_Delay(1000);

  /* 探索を始める前に3台とも停止させる */
  for (uint8_t i = 0; i < 3; i++) {
    uint8_t stop_data[8] = {0};

    robostride_send(RS_TYPE_STOP, ROBOSTRIDE_HOST_ID,
                    robostride_motor_id[i], stop_data);
    HAL_Delay(10);
  }

  /* ID1 → ID2 → ID127の順に探索 */
  for (uint8_t i = 0; i < 3; i++) {
    if (!home_one_motor(i)) {

        /* 失敗したら3台とも停止要求 */
        for (uint8_t j = 0; j < 3; j++) {
            uint8_t stop_data[8] = {0};

            robostride_send(RS_TYPE_STOP, ROBOSTRIDE_HOST_ID,
                            robostride_motor_id[j], stop_data);
            HAL_Delay(10);
        }

        printf("HOME FAILED id=%u\r\n",
               (unsigned int)robostride_motor_id[i]);
        fflush(stdout);

        /* 通常運転へ進まない */
        while (1) {
            HAL_Delay(100);
        }
    }
  }

  /* 3台とも確認 */
  for (uint8_t i = 0; i < 3; i++) {
    if (home_done[i] == 0){

        for (uint8_t j = 0; j < 3; j++) {
            uint8_t stop_data[8] = {0};
            robostride_send(RS_TYPE_STOP, ROBOSTRIDE_HOST_ID,
                            robostride_motor_id[j], stop_data);
            HAL_Delay(10);
        }

        printf("HOME FAILED: homing is incomplete\r\n");
        fflush(stdout);

        while (1) {
            HAL_Delay(100);
        }
    }
  }

  printf("HOME ALL DONE\r\n");
  fflush(stdout);

  /* 全台の原点探索完了後に、通常の目標送信を開始 */
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  //   static uint32_t last_rx_count = 0;
  //   static uint32_t last_print_tick = 0;

  //   if (HAL_GetTick() - last_print_tick >= 300)
  //   {
  //     CAN3_DiagPrint();
  //     FDCAN_ProtocolStatusTypeDef pstatus3;
  //     FDCAN_ErrorCountersTypeDef  ecounters3;
  //     HAL_FDCAN_GetProtocolStatus(&hfdcan3, &pstatus3);
  //     HAL_FDCAN_GetErrorCounters(&hfdcan3, &ecounters3);

  //     printf("RS/FDCAN3: tx_ok=%lu tx_fail=%lu last_st=%d last_id=0x%08lX BusOff=%d ErrPassive=%d Warning=%d TEC=%lu REC=%lu LEC=%lu Act=%lu\r\n",
  //          (unsigned long)tx_success_count,
  //          (unsigned long)tx_fail_count,
  //          (int)rs_last_tx_status,
  //          (unsigned long)rs_last_tx_id,
  //          pstatus3.BusOff,
  //          pstatus3.ErrorPassive,
  //          pstatus3.Warning,
  //          (unsigned long)ecounters3.TxErrorCnt,
  //          (unsigned long)ecounters3.RxErrorCnt,
  //          (unsigned long)pstatus3.LastErrorCode,
  //          (unsigned long)pstatus3.Activity);

  //     FDCAN_ProtocolStatusTypeDef pstatus1;
  //     FDCAN_ErrorCountersTypeDef  ecounters1;
  //     HAL_FDCAN_GetProtocolStatus(&hfdcan1, &pstatus1);
  //     HAL_FDCAN_GetErrorCounters(&hfdcan1, &ecounters1);

  //     printf("FDCAN1: BusOff=%d ErrPassive=%d Warning=%d TEC=%lu REC=%lu LastErrCode=%lu Activity=%lu\r\n",
  //          pstatus1.BusOff, pstatus1.ErrorPassive, pstatus1.Warning,
  //          (unsigned long)ecounters1.TxErrorCnt,
  //          (unsigned long)ecounters1.RxErrorCnt,
  //          (unsigned long)pstatus1.LastErrorCode,
  //          (unsigned long)pstatus1.Activity);

  //     printf("mode=[%d,%d,%d] last_index=0x%04X pos_ref=[%d,%d,%d] spd_ref=[%d,%d,%d]\r\n",
  //      control_mode[0], control_mode[1], control_mode[2],
  //      rs_last_param_index,
  //      (int)(robostride_pos_ref[0] * 1000.0f),
  //      (int)(robostride_pos_ref[1] * 1000.0f),
  //      (int)(robostride_pos_ref[2] * 1000.0f),
  //      (int)(robostride_spd_ref[0] * 1000.0f),
  //      (int)(robostride_spd_ref[1] * 1000.0f),
  //      (int)(robostride_spd_ref[2] * 1000.0f));

  //     last_print_tick = HAL_GetTick();
  //     fflush(stdout);
  //   }

    

  //   if (velocity_rx_count != last_rx_count)
  //   {
  //     printf("CAN0x100 RX#%d vx=%d vy=%d omega=%d\r\n",
  //     (int64_t)velocity_rx_count, (int64_t)rx_vx, (int64_t)rx_vy, (int64_t)rx_omega);
  //     last_rx_count = velocity_rx_count;
  //   }
  // }
    static uint32_t photo_print_tick = 0;

    if (HAL_GetTick() - photo_print_tick >= 100U)
    {
      photo_print_tick = HAL_GetTick();

      unsigned int photo0 =
        (HAL_GPIO_ReadPin(motor_0_GPIO_Port, motor_0_Pin)
         == GPIO_PIN_SET);

      unsigned int photo1 =
        (HAL_GPIO_ReadPin(motor_1_GPIO_Port, motor_1_Pin)
         == GPIO_PIN_SET);

      unsigned int photo2 =
        (HAL_GPIO_ReadPin(motor_2_GPIO_Port, motor_2_Pin)
         == GPIO_PIN_SET);

      printf("PHOTO motor_0=%u motor_1=%u motor_2=%u\r\n",
           photo0, photo1, photo2);

      fflush(stdout);
    }
  }
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 31;
  hfdcan1.Init.DataTimeSeg2 = 8;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = ENABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 4;
  hfdcan3.Init.NominalSyncJumpWidth = 1;
  hfdcan3.Init.NominalTimeSeg1 = 15;
  hfdcan3.Init.NominalTimeSeg2 = 4;
  hfdcan3.Init.DataPrescaler = 1;
  hfdcan3.Init.DataSyncJumpWidth = 1;
  hfdcan3.Init.DataTimeSeg1 = 15;
  hfdcan3.Init.DataTimeSeg2 = 4;
  hfdcan3.Init.StdFiltersNbr = 1;
  hfdcan3.Init.ExtFiltersNbr = 1;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 39999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : motor_2_Pin motor_1_Pin motor_0_Pin */
  GPIO_InitStruct.Pin = motor_2_Pin|motor_1_Pin|motor_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
