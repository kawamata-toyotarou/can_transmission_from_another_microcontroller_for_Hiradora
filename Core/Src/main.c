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
typedef struct __attribute__((packed)) { /* __attribute__((packed))で余計なメモリを入れない設定に */
    float vx;                            /* byte0-3: m/s (そのまま) */
    float vy;                            /* byte4-7: m/s (そのまま) */
    float omega;                         /* byte8-11: rad/s (そのまま) */
} can_velocity_cmd_t;

typedef struct __attribute__((packed)) {  /* __attribute__((packed))で余計なメモリを入れない設定に */
    float   speed_target;                 /* byte0-3 */
    uint8_t pid_mode;                     /* byte4: 0=locate_pid, 1=speed_pid */
    uint8_t control_motor_mode;           /* byte5: 0=電圧制御, 1=電流制御 */
    uint8_t reserved[2];                  /* byte6-7 */
} can_motor_cmd_t;

/* CAN3 startup error diagnostics: no CAN transmission or automatic recovery. */
typedef struct {
    uint32_t tick;                       /* エラーが記録された時刻 */
    uint32_t psr;                        /* CAN通信の状態を示すレジスタの値 */
    uint32_t ecr;                        /* CAN通信のエラーカウンタの値 */
    uint32_t hal_error;                  /* STM32のHALが管理するエラー情報 */
} CAN3_DiagEvent;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 原点を基準とした、車輪の操舵角の上限 */
#define STEER_LIMIT_DEG 90.0f
#define STEER_LIMIT_RAD (STEER_LIMIT_DEG * (float)M_PI / 180.0f)

#define CAN_VELOCITY_CMD_ID  0x100u      // 速度指令を受信するCANID
#define CAN_MOTOR_CMD_BASE_ID 0x301u     // 車輪駆動モーターへの送信IDの基準
#define ROBOSTRIDE_HOST_ID   0xFDu       // stm32側のID

/* robostride側のID*/
#define ROBOSTRIDE_MOTOR_ID_1  1
#define ROBOSTRIDE_MOTOR_ID_2  2
#define ROBOSTRIDE_MOTOR_ID_3  127

/* RoboStrideの命令タイプ */
#define RS_TYPE_ENABLE       0x03u       // モーターを有効化する命令
#define RS_TYPE_STOP         0x04u       // モーター停止命令
#define RS_TYPE_WRITE_PARAM  0x12u       // パラメータを書き換える命令

/* RoboStrideのパラメータ番号 */
#define RS_PARAM_RUN_MODE    0x7005u     // 制御モードを指定するパラメータ番号
#define RS_PARAM_LIMIT_CUR   0x7018u     // 電流制限のパラメータ番号
#define RS_PARAM_ACC_RAD     0x7022u     // 加速度制限のパラメータ番号
#define RS_PARAM_SPD_REF     0x700Au     // 目標速度のパラメータ番号
#define RS_PARAM_LOC_REF     0x7016u     // 目標位置
#define RS_PARAM_LIMIT_SPD   0x7017u     // 位置制御中の最大速度制限
 
#define MAX_WHEEL_RPM 5000.0f            // 輪駆動モーターに送る回転速度指令の上限を5000 rpmに設定 
#define MAX_BODY_SPEED_M_S    4.0f
#define MAX_BODY_OMEGA_RAD_S  6.0f

#define RS_ID_SCAN_ONLY  0               // 1: ID確認、0: 通常動作

#define motor_0_Pin        GPIO_PIN_3
#define motor_0_GPIO_Port  GPIOC

#define CAN3_DIAG_CAPACITY 16U           // CAN3の診断イベントを最大16件保存

/* 停止開始直後の位置と、停止完了位置の差を許容する */
#define HOME_POSITION_TOLERANCE_RAD  0.12f

#define SWERVE_CMD_TIMEOUT_MS 300U
#define SWERVE_ALIGN_RAD      0.174533f  /* 10度 */

#define SWERVE_PERIOD_MS 20U
#define STEER_FEEDBACK_TIMEOUT_MS 60U
#define HOME_TIMEOUT_MS 45000U
#define RS_PARAM_MECH_POS 0x7019u
#define RS_TYPE_READ_PARAM 0x11u

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
volatile uint8_t motor_update_due = 0;   // モーターの制御処理を実行するタイミングになったかどうかを示す変数

/* 前+X、左+Y。操舵座標・車輪半径はm。伝達比はモーター軸回転/車輪側回転。
 * home_speedはRobStride出力軸rad/s、探索範囲とLOW離脱範囲は車輪側rad。
 * 探索範囲は起動位置基準の暫定値。機械端までの余裕は実機で確認する。 */
typedef struct {
    float x, y, wheel_radius;
    float steer_ratio, steer_sign;
    float drive_ratio, drive_sign;
    float home_speed, home_travel, home_release;
    uint16_t home_pin;
    uint8_t steer_id;
} SwerveModule;

static const SwerveModule modules[3] = {
    { 0.155f, -0.2684679f, 0.037f, 3.0f, 1.0f, 1.0f, -1.0f, 1.5f, STEER_LIMIT_RAD, 0.174533f, GPIO_PIN_3, ROBOSTRIDE_MOTOR_ID_1 },
    { 0.155f,  0.2684679f, 0.037f, 3.0f, 1.0f, 1.0f, -1.0f, 1.5f, STEER_LIMIT_RAD, 0.174533f, GPIO_PIN_0, ROBOSTRIDE_MOTOR_ID_2 },
    {-0.310f,  0.0f,       0.037f, 3.0f, 1.0f, 1.0f, -1.0f, 1.5f, STEER_LIMIT_RAD, 0.174533f, GPIO_PIN_2, ROBOSTRIDE_MOTOR_ID_3 }
};

typedef struct {
    float angle;
    uint32_t rx_ms, request_ms;
    uint8_t valid, pending, fault;
} SteeringFeedback;
static SteeringFeedback steering[3];

static struct {
    uint16_t index;
    uint8_t id, waiting, received, ok;
    uint8_t data[4];
} home_reply;

static uint32_t swerve_period_ms, swerve_max_us;
static uint32_t swerve_tx_delay, swerve_tx_fail;
static volatile uint32_t swerve_missed_ticks;

volatile uint32_t velocity_rx_tick = 0;
volatile uint8_t swerve_drive_enable = 1;
float home_position[3] = {0};
float home_detect_position[3] = {0};
uint8_t home_done[3] = {0};

volatile uint32_t tx_success_count = 0;                               // CAN送信要求が成功した回数
volatile uint32_t tx_fail_count = 0;                                  // CAN送信要求が失敗した回数
volatile HAL_StatusTypeDef rs_last_tx_status = HAL_OK;                // 最後のRoboStride送信処理のHALステータス
volatile uint32_t rs_last_tx_id = 0;                                  // 最後に送信要求したCAN ID
volatile uint32_t velocity_rx_count = 0;                              // 速度指令の受信回数

volatile float goal_speed_target = 300;                               // 速度目標 rpm
volatile uint8_t goal_pid_mode = 0;                                   // 0なら位置pidで1から速度pid
volatile uint8_t goal_control_motor_mode = 0;                         // 0なら電圧制御 1なら電流制御
volatile float rx_vx    = 0.0f;                                       /* m/s */
volatile float rx_vy    = 0.0f;                                       /* m/s */
volatile float rx_omega = 0.0f;                                       /* rad/s */

volatile int8_t control_mode[3] = {0, 0, 0};              // RoboStrideの制御モードを管理する配列 0=速度, 1=位置
volatile float robostride_spd_ref[3] = {2.0f, 2.0f, 2.0f};// RoboStrideの速度位置を保存
volatile float robostride_pos_ref[3] = {0.1f, 0.1f, 0.1f};// RoboStrideの目標位置を保存
volatile float wheel_speed_target[3] = {0.0f, 0.0f, 0.0f};// 各車輪駆動モーターの目標速度を保存
volatile uint16_t rs_last_param_index = 0;

/* CAN3のエラー記録 */
static volatile CAN3_DiagEvent can3_diag_events[CAN3_DIAG_CAPACITY];  // 16件分のエラー情報を保存
static volatile uint32_t can3_diag_count;
static volatile uint32_t can3_diag_busoff_seen;
static volatile uint32_t can3_diag_busoff_psr;
static volatile uint32_t can3_diag_busoff_ecr;

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
//static void compute_wheel_targets(float vx, float vy, float omega);

static HAL_StatusTypeDef robostride_send(uint8_t type, uint16_t data_area, uint8_t motor_id, uint8_t data[8]); // RoboStrideにCANメッセージを送信するための関数
static void put_u16_le(uint8_t *p, uint16_t v);
static void put_float_le(uint8_t *p, float v);
static inline float be_bytes_to_float(const uint8_t *p);
static HAL_StatusTypeDef robostride_write_float(uint8_t motor_id, uint16_t index, float value);                // float型のパラメータを書き込む
static HAL_StatusTypeDef robostride_write_u8(uint8_t motor_id, uint16_t index, uint8_t value);                 // 8ビット整数のパラメータを書き込む
static HAL_StatusTypeDef robostride_enable(uint8_t motor_id);                                                  // モーターの有効化
static HAL_StatusTypeDef robostride_stop(uint8_t motor_id);                                                    // モーターの停止
static void robostride_scan_ids(void);                                                                         // モーターIDの探索                       
static int home_read_param(uint8_t id, uint16_t index, void *value, uint8_t size);                             // モーターのパラメータを読み取る
static int home_send_ok(HAL_StatusTypeDef status);                                                             // 送信結果の確認と待機
static int home_one_motor(uint8_t i);                                                                          // 1台の原点復帰
static void compute_wheel_targets(float vx, float vy, float omega);                                            // 3輪の目標角度・速度を計算
static void swerve_task(void);                                                                                 // 独立ステアリングの制御処理
static void can3_receive(void);
static HAL_StatusTypeDef request_parameter(uint8_t id, uint16_t index);
static int can3_wait_idle(void);                                                                               // CAN3の送信待機
HAL_StatusTypeDef send_motor_cmd(uint8_t motor_id, float speed_target, uint8_t pid_mode, uint8_t control_mode);// n5065にcanを送信する関数
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* CAN通信の送信待機関数 */
static int can3_wait_idle(void)
{
    uint32_t started = HAL_GetTick();                    // 開始時刻を取得

    while (hfdcan3.Instance->TXBRP != 0U) {              // CANの送信要求が残っているか確認 TXBRPはFDCANの送信要求保留レジスタで送信要求が保留されていると、対応するビットが1
        if ((hfdcan3.Instance->PSR & FDCAN_PSR_BO) ||    // FDCAN_PSR_BO は、Bus-Off状態を表すビットのマスク
            HAL_GetTick() - started >= 10U) {            // 10ms以上経過したか
            return 0;   
        }

        HAL_Delay(1);
    }

    return (hfdcan3.Instance->PSR & FDCAN_PSR_BO) == 0U; // Bus-Offを確認して結果を返す
}

static int steering_ready(uint32_t now)
{
    for (uint8_t i = 0; i < 3; ++i) {
        if (!home_done[i] || steering[i].fault || !steering[i].valid ||
            now - steering[i].rx_ms >= STEER_FEEDBACK_TIMEOUT_MS) {
            return 0;
        }
    }
    return 1;
}

/* 20msごとに指令を作る。送信FIFOへの投入は空きがある分だけ進める。 */
static void swerve_task(void)
{
    static uint8_t tx_step = 9;
    static uint8_t blocked;
    static int steer_ok, run;
    static uint32_t cmd_stamp, last_cycle;
    uint32_t cycles = DWT->CYCCNT;
    uint32_t now = HAL_GetTick();
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint8_t due = motor_update_due;
    motor_update_due = 0;
    float vx = rx_vx, vy = rx_vy, omega = rx_omega;
    uint32_t stamp = velocity_rx_tick, count = velocity_rx_count;
    __set_PRIMASK(mask);

    if (due) {
        if (tx_step < 9U) {
            swerve_tx_delay++;
        }
        swerve_period_ms = now - last_cycle;
        last_cycle = now;
        tx_step = 0;
        blocked = 0;
        cmd_stamp = stamp;
        steer_ok = steering_ready(now);
        run = 0;
        if (steer_ok && count != 0U && now - stamp < SWERVE_CMD_TIMEOUT_MS) {
            float previous_pos_ref[3];
            for (uint8_t i = 0; i < 3; ++i) {
                previous_pos_ref[i] = robostride_pos_ref[i];
            }

            /* 既存の上位入力→車体軸の対応。実機と送信側で確認する。 */
            compute_wheel_targets(vy, -vx, omega);
            run = 1;
            for (uint8_t i = 0; i < 3; ++i) {
                float error = (robostride_pos_ref[i] - steering[i].angle) /
                    (modules[i].steer_sign * modules[i].steer_ratio);
                if (!isfinite(robostride_pos_ref[i]) ||
                    !isfinite(wheel_speed_target[i]) ||
                    fabsf(wheel_speed_target[i]) > MAX_WHEEL_RPM ||
                    !isfinite(error)) {
                    steer_ok = 0;
                }
                if (fabsf(error) > SWERVE_ALIGN_RAD) {
                    run = 0;
                }
            }
            if (!steer_ok) {
                for (uint8_t i = 0; i < 3; ++i) {
                    robostride_pos_ref[i] = previous_pos_ref[i];
                    wheel_speed_target[i] = 0.0f;
                }
                run = 0;
            }
        } else {
            steer_ok = 0;
        }
    }

    uint32_t elapsed_cycles = DWT->CYCCNT - cycles;
    uint32_t elapsed_us = elapsed_cycles / (SystemCoreClock / 1000000U);
    if (elapsed_us > swerve_max_us) {
        swerve_max_us = elapsed_us;
    }
    if (tx_step >= 9U) {
        return;
    }

    if ((hfdcan3.Instance->PSR & FDCAN_PSR_BO) ||
        HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) == 0U) {
        if (!blocked) {
            swerve_tx_delay++;
            blocked = 1;
        }
        run = 0;
        return;
    }

    blocked = 0;
    HAL_StatusTypeDef status = HAL_OK;
    if (tx_step < 3U) {
        float rpm = 0.0f;
        if (run && swerve_drive_enable && steering_ready(now) &&
            now - cmd_stamp < SWERVE_CMD_TIMEOUT_MS) {
            rpm = wheel_speed_target[tx_step];
        }
        status = send_motor_cmd(tx_step, rpm, goal_pid_mode, goal_control_motor_mode);
    } else if (tx_step < 6U) {
        uint8_t i = tx_step - 3U;
        if (steer_ok && steering_ready(now) && now - cmd_stamp < SWERVE_CMD_TIMEOUT_MS) {
            status = robostride_write_float(modules[i].steer_id, RS_PARAM_LOC_REF,
                                           robostride_pos_ref[i]);
        }
    } else {
        uint8_t i = tx_step - 6U;
        SteeringFeedback *feedback = &steering[i];
        if (feedback->pending && now - feedback->request_ms >= STEER_FEEDBACK_TIMEOUT_MS) {
            feedback->pending = 0;
            feedback->valid = 0;
        }
        if (!feedback->pending) {
            status = request_parameter(modules[i].steer_id, RS_PARAM_MECH_POS);
            if (status == HAL_OK) {
                feedback->request_ms = now;
                feedback->pending = 1;
            }
        }
    }
    if (status != HAL_OK) {
        swerve_tx_fail++;
        run = 0;
    }
    tx_step++;
}

/* 独立ステアリングの座標変換関数 */
static void compute_wheel_targets(float vx, float vy, float omega)
{
    float largest = 0.0f;                     // 最も大きい目標回転数の絶対値を保存
    int feasible = 1;                         // 3輪すべてで必要な向きを作れるかを記録

    for (uint8_t i = 0; i < 3; ++i) {
        float ux = vx - omega * modules[i].y;
        float uy = vy + omega * modules[i].x;
        float speed = hypotf(ux, uy);         // hypotf(ux, uy)は(ux^2 + uy^2)のルート

        wheel_speed_target[i] = 0.0f;

        /* 停止時は最後の操舵目標を保持 */
        if (speed < 0.001f) {
            continue;
        }

        /* 原点基準の現在の車輪角度 */
        float current = (steering[i].angle - home_position[i]) / (modules[i].steer_sign * modules[i].steer_ratio);  // 現在の車輪の向きを、原点基準の角度(rad)に
        float heading = atan2f(uy, ux);        // y座標とx座標から角度を求める関数 +-180°
        float best_angle = 0.0f;               // 最終的に選ぶ車輪の向きを保存
        float best_distance = INFINITY;        // 現在位置からの最短移動量を保存
        float best_sign = 1.0f;                // 駆動方向の補正を保存
        int found = 0;                         // 範囲内の操舵角を見つけたか記録

        /* 同じ走行方向を実現できる角度から、制限内で現在位置に最も近いものを選ぶ */
        for (int k = -1; k <= 1; ++k) {
            float candidate = heading + (float)k * (float)M_PI;  // 元の方向に、180°ずつ足し引きした操舵角を作成

            if (fabsf(candidate) > STEER_LIMIT_RAD) {            // 候補の角度が、設定した上限を超えていないか確認
                continue;
            }

            float distance = fabsf(candidate - current);         // 現在の車輪角度から、その候補まで何rad動く必要があるかを計算

            if (distance < best_distance) {                      // 今までの候補より移動量が小さい場合に、その候補を採用
                best_distance = distance;
                best_angle = candidate;

                /* 180度ずらす場合は駆動方向を反転 */
                best_sign = (k % 2 == 0) ? 1.0f : -1.0f;
                found = 1;
            }
        }

        /* 範囲内の角度がなければ走行させない */
        if (!found) {
            feasible = 0;
            continue;
        }

        /* 現在位置への加算ではなく、原点基準で指定 */
        robostride_pos_ref[i] = home_position[i] + modules[i].steer_sign * modules[i].steer_ratio * best_angle;
        wheel_speed_target[i] = speed * best_sign * 60.0f / (2.0f * (float)M_PI * modules[i].wheel_radius) * modules[i].drive_ratio * modules[i].drive_sign;
        largest = fmaxf(largest, fabsf(wheel_speed_target[i]));
    }

    /* 1軸でも実現できなければ3輪とも駆動停止 */
    if (!feasible) {
        for (uint8_t i = 0; i < 3; ++i) {
            wheel_speed_target[i] = 0.0f;
        }
        return;
    }

    /* 回転数上限を維持 */
    if (largest > MAX_WHEEL_RPM) {
        float scale = MAX_WHEEL_RPM / largest;

        for (uint8_t i = 0; i < 3; ++i) {
            float rpm = wheel_speed_target[i] * scale;
            if (isfinite(rpm)) {
                wheel_speed_target[i] = fminf(MAX_WHEEL_RPM, fmaxf(-MAX_WHEEL_RPM, rpm));
            }
        }
    }
}

/* RoboStrideへのCAN送信要求が正常に受け付けられたか確認し、次の指令まで少し待機するための関数 */
static int home_send_ok(HAL_StatusTypeDef status)
{
    if (status != HAL_OK) return 0;

    HAL_Delay(10);
    return can3_wait_idle();
}

static int read_home_parameter(uint8_t id, uint16_t index, void *value,
                               uint8_t size, uint32_t timeout_ms);

static int home_seek(uint8_t i, float start_angle, uint32_t started,
                     GPIO_PinState end_state, float speed, float wheel_limit)
{
    const SwerveModule *module = &modules[i];
    uint8_t id = module->steer_id;
    if (!home_send_ok(robostride_write_float(id, RS_PARAM_SPD_REF, speed))) {
        return 0;
    }

    while (HAL_GetTick() - started < HOME_TIMEOUT_MS) {
        float angle;
        if (!read_home_parameter(id, RS_PARAM_MECH_POS, &angle, sizeof(angle), 40U) ||
            !isfinite(angle) || steering[i].fault ||
            fabsf(angle - start_angle) >= wheel_limit * module->steer_ratio) {
            break;
        }
        if (HAL_GPIO_ReadPin(GPIOC, module->home_pin) == end_state) {
            return home_send_ok(robostride_write_float(id, RS_PARAM_SPD_REF, 0.0f));
        }
    }
    (void)robostride_stop(id);
    return 0;
}

/* 独立ステアリングの操舵モーターを原点復帰 */
static int home_one_motor(uint8_t i)
{
    uint8_t id = modules[i].steer_id;
    uint8_t mode;
    uint8_t stop_data[8] = {0};
    stop_data[0] = 1;                    // 初期化開始時に異常解除を要求
    home_done[i] = 0;
    steering[i].fault = 0;

    /* 停止 → 速度モード → 電流制限 → 速度0 */
    if (!home_send_ok(robostride_send(RS_TYPE_STOP, ROBOSTRIDE_HOST_ID, id, stop_data)) ||
        !home_send_ok(robostride_write_u8(id, RS_PARAM_RUN_MODE, 2)) ||
        !home_send_ok(robostride_write_float(id, RS_PARAM_LIMIT_CUR, 3.0f)) ||
        !home_send_ok(robostride_write_float(id, RS_PARAM_SPD_REF, 0.0f))) {
        return 0;
    }

    stop_data[0] = 0;

    /* 速度モードに切り替わったことを確認 */
    if (!home_read_param(id, RS_PARAM_RUN_MODE, &mode, 1) || mode != 2) {
        return 0;
    }

    if (!home_send_ok(robostride_enable(id))) return 0;

    printf("HOME START id=%u\r\n", (unsigned int)id);
    fflush(stdout);

    float start_angle;
    if (!home_read_param(id, RS_PARAM_MECH_POS, &start_angle, sizeof(start_angle)) ||
        !isfinite(start_angle) || steering[i].fault) {
        return 0;
    }

    uint32_t started = HAL_GetTick();
    /* LOWで起動した場合は逆方向へ少し動かし、HIGHへ戻ることを確かめる。 */
    if (HAL_GPIO_ReadPin(GPIOC, modules[i].home_pin) == GPIO_PIN_RESET &&
        !home_seek(i, start_angle, started, GPIO_PIN_SET,
                   -0.5f * modules[i].home_speed, modules[i].home_release)) {
        return 0;
    }

    if (!home_seek(i, start_angle, started, GPIO_PIN_RESET,
                   modules[i].home_speed, modules[i].home_travel)) {
        return 0;
    }

    /* 0を検出したら速度指令を0にする */
    if (!home_send_ok(robostride_write_float(id, RS_PARAM_SPD_REF, 0.0f))) {
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

        if (HAL_GetTick() - settling >= 1000U || !home_read_param(id, 0x701Bu, &speed, 4) || !isfinite(speed)) {
            return 0;
        }

        still_count = fabsf(speed) < 0.03f ? (uint8_t)(still_count + 1) : 0;

        HAL_Delay(20);
    }


    if (!home_send_ok(robostride_send(RS_TYPE_STOP, ROBOSTRIDE_HOST_ID, id, stop_data))) {
        return 0;
    }

    /* 現在の多回転角度を読み取って保存 */
    if (!home_read_param(id, 0x7019u, &home_position[i], 4) ||
        !isfinite(home_position[i])) {
        return 0;
    }

    float position_error = fabsf(home_position[i] - home_detect_position[i]);

    if (position_error > HOME_POSITION_TOLERANCE_RAD) {
      printf("HOME FAILED: stop movement too large id=%u\r\n", (unsigned int)id);
      fflush(stdout);
      return 0;
    }

    /* 位置モードへ。目標は0ではなく、いま保存した角度 */
    if (!home_send_ok(robostride_write_u8(id, RS_PARAM_RUN_MODE, 5)) ||
        !home_send_ok(robostride_write_float(id, RS_PARAM_LIMIT_SPD, 15.0f)) ||
        !home_send_ok(robostride_write_float(id, RS_PARAM_LOC_REF, home_position[i]))) {
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

    if (steering[i].fault) {
        return 0;
    }
    home_done[i] = 1;

    return 1;
}

static HAL_StatusTypeDef request_parameter(uint8_t id, uint16_t index)
{
    uint8_t request[8] = {0};
    put_u16_le(request, index);
    return robostride_send(RS_TYPE_READ_PARAM, ROBOSTRIDE_HOST_ID, id, request);
}

/* 原点復帰と通常走行で共用するFIFO0の受信処理。IDスキャンは別の起動モード。 */
static void can3_receive(void)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[64];

    for (uint8_t n = 0; n < 8U && HAL_FDCAN_GetRxFifoFillLevel(&hfdcan3, FDCAN_RX_FIFO0); ++n) {
        if (HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &rx, data) != HAL_OK) {
            return;
        }
        if (rx.IdType != FDCAN_EXTENDED_ID || rx.FDFormat != FDCAN_CLASSIC_CAN ||
            rx.RxFrameType != FDCAN_DATA_FRAME || rx.DataLength != FDCAN_DLC_BYTES_8 ||
            (rx.Identifier & 0xFFU) != ROBOSTRIDE_HOST_ID) {
            continue;
        }

        uint8_t type = (rx.Identifier >> 24) & 0x1FU;
        uint8_t id = (rx.Identifier >> 8) & 0xFFU;
        for (uint8_t i = 0; i < 3U; ++i) {
            if (id != modules[i].steer_id) {
                continue;
            }

            SteeringFeedback *feedback = &steering[i];
            if (type == 2U || type == 24U || type == 21U) {
                uint32_t fault = type == 21U ?
                    (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                    ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24) :
                    (rx.Identifier >> 16) & 0x3FU;
                if (fault || (home_done[i] && type != 21U &&
                              ((rx.Identifier >> 22) & 3U) != 2U)) {
                    feedback->fault = 1;
                    feedback->valid = 0;
                }
                break;
            }
            if (type != RS_TYPE_READ_PARAM) {
                break;
            }

            uint16_t index = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
            uint8_t ok = ((rx.Identifier >> 16) & 0xFFU) == 0U;
            if (home_reply.waiting && !home_reply.received &&
                home_reply.id == id && home_reply.index == index) {
                memcpy(home_reply.data, &data[4], sizeof(home_reply.data));
                home_reply.ok = ok;
                home_reply.received = 1;
            }
            if (index == RS_PARAM_MECH_POS && feedback->pending) {
                float angle;
                uint32_t now = HAL_GetTick();
                memcpy(&angle, &data[4], sizeof(angle));
                feedback->valid = ok && !feedback->fault && isfinite(angle) &&
                    now - feedback->request_ms < STEER_FEEDBACK_TIMEOUT_MS;
                if (feedback->valid) {
                    feedback->angle = angle;
                    feedback->rx_ms = now;
                }
                feedback->pending = 0;
            }
            break;
        }
    }
}

/* 同期待機は原点復帰だけ。受信の解析は共通処理に委ねる。 */
static int read_home_parameter(uint8_t id, uint16_t index, void *value,
                               uint8_t size, uint32_t timeout_ms)
{
    can3_receive();
    home_reply.id = id;
    home_reply.index = index;
    home_reply.waiting = 1;
    home_reply.received = 0;
    if (request_parameter(id, index) == HAL_OK) {
        uint32_t started = HAL_GetTick();
        while (HAL_GetTick() - started < timeout_ms &&
               !(hfdcan3.Instance->PSR & FDCAN_PSR_BO)) {
            can3_receive();
            if (home_reply.received) {
                break;
            }
            HAL_Delay(1);
        }
    }
    home_reply.waiting = 0;
    if (!home_reply.received || !home_reply.ok) {
        return 0;
    }
    memcpy(value, home_reply.data, size);
    return 1;
}

static int home_read_param(uint8_t id, uint16_t index, void *value, uint8_t size)
{
    return read_home_parameter(id, index, value, size, 200U);
}

static void CAN3_DiagCapture(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance != FDCAN3) {
        return;
    }

    /* Reading PSR acknowledges the last error code. Preserve this first read. */
    uint32_t psr = hfdcan->Instance->PSR;  // 通信状態や最後のエラーを保持
    uint32_t ecr = hfdcan->Instance->ECR;  // 送受信エラーカウンタを保持
    uint32_t lec = psr & FDCAN_PSR_LEC;

    /* 最初に検出したBus-Offの情報を残しておく */
    if ((psr & FDCAN_PSR_BO) != 0U && can3_diag_busoff_seen == 0U) {
        can3_diag_busoff_psr = psr;
        can3_diag_busoff_ecr = ecr;
        can3_diag_busoff_seen = 1U;
    }

    /* エラーがなければ終了 */
    if (lec == 0U || lec == 7U) {
        return;
    }

    /* エラー情報を配列に保存 */
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
    return HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0U);
}

/* エラー通知を有効化する関数 */
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

// /* 3輪オムニの逆運動学: vx,vy[m/s], omega[rad/s] → 各ホイールrpm */

//     for (int i = 0; i < 3; i++)
//     {
//         // 接線方向速度成分 + 回転による寄与
//         float v_wheel_mps = -vx * sinf(wheel_angle[i]) + vy * cosf(wheel_angle[i])
//                              + ROBOT_RADIUS_M * omega;

//         // m/s → rpm変換: rpm = v / (2*pi*r) * 60
//         float rpm = v_wheel_mps / (2.0f * (float)M_PI * modules[i].wheel_radius) * 60.0f;

//         if (rpm > MAX_WHEEL_RPM)  rpm = MAX_WHEEL_RPM;
//         if (rpm < -MAX_WHEEL_RPM) rpm = -MAX_WHEEL_RPM;

//         wheel_speed_target[i] = rpm;
//     }
// }

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
    if (RxHeader.RxFrameType != FDCAN_DATA_FRAME) return;
    if (RxHeader.FDFormat != FDCAN_FD_CAN || RxHeader.BitRateSwitch != FDCAN_BRS_ON || RxHeader.DataLength != FDCAN_DLC_BYTES_12)
    {
      return;
    }
    can_velocity_cmd_t cmd;
    
    cmd.vx    = be_bytes_to_float(&RxData[0]);
    cmd.vy    = be_bytes_to_float(&RxData[4]);
    cmd.omega = be_bytes_to_float(&RxData[8]);

    if (!isfinite(cmd.vx) || !isfinite(cmd.vy) || !isfinite(cmd.omega) ||
        hypotf(cmd.vx, cmd.vy) > MAX_BODY_SPEED_M_S ||
        fabsf(cmd.omega) > MAX_BODY_OMEGA_RAD_S) {
        return;
    }

    rx_vx    = cmd.vx;      
    rx_vy    = cmd.vy;
    rx_omega = cmd.omega;

    velocity_rx_tick = HAL_GetTick(); 

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
    if (htim->Instance != TIM6) {
        return;
    }
    static uint8_t divider;
    if (++divider >= SWERVE_PERIOD_MS) {
        divider = 0;
        if (motor_update_due) {
            swerve_missed_ticks++;
        }
        motor_update_due = 1;
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
  //   uint8_t id = modules[i].steer_id;

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

    robostride_send(RS_TYPE_STOP, ROBOSTRIDE_HOST_ID, modules[i].steer_id, stop_data);
    HAL_Delay(10);
  }

  /* ID1 → ID2 → ID127の順に探索 */
  for (uint8_t i = 0; i < 3; i++) {
 
    /* 一時対応：ID2・ID127はセンサー0なら仮OK */
    // if ((modules[i].steer_id == 2 ||
    //   modules[i].steer_id == 127) &&
    //   HAL_GPIO_ReadPin(GPIOC, modules[i].home_pin)
    //     == GPIO_PIN_RESET) {

    //   home_done[i] = 1;
    //   control_mode[i] = -1;
    //   continue;
    // }

    if (!home_one_motor(i)) {

        /* 失敗したら3台とも停止要求 */
        for (uint8_t j = 0; j < 3; j++) {
            uint8_t stop_data[8] = {0};

            robostride_send(RS_TYPE_STOP, ROBOSTRIDE_HOST_ID, modules[j].steer_id, stop_data);
            HAL_Delay(10);
        }

        printf("HOME FAILED id=%u\r\n",
               (unsigned int)modules[i].steer_id);
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
            robostride_send(RS_TYPE_STOP, ROBOSTRIDE_HOST_ID, modules[j].steer_id, stop_data);
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
  motor_update_due = 0;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
    Error_Handler();
}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    can3_receive();
    swerve_task();

    /* シリアル送信を待たず、TDRに空きがあるときだけ進める。 */
    static char log_line[256];
    static size_t log_used, log_sent;
    static uint32_t rx_print_tick, previous_rx_count;
    if (log_sent < log_used && __HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE)) {
        huart2.Instance->TDR = (uint8_t)log_line[log_sent++];
    }
    uint32_t now = HAL_GetTick();
    if (log_sent == log_used && now - rx_print_tick >= 500U) {
        rx_print_tick = now;
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        uint32_t count = velocity_rx_count;
        uint32_t stamp = velocity_rx_tick;
        float vx = rx_vx, vy = rx_vy, omega = rx_omega;
        __set_PRIMASK(mask);

        int length = snprintf(log_line, sizeof(log_line),
            "CAN1 RX=%lu +%lu age_ms=%lu vx_mm_s=%ld vy_mm_s=%ld omega_mrad_s=%ld\r\n"
            "SWERVE period=%lu ms calc_max=%lu us missed=%lu tx_delay=%lu tx_fail=%lu fault=%u%u%u\r\n",
            (unsigned long)count, (unsigned long)(count - previous_rx_count),
            (unsigned long)(count ? HAL_GetTick() - stamp : 0U),
            (long)(vx * 1000.0f), (long)(vy * 1000.0f), (long)(omega * 1000.0f),
            (unsigned long)swerve_period_ms, (unsigned long)swerve_max_us,
            (unsigned long)swerve_missed_ticks, (unsigned long)swerve_tx_delay,
            (unsigned long)swerve_tx_fail, (unsigned int)steering[0].fault,
            (unsigned int)steering[1].fault, (unsigned int)steering[2].fault);
        log_sent = 0;
        log_used = length > 0 && (size_t)length < sizeof(log_line) ? (size_t)length : 0;
        previous_rx_count = count;
    }

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
    // static uint32_t photo_print_tick = 0;

    // if (HAL_GetTick() - photo_print_tick >= 100U)
    // {
    //   photo_print_tick = HAL_GetTick();

    //   unsigned int photo0 =
    //     (HAL_GPIO_ReadPin(motor_0_GPIO_Port, motor_0_Pin)
    //      == GPIO_PIN_SET);

    //   unsigned int photo1 =
    //     (HAL_GPIO_ReadPin(motor_1_GPIO_Port, motor_1_Pin)
    //      == GPIO_PIN_SET);

    //   unsigned int photo2 =
    //     (HAL_GPIO_ReadPin(motor_2_GPIO_Port, motor_2_Pin)
    //      == GPIO_PIN_SET);

    //   printf("PHOTO motor_0=%u motor_1=%u motor_2=%u\r\n",
    //        photo0, photo1, photo2);

    //   fflush(stdout);
    // }
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
