#include "swerve.h"

#include "drive_can.h"
#include "robostride_can.h"
#include "swerve_homing.h"

#include <math.h>
#include <stdio.h>

extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart2;

#define RS_ID_SCAN_ONLY 0
#define SWERVE_PERIOD_MS 20U
#define SWERVE_CMD_TIMEOUT_MS 300U
#define SWERVE_ALIGN_RAD 0.174533f  /* 車輪側で10度 */

enum {
    DRIVE_TX_START = 0,
    STEER_TX_START = SWERVE_WHEEL_COUNT,
    ANGLE_REQUEST_START = 2U * SWERVE_WHEEL_COUNT,
    TX_STEP_COUNT = 3U * SWERVE_WHEEL_COUNT
};

/* FDCAN3の駆動ID 0x301 / 0x302 / 0x303 と同じ、右前 / 左前 / 後ろの順。 */
const SwerveModule swerve_modules[SWERVE_WHEEL_COUNT] = {
    { .steer_id = 1, .home_pin = GPIO_PIN_3, .steer_ratio = 3.0f, .steer_sign = 1.0f,
      .drive_sign = -1.0f, .home_speed_rad_s = 1.5f,
      .home_travel_rad = SWERVE_STEER_LIMIT_RAD, .home_release_rad = SWERVE_ALIGN_RAD },
    { .steer_id = 2, .home_pin = GPIO_PIN_0, .steer_ratio = 3.0f, .steer_sign = 1.0f,
      .drive_sign = -1.0f, .home_speed_rad_s = 1.5f,
      .home_travel_rad = SWERVE_STEER_LIMIT_RAD, .home_release_rad = SWERVE_ALIGN_RAD },
    { .steer_id = 127, .home_pin = GPIO_PIN_2, .steer_ratio = 3.0f, .steer_sign = 1.0f,
      .drive_sign = -1.0f, .home_speed_rad_s = 1.5f,
      .home_travel_rad = SWERVE_STEER_LIMIT_RAD, .home_release_rad = SWERVE_ALIGN_RAD }
};

static float home_position[SWERVE_WHEEL_COUNT];
static float steer_target_rad[SWERVE_WHEEL_COUNT];  /* RobStride出力軸の絶対角 */
static float motor_target_rpm[SWERVE_WHEEL_COUNT];  /* N5065軸、実機の符号適用済み */

/* デバッガーで設定できる平ドラ側のモード。起動時の最初の指令で確定する。 */
volatile uint8_t swerve_drive_enable = 1U;
volatile uint8_t goal_pid_mode = 0U;            /* 0: 位置型PID、1: 速度型PID */
volatile uint8_t goal_control_motor_mode = 0U;  /* 0: 電圧、1: 電流PI */

static volatile bool cycle_due;
static volatile uint32_t missed_ticks;
static uint32_t period_ms, calc_max_us, tx_delay, tx_fail;
static struct {
    uint32_t command_rx_ms;
    uint8_t step;
    bool queue_blocked;
    bool steer_allowed;
    bool drive_allowed;
} tx_cycle = { .step = TX_STEP_COUNT };

static bool command_fresh(uint32_t now, uint32_t rx_ms)
{
    return now - rx_ms < SWERVE_CMD_TIMEOUT_MS;
}

static bool steering_ready(uint32_t now)
{
    for (uint8_t i = 0; i < SWERVE_WHEEL_COUNT; ++i) {
        const SteeringFeedback *feedback = robostride_feedback(i);
        if (feedback->fault || !feedback->valid ||
            now - feedback->rx_ms >= STEER_FEEDBACK_TIMEOUT_MS) {
            return false;
        }
    }
    return true;
}

static bool select_wheel_target(uint8_t i, const WheelCommand *command,
                                float *target_angle, float *target_rpm)
{
    const SwerveModule *module = &swerve_modules[i];
    const float steer_scale = module->steer_sign * module->steer_ratio;
    const float current_angle = (robostride_feedback(i)->angle_rad - home_position[i]) / steer_scale;
    float selected_angle = 0.0f;
    float nearest_distance = INFINITY;

    *target_rpm = 0.0f;
    if (command->motor_rpm == 0.0f) {
        *target_angle = steer_target_rad[i];  /* 停止している輪は最後の向きを保持する。 */
        return true;
    }

    /* 180度違う向きでも、駆動の符号を反転すれば同じ方向へ走れる。 */
    for (int turn = -1; turn <= 1; ++turn) {
        const float candidate = command->steer_rad + (float)turn * (float)M_PI;
        const float distance = fabsf(candidate - current_angle);
        if (fabsf(candidate) <= SWERVE_STEER_LIMIT_RAD && distance < nearest_distance) {
            selected_angle = candidate;
            nearest_distance = distance;
            *target_rpm = command->motor_rpm * module->drive_sign *
                          (turn == 0 ? 1.0f : -1.0f);
        }
    }
    *target_angle = home_position[i] + steer_scale * selected_angle;
    return isfinite(nearest_distance) && isfinite(*target_angle);
}

static void begin_cycle(void)
{
    static uint32_t previous_cycle_ms;
    const DriveCommand command = drive_can_latest_command();
    const uint32_t now = HAL_GetTick();
    if (tx_cycle.step < TX_STEP_COUNT) {
        tx_delay++;
    }
    period_ms = now - previous_cycle_ms;
    previous_cycle_ms = now;
    tx_cycle.step = DRIVE_TX_START;
    tx_cycle.queue_blocked = false;
    tx_cycle.command_rx_ms = command.rx_ms;
    tx_cycle.steer_allowed = false;
    tx_cycle.drive_allowed = false;

    if (command.count == 0U || !command_fresh(now, command.rx_ms) ||
        !steering_ready(now)) {
        return;
    }

    /* 全輪の検証が通ってから目標をまとめて更新する。 */
    float angles[SWERVE_WHEEL_COUNT];
    float rpm[SWERVE_WHEEL_COUNT];
    bool aligned = true;
    for (uint8_t i = 0; i < SWERVE_WHEEL_COUNT; ++i) {
        if (!select_wheel_target(i, &command.wheels[i], &angles[i], &rpm[i])) {
            return;
        }
        const float error = fabsf(angles[i] - robostride_feedback(i)->angle_rad) /
                            swerve_modules[i].steer_ratio;
        if (!isfinite(error)) {
            return;
        }
        if (error > SWERVE_ALIGN_RAD) {
            aligned = false;
        }
    }
    for (uint8_t i = 0; i < SWERVE_WHEEL_COUNT; ++i) {
        steer_target_rad[i] = angles[i];
        motor_target_rpm[i] = rpm[i];
    }
    tx_cycle.steer_allowed = true;
    tx_cycle.drive_allowed = aligned;
}

/* 1回のメインループで最大1件。返信や送信の完了をここでは待たない。 */
static void send_next_frame(uint32_t now)
{
    const uint8_t step = tx_cycle.step;
    if (step >= TX_STEP_COUNT) {
        return;
    }

    if (!robostride_tx_ready()) {
        if (!tx_cycle.queue_blocked) {
            tx_delay++;
            tx_cycle.queue_blocked = true;
        }
        tx_cycle.drive_allowed = false;
        return;
    }

    tx_cycle.queue_blocked = false;
    const bool ready = command_fresh(now, tx_cycle.command_rx_ms) && steering_ready(now);
    HAL_StatusTypeDef status = HAL_OK;
    if (step < STEER_TX_START) {
        /* 上位指令や角度返信を失ったときも、下流へ0 rpmを送る。 */
        float rpm = 0.0f;
        if (tx_cycle.drive_allowed && swerve_drive_enable && ready) {
            rpm = motor_target_rpm[step];
        }
        status = drive_can_send_rpm(step, rpm, goal_pid_mode, goal_control_motor_mode);
    } else if (step < ANGLE_REQUEST_START) {
        const uint8_t i = step - STEER_TX_START;
        if (tx_cycle.steer_allowed && ready) {
            status = robostride_write_float(swerve_modules[i].steer_id,
                                          RS_PARAM_LOC_REF, steer_target_rad[i]);
        }
    } else {
        status = robostride_request_angle(step - ANGLE_REQUEST_START, now);
    }

    if (status != HAL_OK) {
        tx_fail++;
        tx_cycle.drive_allowed = false;
    }
    tx_cycle.step++;
}

static bool take_cycle_request(void)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const bool due = cycle_due;
    cycle_due = false;
    __set_PRIMASK(mask);
    return due;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) {
        return;
    }
    static uint8_t divider;
    if (++divider >= SWERVE_PERIOD_MS) {
        divider = 0;
        if (cycle_due) {
            missed_ticks++;
        }
        cycle_due = true;
    }
}

static void report_status(void)
{
    static char line[256];
    static size_t used, sent;
    static uint32_t previous_tick, previous_count;

    /* UARTが空いたときに1バイトずつ送り、CANの周期処理を待たせない。 */
    if (sent < used && __HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE)) {
        huart2.Instance->TDR = (uint8_t)line[sent++];
    }

    const uint32_t now = HAL_GetTick();
    if (sent != used || now - previous_tick < 500U) {
        return;
    }

    const DriveCommand command = drive_can_latest_command();
    unsigned int faults = 0;
    for (uint8_t i = 0; i < SWERVE_WHEEL_COUNT; ++i) {
        if (robostride_feedback(i)->fault) {
            faults |= 1U << i;
        }
    }
    const int length = snprintf(line, sizeof(line),
        "CAN1 ID=0x110 RX=%lu +%lu age_ms=%lu rpm=[%ld,%ld,%ld]\r\n"
        "SWERVE period=%lu ms calc_max=%lu us missed=%lu tx_delay=%lu tx_fail=%lu faults=0x%X\r\n",
        (unsigned long)command.count, (unsigned long)(command.count - previous_count),
        (unsigned long)(command.count ? now - command.rx_ms : 0U),
        (long)command.wheels[0].motor_rpm, (long)command.wheels[1].motor_rpm,
        (long)command.wheels[2].motor_rpm, (unsigned long)period_ms,
        (unsigned long)calc_max_us, (unsigned long)missed_ticks,
        (unsigned long)tx_delay, (unsigned long)tx_fail, faults);
    sent = 0;
    used = length > 0 && (size_t)length < sizeof(line) ? (size_t)length : 0;
    previous_tick = now;
    previous_count = command.count;
}

void swerve_start(void)
{
    robostride_can_start();  // FDCAN3
    drive_can_start();       // 0x110 FDCAN1

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

    /* もしcanidのscanモードなら */
    if (RS_ID_SCAN_ONLY) {
        HAL_Delay(200);
        
        robostride_scan_ids();
        while (1) {
            HAL_Delay(1000);
            robostride_print_startup_errors();
        }
    }

    if (!swerve_home_all(home_position)) {
        while (1) {
            HAL_Delay(100);
        }
    }
    for (uint8_t i = 0; i < SWERVE_WHEEL_COUNT; ++i) {
        steer_target_rad[i] = home_position[i];
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        Error_Handler();
    }
}

void swerve_poll(void)
{
    robostride_receive();
    if (take_cycle_request()) {
        const uint32_t started = DWT->CYCCNT;
        begin_cycle();
        const uint32_t elapsed_us = (DWT->CYCCNT - started) / (SystemCoreClock / 1000000U);
        if (elapsed_us > calc_max_us) {
            calc_max_us = elapsed_us;
        }
    }
    send_next_frame(HAL_GetTick());
    report_status();
}

/* printfによる起動ログは、周期処理を開始する前に使用する。 */
int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}
