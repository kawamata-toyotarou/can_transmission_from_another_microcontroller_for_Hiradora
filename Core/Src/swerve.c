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
#define SWERVE_ALIGN_RAD 0.174533f        /* 初期化時車輪側で許容誤差10度 */
#define SWERVE_DRIVE_ALIGN_RAD 0.349066f  /* 走行時車輪側で許容誤差20度 */
#define SWERVE_STEER_RATIO 3.0f
/* 原点探索の上限はRobStride出力軸で370°。設定欄には車輪側の角度で渡す。減速比3 */
#define HOME_TRAVEL_ROBOSTRIDE_RAD (370.0f * (float)M_PI / 180.0f) * 3.0
#define HOME_TRAVEL_WHEEL_RAD (HOME_TRAVEL_ROBOSTRIDE_RAD / SWERVE_STEER_RATIO)

enum {
    DRIVE_TX_START = 0,
    STEER_TX_START = SWERVE_WHEEL_COUNT,
    ANGLE_REQUEST_START = 2U * SWERVE_WHEEL_COUNT,
    TX_STEP_COUNT = 3U * SWERVE_WHEEL_COUNT
};

/* FDCAN3の駆動ID 0x301 / 0x302 / 0x303 と同じ、右前 / 左前 / 後ろの順。 */
const SwerveModule swerve_modules[SWERVE_WHEEL_COUNT] = {
    { .x_m = 0.155f, .y_m = -0.2684679f, .wheel_radius_m = 0.0321f, .drive_ratio = 1.0f,
      .steer_id = 1, .home_pin = GPIO_PIN_3, .steer_ratio = SWERVE_STEER_RATIO, .steer_sign = 1.0f,
      .drive_sign = -1.0f, .home_speed_rad_s = 1.5f,
      .home_travel_rad = HOME_TRAVEL_WHEEL_RAD, .home_release_rad = SWERVE_ALIGN_RAD },
    { .x_m = 0.155f, .y_m = 0.2684679f, .wheel_radius_m = 0.0321f, .drive_ratio = 1.0f,
      .steer_id = 2, .home_pin = GPIO_PIN_0, .steer_ratio = SWERVE_STEER_RATIO, .steer_sign = 1.0f,
      .drive_sign = -1.0f, .home_speed_rad_s = 1.5f,
      .home_travel_rad = HOME_TRAVEL_WHEEL_RAD, .home_release_rad = SWERVE_ALIGN_RAD },
    { .x_m = -0.310f, .y_m = 0.0f, .wheel_radius_m = 0.0321f, .drive_ratio = 1.0f,
      .steer_id = 127, .home_pin = GPIO_PIN_2, .steer_ratio = SWERVE_STEER_RATIO, .steer_sign = 1.0f,
      .drive_sign = -1.0f, .home_speed_rad_s = 1.5f,
      .home_travel_rad = HOME_TRAVEL_WHEEL_RAD, .home_release_rad = SWERVE_ALIGN_RAD }
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
    uint32_t command_rx_ms;  // 今周期に使うPC指令の受信時刻 [ms]
    uint8_t step;            // 次に処理するCAN送信手順の番号
    bool queue_blocked;      // 今回のキュー詰まりを記録済みか
    bool steer_allowed;      // RobStrideへ操舵目標を送ってよいか
    bool drive_allowed;      // 平ドラへ駆動rpmを送ってよいか
} tx_cycle = { .step = TX_STEP_COUNT };  // 起動時は送信待ちの手順なし

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

static bool select_wheel_target(uint8_t i, float vx_m_s, float vy_m_s, float omega_rad_s, float *target_angle, float *target_rpm)
{
    const SwerveModule *module = &swerve_modules[i];
    const float steer_scale = module->steer_sign * module->steer_ratio;
    const float current_angle = (robostride_feedback(i)->angle_rad - home_position[i]) / steer_scale; // 実測角から home_position[i] を引いて、直進の原点からどれだけ回ったかを求める
    float selected_angle = 0.0f;
    float nearest_distance = INFINITY;

    /* rpmが0なら、前回の向きを保持する */
    /* 車体の並進に、車体中心まわりの旋回が作る各輪の速度を足す。 */
    const float ux = vx_m_s - omega_rad_s * module->y_m;
    const float uy = vy_m_s + omega_rad_s * module->x_m;
    const float speed_m_s = hypotf(ux, uy); //vx vyの速度を2つ合わせた大きさに

    *target_rpm = 0.0f;
    if (speed_m_s < 0.001f) {
        *target_angle = steer_target_rad[i];  /* 停止している輪は最後の向きを保持する。 */
        return true;
    }

    const float heading = atan2f(uy, ux);   // 前方から測った矢印の角度
    /* 1回転の移動距離で割って回転/秒にし、60を掛けてN5065軸rpmにする。 */
    const float motor_rpm = speed_m_s * 60.0f * module->drive_ratio / (2.0f * (float)M_PI * module->wheel_radius_m);

    /* 180度違う向きでも、駆動の符号を反転すれば同じ方向へ走れる。 */
    for (int turn = -1; turn <= 1; ++turn) {
        const float candidate = heading + (float)turn * (float)M_PI;
        const float distance = fabsf(candidate - current_angle);
        if (fabsf(candidate) <= SWERVE_STEER_LIMIT_RAD && distance < nearest_distance) {
            selected_angle = candidate;
            nearest_distance = distance;  // 角度差が一番小さいものを選択する
            *target_rpm = motor_rpm * module->drive_sign * (turn == 0 ? 1.0f : -1.0f); //角度を設定したらそのrpmを実行.
        }
    }
    *target_angle = home_position[i] + steer_scale * selected_angle;
    return isfinite(nearest_distance) && isfinite(*target_angle);
}

static void begin_cycle(void)
{
    static uint32_t previous_cycle_ms;                        // 前回この関数を実行した時刻
    const DriveCommand command = drive_can_latest_command();  //車体速度、受信時刻、受信回数
    const uint32_t now = HAL_GetTick();                       // 現在時刻
    /*tx_cycle.step は、今周期のCAN送信処理がどこまで進んだか。前周期の手順が最後まで進んでいなければ tx_delay を増やし、新しい周期を駆動指令の先頭から始める。続く行で、送信枠の詰まりを記録するフラグを戻し、今回使う指令の受信時刻を保存。*/
    if (tx_cycle.step < TX_STEP_COUNT) {
        tx_delay++;
    }
    /* 操舵と駆動を禁止にして、指令とセンサーを確認 */
    period_ms = now - previous_cycle_ms;    // 前回から何ms経ったか記録
    /* 今回の送信を最初から準備 */
    previous_cycle_ms = now;
    tx_cycle.step = DRIVE_TX_START;
    tx_cycle.queue_blocked = false;
    tx_cycle.command_rx_ms = command.rx_ms;
    tx_cycle.steer_allowed = false;
    tx_cycle.drive_allowed = false;

    if (command.count == 0U || !command_fresh(now, command.rx_ms) || !steering_ready(now)) {
        return;
    }

    /* 全輪の検証が通ってから目標をまとめて更新する。 */
    float angles[SWERVE_WHEEL_COUNT];
    float rpm[SWERVE_WHEEL_COUNT];
    float largest_rpm = 0.0f;
    bool aligned = true;

    /* 従来のとうロボの軸割り当てを、車体前方+X・左+Yにそろえる。 */
    const float vx_m_s = command.vy_m_s;
    const float vy_m_s = -command.vx_m_s;
    for (uint8_t i = 0; i < SWERVE_WHEEL_COUNT; ++i) {
        if (!select_wheel_target(i, vx_m_s, vy_m_s, command.omega_rad_s, &angles[i], &rpm[i]) || !isfinite(rpm[i])) {
            return;
        }
        const float error = fabsf(angles[i] - robostride_feedback(i)->angle_rad) /
                            swerve_modules[i].steer_ratio;
        if (!isfinite(error)) {
            return;
        }
        if (error > SWERVE_DRIVE_ALIGN_RAD) {
            aligned = false;
        }
        largest_rpm = fmaxf(largest_rpm, fabsf(rpm[i]));
    }

    /* 平ドラの上限を超えるときは、全輪に同じ倍率を掛けて速度比を保つ。 */
    const float scale = largest_rpm > DRIVE_MOTOR_MAX_RPM ? DRIVE_MOTOR_MAX_RPM / largest_rpm : 1.0f;
    for (uint8_t i = 0; i < SWERVE_WHEEL_COUNT; ++i) {
        steer_target_rad[i] = angles[i];
        motor_target_rpm[i] = copysignf(fminf(fabsf(rpm[i]) * scale, DRIVE_MOTOR_MAX_RPM), rpm[i]);
    }
    tx_cycle.steer_allowed = true;
    tx_cycle.drive_allowed = aligned;
}

/* 1回のメインループで最大1件。返信や送信の完了をここでは待たない。 */
static void send_next_frame(uint32_t now)
{
    /* 送る項目が残っているか、キューに空きがあるかを確認 */
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

    /* 今も指令と操舵角が使えるかを確認 */
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
    const bool due = cycle_due;  // 更新があった時だけtrueに
    cycle_due = false;
    __set_PRIMASK(mask);
    return due;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) {
        return;
    }
    /* TIM6の1 msごとの割り込みを20回数えて、メインループに次の制御周期が来たと知らせる処理 */
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
    static char line[320];
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
        "CAN1 ID=0x100 RX=%lu +%lu age_ms=%lu vx_mm_s=%ld vy_mm_s=%ld omega_mrad_s=%ld\r\n"
        "SWERVE period=%lu ms calc_max=%lu us missed=%lu tx_delay=%lu tx_fail=%lu faults=0x%X\r\n",
        (unsigned long)command.count, (unsigned long)(command.count - previous_count),
        (unsigned long)(command.count ? now - command.rx_ms : 0U),
        (long)(command.vx_m_s * 1000.0f), (long)(command.vy_m_s * 1000.0f),
        (long)(command.omega_rad_s * 1000.0f), (unsigned long)period_ms,
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
    drive_can_start();       // 0x100 FDCAN1

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
    /* 各輪の最初の操舵目標を原点 */
    for (uint8_t i = 0; i < SWERVE_WHEEL_COUNT; ++i) {
        steer_target_rad[i] = home_position[i];
    }

    /* CPUの処理時間を測るカウンタを始動 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {  // HAL_TIM_PeriodElapsedCallbackを呼ぶ
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
