#include "swerve_homing.h"
#include "robostride_can.h"

#include <math.h>
#include <stdio.h>

#define HOME_TIMEOUT_MS 45000U
/* 検出時から停止完了までに許すRobStride出力軸側の角度差。 */
#define HOME_POSITION_TOLERANCE_RAD 0.12f

static bool home_send_ok(HAL_StatusTypeDef status)
{
    /* robostrideへの送信がうまくいかなった時 */
    if (status != HAL_OK) return false;

    HAL_Delay(10);
    return robostride_wait_idle();
}

static bool home_read_param(uint8_t id, uint16_t index, void *value, uint8_t size)
{
    return robostride_read_parameter(id, index, value, size, 200U);
}

static bool home_seek(uint8_t i, float start_angle, uint32_t started, GPIO_PinState end_state, float speed, float wheel_limit)
{
    const SwerveModule *module = &swerve_modules[i]; // module は対象の車輪の設定
    uint8_t id = module->steer_id;                   // RobStrideのCAN IDを取得
    if (!home_send_ok(robostride_write_float(id, RS_PARAM_SPD_REF, speed))) {
        return false;
    }

    while (HAL_GetTick() - started < HOME_TIMEOUT_MS) { // 45秒未満の間、探索
        float angle;
        /* 現在角度が読めなかった時 か回転量が370度を超えた時 */
        if (!robostride_read_parameter(id, RS_PARAM_MECH_POS, &angle, sizeof(angle), 40U) || !isfinite(angle) || robostride_feedback(i)->fault || fabsf(angle - start_angle) >= wheel_limit * module->steer_ratio) {
            break;
        }
        /* センサーが反応したら、速度0を送る */
        if (HAL_GPIO_ReadPin(GPIOC, module->home_pin) == end_state) {
            return home_send_ok(robostride_write_float(id, RS_PARAM_SPD_REF, 0.0f));
        }
    }
    (void)robostride_stop(id, true); // 低電圧エラー解除 true が、停止と同時に故障フラグのクリアを要求 data[0] = 1にすることにより
    return false;
}

/* 独立ステアリングの操舵モーターを原点復帰 */
static bool home_one_motor(uint8_t i, float *home_position)
{
    uint8_t id = swerve_modules[i].steer_id;
    uint8_t mode;
    /* 停止 → 速度モード → 電流制限 → 速度0 */
    if (!home_send_ok(robostride_stop(id, true)) ||                            // 低電圧エラー解除 true が、停止と同時に故障フラグのクリアを要求 data[0] = 1にすることにより                       
        !home_send_ok(robostride_write_u8(id, RS_PARAM_RUN_MODE, 2)) ||       // 2 を書き込み、速度モードを指定
        !home_send_ok(robostride_write_float(id, RS_PARAM_LIMIT_CUR, 3.0f)) ||// 原点探索中のRobStrideの電流制限を3 Aに設定
        !home_send_ok(robostride_write_float(id, RS_PARAM_SPD_REF, 0.0f))) {  // 目標速度を 0 rad/s に設定
        return false;
    }

    /* 速度モードに切り替わったことを確認 */
    if (!home_read_param(id, RS_PARAM_RUN_MODE, &mode, 1) || mode != 2) {
        return false;
    }

    if (!home_send_ok(robostride_enable(id))) return false;  // ロボストライドの起動がうまくいかなかった時

    printf("HOME START id=%u\r\n", (unsigned int)id);
    fflush(stdout);

    float start_angle;
    if (!home_read_param(id, RS_PARAM_MECH_POS, &start_angle, sizeof(start_angle)) || !isfinite(start_angle) || robostride_feedback(i)->fault) {
        return false;
    }

    uint32_t started = HAL_GetTick();
    /* LOWで起動した場合は逆方向へ少し動かし、HIGHへ戻ることを確かめる。 */
    if (HAL_GPIO_ReadPin(GPIOC, swerve_modules[i].home_pin) == GPIO_PIN_RESET && !home_seek(i, start_angle, started, GPIO_PIN_SET, -0.5f * swerve_modules[i].home_speed_rad_s, swerve_modules[i].home_release_rad)) {
        return false;
    }

    if (!home_seek(i, start_angle, started, GPIO_PIN_RESET, swerve_modules[i].home_speed_rad_s, swerve_modules[i].home_travel_rad)) {
        return false;
    }

    /* 0を検出したら速度指令を0にする */
    if (!home_send_ok(robostride_write_float(id, RS_PARAM_SPD_REF, 0.0f))) {
        return false;
    }

    /* 停止指令直後の角度を記録 */
    float detected_angle;
    if (!home_read_param(id, RS_PARAM_MECH_POS, &detected_angle, sizeof(detected_angle)) || !isfinite(detected_angle)) {
      return false;
    }

    /* 速度が落ち着くまで待つ */
    uint32_t settling = HAL_GetTick(); // settling に停止確認を始めた時刻
    uint8_t still_count = 0;           // still_countは十分遅いという判定が何回連続
    while (still_count < 3) {
        float speed;
        if (HAL_GetTick() - settling >= 1000U || !home_read_param(id, RS_PARAM_MECH_SPEED, &speed, sizeof(speed)) || !isfinite(speed)) {
            return false;
        }
        still_count = fabsf(speed) < 0.03f ? (uint8_t)(still_count + 1) : 0; // 実測速度の絶対値が 0.03 rad/s 未満なら、カウントを1増やす。しきい値以上なら、カウントを0に戻す。
        HAL_Delay(20);
    }

    /* 正しく停止したら、止める*/
    if (!home_send_ok(robostride_stop(id, false))) {
        return false;
    }

    /* 停止後現在の多回転角度を読み取って保存 */
    if (!home_read_param(id, RS_PARAM_MECH_POS, home_position, sizeof(*home_position)) || !isfinite(*home_position)) {
        return false;
    }

    /* センサー検出後に最初に読んだ角度 detected_angle と、最終的に止まった角度 *home_position を比べる */
    float position_error = fabsf(*home_position - detected_angle);

    /* 停止前と停止後の角度の差が大きすぎたらアウト */
    if (position_error > HOME_POSITION_TOLERANCE_RAD) {
      printf("HOME FAILED: stop movement too large id=%u\r\n", (unsigned int)id);
      fflush(stdout);
      return false;
    }

    /* 位置モードへ。目標は0ではなく、いま保存した角度 */
    if (!home_send_ok(robostride_write_u8(id, RS_PARAM_RUN_MODE, 5)) ||                // 制御モードに 5 を書き、位置制御（CSP）を選ぶ
        !home_send_ok(robostride_write_float(id, RS_PARAM_LIMIT_SPD, 15.0f)) ||        // 位置制御で目標へ動くときの速度上限を、RobStride出力軸で 15 rad/s にする
        !home_send_ok(robostride_write_float(id, RS_PARAM_LOC_REF, *home_position))) { // 原点で停止したときに記録した角度を、位置制御の目標にする
        return false;
    }

    /* RobStrideが実際に 5 を返したことを確認 */
    if (!home_read_param(id, RS_PARAM_RUN_MODE, &mode, 1) || mode != 5) {
        return false;
    }

    if (!home_send_ok(robostride_enable(id))) return false;

    printf("HOME DONE id=%u\r\n", (unsigned int)id);
    fflush(stdout);

    if (robostride_feedback(i)->fault) {
        return false;
    }
    robostride_set_homed(i);             // 指定した車輪の原点復帰が完了したことを、STM32の内部に記録する関数

    return true;
}

static void stop_all_steering(void)
{
    for (uint8_t i = 0; i < SWERVE_WHEEL_COUNT; ++i) {
        (void)robostride_stop(swerve_modules[i].steer_id, false);
        HAL_Delay(10);
    }
}

bool swerve_home_all(float home_position[SWERVE_WHEEL_COUNT])
{
    HAL_Delay(1000);
    stop_all_steering();

    /* ID 1 → 2 → 127。1台でも失敗したら通常の走行指令へ進まない。 */
    for (uint8_t i = 0; i < SWERVE_WHEEL_COUNT; ++i) {
        if (!home_one_motor(i, &home_position[i])) {
            stop_all_steering();
            printf("HOME FAILED id=%u\r\n", (unsigned int)swerve_modules[i].steer_id);
            fflush(stdout);
            return false;
        }
    }
    printf("HOME ALL DONE\r\n");
    fflush(stdout);
    return true;
}
