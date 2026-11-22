#ifndef SWERVE_CONFIG_H
#define SWERVE_CONFIG_H

#include <stdint.h>

#define SWERVE_WHEEL_COUNT 3U
#define SWERVE_STEER_LIMIT_RAD 1.57079632679f  /* 車輪側で±90度 */

typedef struct {
    float x_m, y_m;             /* 車体中心から各操舵軸まで [m]。前+X、左+Y */
    float wheel_radius_m;      /* 接地する車輪の半径 [m] */
    float drive_ratio;          /* N5065軸の回転数 / 接地車輪の回転数 */
    float steer_ratio;          /* RobStride出力軸の回転数 / 操舵軸の回転数 */
    float steer_sign;
    float drive_sign;
    float home_speed_rad_s;     /* RobStride出力軸の探索速度 */
    float home_travel_rad;      /* 操舵軸側で許可する探索角度 */
    float home_release_rad;     /* 起動時LOWから抜けるときの許容角度 */
    uint16_t home_pin;          /* GPIOC。原点センサーはLOWで検出 */
    uint8_t steer_id;
} SwerveModule;

extern const SwerveModule swerve_modules[SWERVE_WHEEL_COUNT];

#endif
