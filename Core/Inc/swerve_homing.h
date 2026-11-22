#ifndef SWERVE_HOMING_H
#define SWERVE_HOMING_H

#include <stdbool.h>
#include "swerve_config.h"

/* 右前、左前、後ろの順に原点を合わせ、RobStride出力軸の角度を返す。 */
bool swerve_home_all(float home_position[SWERVE_WHEEL_COUNT]);

#endif
