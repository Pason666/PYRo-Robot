#ifndef __CONFIG_H__
#define __CONFIG_H__
#include <cstdint>
#include "BMI088_driver.h"

// =========================================================
// 丝杠机构运动学参数 (由 SolidWorks 测算)
// L1 = 80.0mm, L2 = 106.8mm
// =========================================================
constexpr float SCREW_L1_SQ_PLUS_L2_SQ  = 17806.24f; // 80^2 + 106.8^2
constexpr float SCREW_TWO_L1_L2         = 17088.0f;  // 2 * 80 * 106.8
constexpr float SCREW_THETA_ZERO_RAD    = 1.2353835f; // 绝对水平时的推杆内部夹角 (70.78226度 转 rad)
constexpr float SCREW_S_LIMIT_BOTTOM    = 128.0424f; // 丝杠最底端初始长度 (mm)

// 云台上电校准时间 (以主循环 Tick 数计，假设 1kHz 则 1000 = 1秒)
constexpr uint32_t PITCH_CALIB_MAX_TICKS = 1000;

// =========================================================
// 限位与偏置
// =========================================================
constexpr float PITCH_MIN_RAD = -0.6f; // Pitch 轴最小角度 (rad)
constexpr float PITCH_MAX_RAD = -0.1f;  // Pitch 轴最大角度 (rad)

constexpr float YAW_OFFSET_RAD = -1.99801016f;

#endif