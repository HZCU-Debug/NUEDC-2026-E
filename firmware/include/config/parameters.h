#pragma once

#include <stdint.h>

#include "config/hardware.h"

namespace config {

/** 按钮消抖时间，单位 ms */
constexpr uint32_t kButtonDebounceMs = 25;

/** XY 平面运动转速，单位 RPM */
constexpr uint16_t kXyMotionRpm =
    kXMotor.model == MotorModel::Atk ? 500 : 5000;
/** XY 平面运动加速度档位 */
constexpr uint8_t kXyMotionAcceleration =
    kXMotor.model == MotorModel::Atk ? 22 : 220;
/** Z 轴运动转速，单位 RPM */
constexpr uint16_t kZMotionRpm =
    kZMotor.model == MotorModel::Atk ? 40 : 400;
/** Z 轴运动加速度档位 */
constexpr uint8_t kZMotionAcceleration =
    kZMotor.model == MotorModel::Atk ? 18 : 180;
/** Roll 轴运动转速，单位 RPM */
constexpr uint16_t kRollMotionRpm =
    kRollMotor.model == MotorModel::Atk ? 30 : 300;
/** Roll 轴运动加速度档位 */
constexpr uint8_t kRollMotionAcceleration =
    kRollMotor.model == MotorModel::Atk ? 10 : 100;
/** 电机到位状态查询间隔，单位 ms */
constexpr uint32_t kMotorPollIntervalMs = 100;
/** 单个搬运运动阶段允许的最长时间，单位 ms */
constexpr uint32_t kMotorStageTimeoutMs = 3000;
/** 平移轴位置查询判定到位的最大角度误差，单位 ° */
constexpr float kLinearArrivalToleranceDegrees = 5.0f;
/** Roll 轴位置查询判定到位的最大角度误差，单位 ° */
constexpr float kRollArrivalToleranceDegrees = 5.0f;

/** 校准程序回零转速，单位 RPM */
constexpr uint16_t kCalibrationReturnRpm =
    kMotorLayout == MotorLayout::AllZdt ? 300 : 30;
/** 校准程序回零加速度档位 */
constexpr uint8_t kCalibrationReturnAcceleration =
    kMotorLayout == MotorLayout::AllZdt ? 10 : 1;

/** 电机测试程序加速度档位 */
constexpr uint8_t kMotorTestAcceleration =
    kMotorLayout == MotorLayout::AllZdt ? 100 : 10;
/** 电机测试程序最大转速，单位 RPM */
constexpr int16_t kMotorTestMaximumRpm = 6000;

/** A4 纸短边坐标范围，单位 mm */
constexpr uint16_t kPaperWidth = 210;
/** A4 纸长边坐标范围，单位 mm */
constexpr uint16_t kPaperHeight = 297;
/** 第一个标定点的纸面坐标，单位 mm */
constexpr uint16_t kCalibrationStart = 100;

}
