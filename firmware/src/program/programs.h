/**
 * @file programs.h
 * @brief 固件菜单使用的 Program 声明
 */
#pragma once

#include "runtime/program.h"

namespace program {

/**
 * @brief 初始化三轴电机总线并将当前位置清零
 * @return 三个电机均清零成功时返回 true
 */
bool initializeMotors();

/**
 * @brief Z 轴下降位置校准
 * @return Z 轴校准 Program
 */
runtime::Program& zCalibration();

/**
 * @brief A4 纸二维坐标校准
 * @return 纸张坐标校准 Program
 */
runtime::Program& paperCalibration();

/**
 * @brief 让各运动轴回到电机零点
 * @return 回零 Program
 */
runtime::Program& home();

/**
 * @brief 单轴电机速度测试
 * @return 电机测试 Program
 */
runtime::Program& motorTest();

/**
 * @brief 手柄控制电机 Demo
 * @return Demo Program
 */
runtime::Program& controllerMotor();

/**
 * @brief 电机速度梯度 Demo
 * @return Demo Program
 */
runtime::Program& motorRamp();

/**
 * @brief 电机位置读取 Demo
 * @return Demo Program
 */
runtime::Program& motorPosition();

/**
 * @brief 非可靠消息发送 Demo
 * @return Demo Program
 */
runtime::Program& commUnreliable();

/**
 * @brief 可靠消息发送 Demo
 * @return Demo Program
 */
runtime::Program& commReliable();

/**
 * @brief IMU660RB 四元数积分 Demo
 * @return Demo Program
 */
runtime::Program& quaternion();

}
