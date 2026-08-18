#pragma once

namespace gyroscope {

/**
 * @brief 一次陀螺仪采样，角速度单位为 rad/s，时间单位为 s
 */
struct Sample {
    float gyroX;
    float gyroY;
    float gyroZ;
    float dtSeconds;
};

/**
 * @brief 初始化陀螺仪
 * @return 初始化成功时返回 true
 */
bool begin();

/**
 * @brief 读取一组三轴角速度和实际采样间隔
 * @param sample 接收采样结果
 * @return 获得新采样时返回 true
 */
bool read(Sample& sample);

/**
 * @brief 停止陀螺仪
 */
void end();

}
