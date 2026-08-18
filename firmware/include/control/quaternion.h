#pragma once

#include <math.h>

namespace control {

/**
 * @brief 标量在前的单位四元数
 */
struct Quaternion {
    /** 标量分量 */
    float scalar;
    /** X 轴虚部 */
    float x;
    /** Y 轴虚部 */
    float y;
    /** Z 轴虚部 */
    float z;
};

/**
 * @brief 使用三轴角速度和实际时间间隔积分姿态四元数
 */
class QuaternionIntegrator {
public:
    /**
     * @brief 创建单位姿态积分器
     */
    QuaternionIntegrator() : value_{1.0f, 0.0f, 0.0f, 0.0f} {}

    /**
     * @brief 重置为单位姿态
     */
    void reset() { value_ = {1.0f, 0.0f, 0.0f, 0.0f}; }

    /**
     * @brief 按本体三轴角速度更新姿态
     * @param gyroXRadPerSecond X 轴角速度，单位 rad/s
     * @param gyroYRadPerSecond Y 轴角速度，单位 rad/s
     * @param gyroZRadPerSecond Z 轴角速度，单位 rad/s
     * @param dtSeconds 距离上次更新的实际时间，单位 s
     */
    void update(float gyroXRadPerSecond, float gyroYRadPerSecond,
                float gyroZRadPerSecond, float dtSeconds) {
        if (dtSeconds <= 0.0f) {
            return;
        }

        const float speed = sqrtf(gyroXRadPerSecond * gyroXRadPerSecond +
                                  gyroYRadPerSecond * gyroYRadPerSecond +
                                  gyroZRadPerSecond * gyroZRadPerSecond);
        if (speed == 0.0f) {
            return;
        }

        const float halfAngle = speed * dtSeconds * 0.5f;
        const float scale = sinf(halfAngle) / speed;
        const Quaternion delta = {cosf(halfAngle),
                                  gyroXRadPerSecond * scale,
                                  gyroYRadPerSecond * scale,
                                  gyroZRadPerSecond * scale};
        const Quaternion previous = value_;
        value_ = {previous.scalar * delta.scalar - previous.x * delta.x -
                      previous.y * delta.y - previous.z * delta.z,
                  previous.scalar * delta.x + previous.x * delta.scalar +
                      previous.y * delta.z - previous.z * delta.y,
                  previous.scalar * delta.y - previous.x * delta.z +
                      previous.y * delta.scalar + previous.z * delta.x,
                  previous.scalar * delta.z + previous.x * delta.y -
                      previous.y * delta.x + previous.z * delta.scalar};
        normalize();
    }

    /**
     * @brief 获取当前姿态
     * @return 标量在前的单位四元数
     */
    const Quaternion& value() const { return value_; }

private:
    void normalize() {
        const float inverseNorm =
            1.0f / sqrtf(value_.scalar * value_.scalar + value_.x * value_.x +
                         value_.y * value_.y + value_.z * value_.z);
        value_.scalar *= inverseNorm;
        value_.x *= inverseNorm;
        value_.y *= inverseNorm;
        value_.z *= inverseNorm;
    }

    Quaternion value_;
};

}
