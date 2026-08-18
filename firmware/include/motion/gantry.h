#pragma once

#include "motor/motor.h"

namespace motion {

/**
 * @brief 一次 XY 位置读取结果
 */
struct XYPosition {
    XYPosition() : x(0.0f), y(0.0f) {}
    XYPosition(float x, float y) : x(x), y(y) {}

    /** X 轴角度 */
    float x;
    /** Y 轴角度 */
    float y;
};

/**
 * @brief 一组运动轴的到位查询结果
 */
enum class Arrival : uint8_t {
    /** 尚未到位或本次查询失败 */
    Pending,
    /** 所有被查询的轴均已到位 */
    Reached,
    /** 驱动器报告堵转或保护状态 */
    Fault,
};

/**
 * @brief 运动机构轴标识
 */
enum class Axis : uint8_t {
    /** X 轴 */
    X,
    /** Y 轴 */
    Y,
    /** Z 轴 */
    Z,
    /** 旋转轴 */
    Rotation,
};

/**
 * @brief 最近一次轴操作的诊断信息
 */
struct PollDiagnostic {
    PollDiagnostic(char axis = '-', motor::Error error = motor::Error::None,
                   uint8_t detail = 0, uint8_t flags = 0)
        : axis(axis), error(error), detail(detail), flags(flags) {}

    /** 被查询的轴 */
    char axis;
    /** 查询失败原因，成功时为 Error::None */
    motor::Error error;
    /** 厂商 SDK 原始错误码，成功时为 0 */
    uint8_t detail;
    /** 查询成功时归一化后的状态字 */
    uint8_t flags;
};

/**
 * @brief 集中管理 X、Y、Z 和旋转轴电机配置与运动操作
 *
 * X 对应 1 号短边电机，Y 对应 2 号长边电机
 */
class Gantry {
public:
    /**
     * @brief 绑定四个运动轴
     * @param x X 轴电机
     * @param y Y 轴电机
     * @param z Z 轴电机
     * @param rotation 旋转轴电机
     */
    Gantry(motor::Motor& x, motor::Motor& y, motor::Motor& z,
           motor::Motor& rotation);

    /**
     * @brief 初始化总线并将各轴当前位置清零
     * @return 初始化结果
     */
    motor::Status begin();

    /**
     * @brief 同时使能或失能各轴
     * @param enabled true 表示使能，false 表示失能
     * @return 命令发送结果
     */
    motor::Status setEnabled(bool enabled = true);

    /**
     * @brief 使能或失能单个轴
     * @param axis 目标轴
     * @param enabled true 表示使能，false 表示失能
     * @return 命令发送结果
     */
    motor::Status setEnabled(Axis axis, bool enabled = true);

    /**
     * @brief 让单个轴以指定速度持续运行
     * @param axis 目标轴
     * @param signedRpm 带符号目标转速，0 表示停止该轴
     * @param acceleration 归一化加速度参数，范围 0–255
     * @return 命令执行结果
     */
    motor::Status run(Axis axis, int16_t signedRpm,
                      uint8_t acceleration = 0);

    /**
     * @brief 让单个轴移动到绝对角度
     * @param axis 目标轴
     * @param degrees 目标角度
     * @param options 速度和加速度参数
     * @return 命令执行结果
     */
    motor::Status moveAbsolute(Axis axis, float degrees,
                               const motor::MotionOptions& options);

    /**
     * @brief 同步移动 X 和 Y 轴到绝对角度
     * @param xDegrees X 轴目标角度
     * @param yDegrees Y 轴目标角度
     * @param options 速度和加速度参数
     * @return 命令发送结果
     */
    motor::Status moveXY(float xDegrees, float yDegrees,
                         const motor::MotionOptions& options);

    /**
     * @brief 同步移动 X、Y 和旋转轴到绝对角度
     * @param xDegrees X 轴目标角度
     * @param yDegrees Y 轴目标角度
     * @param rotationDegrees 旋转轴目标角度
     * @param xyOptions X 和 Y 轴速度与加速度参数
     * @param rotationOptions 旋转轴速度与加速度参数
     * @return 命令发送结果
     */
    motor::Status moveXYRotation(float xDegrees, float yDegrees,
                                 float rotationDegrees,
                                 const motor::MotionOptions& xyOptions,
                                 const motor::MotionOptions& rotationOptions);

    /**
     * @brief 同步移动 X、Y 到绝对角度并让旋转轴转动相对角度
     * @param xDegrees X 轴目标角度
     * @param yDegrees Y 轴目标角度
     * @param rotationDegrees 旋转轴相对转动角度
     * @param xyOptions X 和 Y 轴速度与加速度参数
     * @param rotationOptions 旋转轴速度与加速度参数
     * @return 命令发送结果
     */
    motor::Status moveXYRelativeRotation(
        float xDegrees, float yDegrees, float rotationDegrees,
        const motor::MotionOptions& xyOptions,
        const motor::MotionOptions& rotationOptions);

    /**
     * @brief 移动 Z 轴到绝对角度
     * @param degrees Z 轴目标角度
     * @param options 速度和加速度参数
     * @return 命令发送结果
     */
    motor::Status moveZ(float degrees, const motor::MotionOptions& options);

    /**
     * @brief 查询 X 和 Y 轴是否均已到位
     * @return 到位、等待或故障状态
     */
    Arrival pollXY();

    /**
     * @brief 查询 X、Y 和旋转轴是否均已到位
     * @return 到位、等待或故障状态
     */
    Arrival pollXYRotation();

    /**
     * @brief 查询 Z 轴是否已经到位
     * @return 到位、等待或故障状态
     */
    Arrival pollZ();

    /**
     * @brief 获取最近一次轴操作的轴、错误和状态字
     * @return 最近一次操作诊断信息
     */
    PollDiagnostic pollDiagnostic() const;

    /**
     * @brief 读取 X 和 Y 轴当前位置
     * @return 两轴角度或通信错误
     */
    motor::Result<XYPosition> readXY();

    /**
     * @brief 读取 Z 轴当前位置
     * @return Z 轴角度或通信错误
     */
    motor::Result<float> readZ();

    /**
     * @brief 停止各轴运动
     * @return 所有停止命令均成功时返回成功
     */
    motor::Status stop();

private:
    motor::Status record(char axis, const motor::Status& status);
    motor::Motor& motorFor(Axis axis);
    motor::Status moveXYRotationImpl(
        float xDegrees, float yDegrees, float rotationDegrees,
        const motor::MotionOptions& xyOptions,
        const motor::MotionOptions& rotationOptions, bool relativeRotation);

    motor::Motor& x_;
    motor::Motor& y_;
    motor::Motor& z_;
    motor::Motor& rotation_;
    PollDiagnostic pollDiagnostic_;
    float targetX_;
    float targetY_;
    float targetZ_;
    float targetRotation_;
};

/**
 * @brief 获取固件共享的运动机构
 * @return 运动机构实例
 */
Gantry& systemGantry();

}
