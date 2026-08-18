#pragma once

#include <stdint.h>

namespace motor {

/**
 * @brief 厂商无关的电机操作失败原因
 */
enum class Error : uint8_t {
    /** 操作成功 */
    None,
    /** 通信总线尚未初始化 */
    NotStarted,
    /** 参数超出当前驱动器支持范围 */
    InvalidArgument,
    /** 串口未能写入完整命令 */
    WriteFailed,
    /** 等待驱动器应答超时 */
    Timeout,
    /** 驱动器应答格式错误 */
    InvalidResponse,
    /** 驱动器拒绝执行命令 */
    DeviceRejected,
};

/**
 * @brief 不携带返回数据的电机操作结果
 */
struct Status {
    explicit Status(Error error = Error::None, uint8_t detail = 0)
        : error(error), detail(detail) {}

    /**
     * @brief 判断操作是否成功
     * @return 成功时返回 true
     */
    explicit operator bool() const { return error == Error::None; }

    /** 成功时为 Error::None */
    Error error;
    /** 厂商 SDK 原始错误码，成功时为 0 */
    uint8_t detail;
};

/**
 * @brief 携带返回数据的电机操作结果
 * @tparam T 返回数据类型
 */
template <typename T>
struct Result {
    explicit Result(Error error = Error::None, uint8_t detail = 0)
        : value(), error(error), detail(detail) {}
    explicit Result(const T& value) : value(value), error(Error::None), detail(0) {}

    /**
     * @brief 判断操作是否成功
     * @return 成功时返回 true
     */
    explicit operator bool() const { return error == Error::None; }

    /** 操作成功时的返回数据 */
    T value;
    /** 成功时为 Error::None */
    Error error;
    /** 厂商 SDK 原始错误码，成功时为 0 */
    uint8_t detail;
};

/**
 * @brief 厂商无关的位置运动参数
 */
struct MotionOptions {
    MotionOptions(uint16_t rpm, uint8_t acceleration = 0)
        : rpm(rpm), acceleration(acceleration) {}

    /** 最大转速，单位 RPM */
    uint16_t rpm;
    /** 当前驱动器的原始加速度档位 */
    uint8_t acceleration;
};

/**
 * @brief 厂商无关的电机运行状态
 */
struct State {
    State() : enabled(false), reached(false), faulted(false), stalled(false) {}

    /** 电机输出是否已使能 */
    bool enabled;
    /** 位置命令是否已经到位 */
    bool reached;
    /** 驱动器是否报告故障 */
    bool faulted;
    /** 驱动器是否检测到堵转 */
    bool stalled;
};

/**
 * @brief 电机业务层使用的最小控制接口
 */
class Motor {
public:
    /**
     * @brief 释放电机接口
     */
    virtual ~Motor() {}

    /**
     * @brief 初始化电机所在通信总线
     * @return 初始化结果
     */
    virtual Status begin() = 0;

    /**
     * @brief 使能或失能电机输出
     * @param enabled true 表示使能，false 表示失能
     * @return 命令执行结果
     */
    virtual Status enable(bool enabled = true) = 0;

    /**
     * @brief 将驱动器当前位置设置为零点
     * @return 命令执行结果
     */
    virtual Status clearPosition() = 0;

    /**
     * @brief 以速度模式持续运行
     * @param signedRpm 带符号目标转速，正负号表示方向
     * @param acceleration 当前驱动器的原始加速度档位
     * @return 命令执行结果
     */
    virtual Status run(int16_t signedRpm, uint8_t acceleration = 0) = 0;

    /**
     * @brief 按带符号角度相对移动
     * @param degrees 相对运动角度
     * @param options 速度和加速度参数
     * @return 命令执行结果
     */
    virtual Status moveRelative(float degrees,
                                const MotionOptions& options) = 0;

    /**
     * @brief 移动到带符号绝对角度
     * @param degrees 目标角度
     * @param options 速度和加速度参数
     * @return 命令执行结果
     */
    virtual Status moveAbsolute(float degrees,
                                const MotionOptions& options) = 0;

    /**
     * @brief 停止电机
     * @return 命令执行结果
     */
    virtual Status stop() = 0;

    /**
     * @brief 读取电机运行状态
     * @return 运行状态或通信错误
     */
    virtual Result<State> readState() = 0;

    /**
     * @brief 读取相对驱动器零点的实时角度
     * @return 实时角度或通信错误
     */
    virtual Result<float> readPositionDegrees() = 0;
};

}
