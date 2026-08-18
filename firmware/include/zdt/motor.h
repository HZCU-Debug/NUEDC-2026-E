#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace zdt {

/**
 * @brief SDK 操作失败原因
 */
enum class Error : uint8_t {
    /** 操作成功 */
    None,
    /** Bus::begin() 尚未成功调用 */
    NotStarted,
    /** 地址、速度、角度或配置超出支持范围 */
    InvalidArgument,
    /** 串口未能写入完整命令帧 */
    WriteFailed,
    /** 在配置的超时时间内未收到完整应答 */
    Timeout,
    /** 应答的地址、功能码、长度或校验字节不正确 */
    InvalidResponse,
};

/**
 * @brief 不携带返回数据的操作结果
 */
struct Status {
    explicit Status(Error error = Error::None) : error(error) {}

    /**
     * @brief 判断操作是否成功
     * @return 成功时返回 true
     */
    explicit operator bool() const { return error == Error::None; }

    /** 成功时为 Error::None */
    Error error;
};

/**
 * @brief 携带返回数据的操作结果
 * @tparam T 返回数据类型
 */
template <typename T>
struct Result {
    explicit Result(Error error = Error::None) : value(), error(error) {}
    explicit Result(const T& value) : value(value), error(Error::None) {}

    /**
     * @brief 判断操作是否成功
     * @return 成功时返回 true
     */
    explicit operator bool() const { return error == Error::None; }

    /** 操作成功时的返回数据 */
    T value;
    /** 成功时为 Error::None */
    Error error;
};

/**
 * @brief 命令执行方式
 */
enum class Start : uint8_t {
    /** 立即执行 */
    Immediate = 0,
    /** 等待 Bus::triggerSynchronized() 统一触发 */
    Synchronized = 1,
};

/**
 * @brief 驱动器支持的回零方式
 */
enum class HomeMode : uint8_t {
    /** 单圈就近回零 */
    Nearest = 0,
    /** 单圈方向回零 */
    Directional = 1,
    /** 多圈无限位碰撞回零 */
    Sensorless = 2,
    /** 多圈有限位开关回零 */
    EndStop = 3,
};

/**
 * @brief UART 总线配置
 *
 * 协议使用 8N1 和固定 0x6B 校验字节
 */
struct BusConfig {
    BusConfig(uint32_t baudRate = 115200, int8_t rxPin = -1, int8_t txPin = -1,
              uint32_t timeoutMs = 150, uint32_t queryIntervalMs = 10)
        : baudRate(baudRate),
          rxPin(rxPin),
          txPin(txPin),
          timeoutMs(timeoutMs),
          queryIntervalMs(queryIntervalMs) {}

    /** UART 波特率，必须与驱动器一致 */
    uint32_t baudRate;
    /** ESP32 RX 引脚，-1 表示使用串口默认引脚 */
    int8_t rxPin;
    /** ESP32 TX 引脚，-1 表示使用串口默认引脚 */
    int8_t txPin;
    /** 等待一帧应答的最长时间，单位毫秒 */
    uint32_t timeoutMs;
    /** 相邻查询前保留的总线静默时间，单位毫秒 */
    uint32_t queryIntervalMs;
};

/**
 * @brief 单个电机的逻辑配置
 */
struct MotorConfig {
    MotorConfig(uint8_t address, uint32_t pulsesPerRevolution, bool invertDirection = false)
        : address(address),
          pulsesPerRevolution(pulsesPerRevolution),
          invertDirection(invertDirection) {}

    /** 驱动器地址，范围 1–255 */
    uint8_t address;
    /** 当前细分设置下电机转一圈所需的脉冲数 */
    uint32_t pulsesPerRevolution;
    /** 是否反转上层正负方向与驱动器方向的映射 */
    bool invertDirection;
};

/**
 * @brief 位置运动参数
 */
struct MotionOptions {
    MotionOptions(uint16_t rpm, uint8_t acceleration = 0,
                  Start start = Start::Immediate)
        : rpm(rpm), acceleration(acceleration), start(start) {}

    /** 最大转速，范围 1–5000 RPM */
    uint16_t rpm;
    /** 驱动器加速度档位，0 表示直接启动 */
    uint8_t acceleration;
    /** 立即执行或等待同步触发 */
    Start start;
};

/**
 * @brief 电机运行状态
 */
struct MotorState {
    MotorState()
        : enabled(false),
          reached(false),
          stalled(false),
          stallProtected(false),
          homing(false),
          homingFailed(false) {}

    /** 电机输出是否已使能 */
    bool enabled;
    /** 位置命令是否已经到位 */
    bool reached;
    /** 驱动器是否检测到堵转 */
    bool stalled;
    /** 堵转保护是否已经触发 */
    bool stallProtected;
    /** 电机是否正在回零 */
    bool homing;
    /** 最近一次回零是否失败 */
    bool homingFailed;
};

/**
 * @brief 一次读取获得的状态、位置和速度
 */
struct MotorSnapshot {
    MotorSnapshot() : state(), positionDegrees(0.0f), speedRpm(0.0f) {}

    /** 电机运行状态 */
    MotorState state;
    /** 相对驱动器零点的带符号角度 */
    float positionDegrees;
    /** 带符号实时转速，符号表示逻辑方向 */
    float speedRpm;
};

class Motor;

/**
 * @brief 管理一条 HardwareSerial 总线上的收发、超时和多电机同步
 *
 * 控制命令只确认串口写入，查询命令等待驱动器应答
 */
class Bus {
public:
    /**
     * @brief 绑定一个硬件串口
     * @param serial Arduino 硬件串口
     * @param config UART 总线配置
     */
    explicit Bus(HardwareSerial& serial, const BusConfig& config = BusConfig());

    /**
     * @brief 初始化 UART
     * @return 初始化结果
     */
    Status begin();

    /**
     * @brief 广播触发所有已接收同步命令的电机
     * @return 命令帧发送结果
     */
    Status triggerSynchronized();

    /**
     * @brief 广播清零总线上所有电机的当前角度和脉冲计数
     * @return 命令帧发送结果
     */
    Status clearPositions();

    /**
     * @brief 广播使能或失能总线上的所有电机
     * @param enabled true 表示使能，false 表示失能
     * @return 命令帧发送结果
     */
    Status enableAll(bool enabled = true);

private:
    friend class Motor;

    Status command(const uint8_t* frame, size_t size);
    Status query(uint8_t address, uint8_t function, const uint8_t* frame, size_t frameSize,
                 uint8_t* response, size_t responseSize);
    Status send(const uint8_t* frame, size_t size);
    Status receive(uint8_t address, uint8_t function, uint8_t* response, size_t size);
    void discardInput();

    HardwareSerial& serial_;
    BusConfig config_;
    bool started_;
};

/**
 * @brief 使用角度、RPM 和状态等上层语义控制一个地址上的电机
 */
class Motor {
public:
    /**
     * @brief 创建电机句柄
     * @param bus 电机所在的总线
     * @param config 电机逻辑配置
     */
    Motor(Bus& bus, const MotorConfig& config);

    /**
     * @brief 使能或失能电机输出
     * @param enabled true 表示使能，false 表示失能
     * @return 命令帧发送结果
     */
    Status enable(bool enabled = true);

    /**
     * @brief 失能电机输出
     * @return 命令帧发送结果
     */
    Status disable() { return enable(false); }

    /**
     * @brief 将驱动器当前角度、位置误差和脉冲计数清零
     * @return 命令帧发送结果
     */
    Status clearPosition();

    /**
     * @brief 以速度模式持续运行
     * @param signedRpm 带符号目标转速，范围 -5000–5000 RPM，不能为 0
     * @param acceleration 驱动器加速度档位，0 表示直接启动
     * @param start 命令执行方式
     * @return 命令帧发送结果
     */
    Status run(int16_t signedRpm, uint8_t acceleration = 0,
               Start start = Start::Immediate);

    /**
     * @brief 按带符号角度相对移动
     * @param degrees 相对运动角度，正负号表示逻辑方向
     * @param options 位置运动参数
     * @return 命令帧发送结果，不表示运动已经到位
     */
    Status moveRelative(float degrees, const MotionOptions& options);

    /**
     * @brief 移动到相对驱动器零点的带符号绝对角度
     * @param degrees 目标角度，正负号表示逻辑方向
     * @param options 位置运动参数
     * @return 命令帧发送结果，不表示运动已经到位
     */
    Status moveAbsolute(float degrees, const MotionOptions& options);

    /**
     * @brief 停止电机
     * @param start 命令执行方式
     * @return 命令帧发送结果
     */
    Status stop(Start start = Start::Immediate);

    /**
     * @brief 使用驱动器已保存的参数触发回零
     * @param mode 回零方式
     * @param start 命令执行方式
     * @return 命令帧发送结果
     */
    Status home(HomeMode mode = HomeMode::Nearest, Start start = Start::Immediate);

    /**
     * @brief 读取电机状态和回零状态
     * @return 电机状态或通信错误
     */
    Result<MotorState> readState();

    /**
     * @brief 读取使能、到位、堵转和保护状态
     * @return 电机运动状态或通信错误
     */
    Result<MotorState> readMotionState();

    /**
     * @brief 读取相对驱动器零点的带符号实时角度
     * @return 实时角度或通信错误
     */
    Result<float> readPositionDegrees();

    /**
     * @brief 读取带符号实时转速
     * @return RPM 转速或通信错误
     */
    Result<float> readSpeedRpm();

    /**
     * @brief 顺序读取状态、位置和速度
     * @return 电机快照或任意一次查询产生的错误
     */
    Result<MotorSnapshot> readSnapshot();

private:
    Status move(float degrees, const MotionOptions& options, bool absolute);
    bool valid() const;
    uint8_t directionFor(bool negative) const;

    Bus& bus_;
    MotorConfig config_;
};

}
