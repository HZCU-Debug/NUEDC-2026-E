#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace atk {

/**
 * @brief SDK 操作失败原因
 */
enum class Error : uint8_t {
    /** 操作成功 */
    None,
    /** Bus::begin() 尚未成功调用 */
    NotStarted,
    /** 串口配置、驱动器地址或运动参数超出支持范围 */
    InvalidArgument,
    /** 串口未能写入完整命令帧 */
    WriteFailed,
    /** 在配置的超时时间内未收到完整应答 */
    Timeout,
    /** 应答的地址、功能码、长度、校验码或帧尾不正确 */
    InvalidResponse,
    /** 驱动器拒绝执行命令 */
    DeviceRejected,
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
 * @brief PDxxS1 私有串口协议配置
 */
struct BusConfig {
    BusConfig(uint32_t baudRate = 115200, int8_t rxPin = -1,
              int8_t txPin = -1, uint32_t timeoutMs = 150,
              uint32_t commandIntervalMs = 2)
        : baudRate(baudRate),
          rxPin(rxPin),
          txPin(txPin),
          timeoutMs(timeoutMs),
          commandIntervalMs(commandIntervalMs) {}

    /** UART 波特率，必须与驱动器一致 */
    uint32_t baudRate;
    /** ESP32 RX 引脚，-1 表示使用串口默认引脚 */
    int8_t rxPin;
    /** ESP32 TX 引脚，-1 表示使用串口默认引脚 */
    int8_t txPin;
    /** 等待一帧应答的最长时间，单位毫秒 */
    uint32_t timeoutMs;
    /** 相邻命令之间的最短间隔，必须至少为 2 毫秒 */
    uint32_t commandIntervalMs;
};

/**
 * @brief 单个 PDxxS1 驱动器的逻辑配置
 */
struct MotorConfig {
    explicit MotorConfig(uint8_t address, bool invertDirection = false,
                         int8_t rxPin = -1)
        : address(address), invertDirection(invertDirection), rxPin(rxPin) {}

    /** 驱动器地址，范围 1–255 */
    uint8_t address;
    /** 是否反转上层正负方向与驱动器方向的映射 */
    bool invertDirection;
    /** 此驱动器独立连接的 ESP32 RX 引脚，-1 表示保持总线当前引脚 */
    int8_t rxPin;
};

/**
 * @brief 位置运动参数
 */
struct MotionOptions {
    explicit MotionOptions(uint16_t rpm, uint8_t acceleration = 0)
        : rpm(rpm), acceleration(acceleration) {}

    /** 最大转速，范围 1–6000 RPM */
    uint16_t rpm;
    /** 驱动器加减速度档位，范围 0–200 */
    uint8_t acceleration;
};

/**
 * @brief 驱动器支持的回零方式
 */
enum class HomeMode : uint8_t {
    /** 单圈回零 */
    SingleTurn = 0,
    /** 就近回零 */
    Nearest = 1,
    /** 多圈回零 */
    MultiTurn = 2,
};

/**
 * @brief 电机运行状态
 */
struct MotorState {
    MotorState()
        : enabled(false), reached(false), faulted(false), stalled(false) {}

    /** 电机是否处于 Operation Enabled 状态 */
    bool enabled;
    /** 位置命令是否已经到位 */
    bool reached;
    /** 驱动器是否报告过载、堵转或欠压 */
    bool faulted;
    /** 驱动器是否检测到堵转 */
    bool stalled;
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
    /** 带符号实时转速 */
    float speedRpm;
};

class Motor;

/**
 * @brief 管理一条 HardwareSerial 总线上的 PDxxS1 私有协议通信
 */
class Bus {
public:
    /**
     * @brief 绑定一个硬件串口
     * @param serial Arduino 硬件串口
     * @param config UART 总线配置
     */
    explicit Bus(HardwareSerial& serial,
                 const BusConfig& config = BusConfig());

    /**
     * @brief 初始化 UART
     * @return 初始化结果
     */
    Status begin();

private:
    friend class Motor;

    Status command(uint8_t address, int8_t rxPin, uint8_t function,
                   const uint8_t* payload, uint8_t payloadSize,
                   uint8_t* response, uint8_t responseSize);
    Status send(const uint8_t* frame, size_t size);
    Status receive(uint8_t address, uint8_t function, uint8_t* response,
                   uint8_t responseSize);
    void discardInput();

    HardwareSerial& serial_;
    BusConfig config_;
    bool started_;
};

/**
 * @brief 使用角度、RPM 和状态等上层语义控制一个 PDxxS1 驱动器
 */
class Motor {
public:
    /**
     * @brief 创建电机句柄
     * @param bus 电机所在的串口总线
     * @param config 电机地址和方向配置
     */
    Motor(Bus& bus, const MotorConfig& config);

    /**
     * @brief 使能电机输出
     * @return 驱动器应答结果
     */
    Status enable();

    /**
     * @brief 失能电机输出
     * @return 驱动器应答结果
     */
    Status disable();

    /**
     * @brief 将驱动器当前位置设置为零点
     * @return 驱动器应答结果
     */
    Status clearPosition();

    /**
     * @brief 以速度模式持续运行
     * @param signedRpm 带符号目标转速，范围 -6000–6000 RPM，0 表示停止
     * @param acceleration 加减速度，单位 R/s²
     * @return 运动参数配置或执行结果
     */
    Status run(int16_t signedRpm, uint8_t acceleration = 0);

    /**
     * @brief 按带符号角度相对移动
     * @param degrees 相对运动角度，正负号表示逻辑方向
     * @param options 位置运动参数
     * @return 运动参数配置或执行结果
     */
    Status moveRelative(float degrees, const MotionOptions& options);

    /**
     * @brief 移动到相对驱动器零点的带符号绝对角度
     * @param degrees 目标角度，正负号表示逻辑方向
     * @param options 位置运动参数
     * @return 运动参数配置或执行结果
     */
    Status moveAbsolute(float degrees, const MotionOptions& options);

    /**
     * @brief 将速度设置为零以停止电机
     * @return 驱动器应答结果
     */
    Status stop();

    /**
     * @brief 使用驱动器已保存的参数触发回零
     * @param mode 回零方式
     * @return 驱动器应答结果
     */
    Status home(HomeMode mode = HomeMode::Nearest);

    /**
     * @brief 读取使能、到位、故障和堵转状态
     * @return 电机状态或通信错误
     */
    Result<MotorState> readState();

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
    Status command(uint8_t function, const uint8_t* payload,
                   uint8_t payloadSize, uint8_t* response,
                   uint8_t responseSize);
    Status move(float degrees, const MotionOptions& options, bool absolute);
    bool valid() const;
    bool directionFor(bool negative) const;

    Bus& bus_;
    MotorConfig config_;
};

}
