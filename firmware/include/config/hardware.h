#pragma once

#include <stdint.h>

namespace config {

/**
 * @brief 控制板与外设之间的 GPIO 映射
 */
struct Pins {
    /** 屏幕 SPI 时钟 */
    int8_t displayClock;
    /** 屏幕复位 */
    int8_t displayReset;
    /** 屏幕 SPI 数据 */
    int8_t displayData;
    /** 屏幕背光 */
    int8_t displayBacklight;
    /** 屏幕数据命令选择 */
    int8_t displayDc;
    /** 屏幕片选 */
    int8_t displayCs;
    /** 电磁铁控制 */
    int8_t electromagnet;
    /** 低电平触发蜂鸣器 */
    int8_t buzzer;
    /** 向上按键 */
    int8_t upButton;
    /** 向下按键 */
    int8_t downButton;
    /** 确认按键 */
    int8_t selectButton;
    /** 返回按键 */
    int8_t backButton;
    /** IMU I2C 数据 */
    int8_t imuSda;
    /** IMU I2C 时钟 */
    int8_t imuScl;
    /** X 轴电机串口接收，张大头总线也使用此引脚 */
    int8_t motorRx;
    /** Y 轴电机串口接收 */
    int8_t motorYRx;
    /** Z 轴电机串口接收 */
    int8_t motorZRx;
    /** Roll 轴电机串口接收 */
    int8_t motorRollRx;
    /** 电机串口发送 */
    int8_t motorTx;
    /** 正点原子电机串口发送，-1 表示与其他电机共用 */
    int8_t atkMotorTx;
    /** 视觉串口接收 */
    int8_t visionRx;
    /** 视觉串口发送 */
    int8_t visionTx;
};

/** 按钮是否使用 ESP32 内部上拉 */
constexpr bool kButtonsUseInternalPullup =
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    true;
#else
    false;
#endif

/** 电机型号 */
enum class MotorModel : uint8_t {
    /** 张大头电机 */
    Zdt,
    /** 正点原子电机 */
    Atk,
};

/** 电机组合 */
enum class MotorLayout : uint8_t {
    /** 四轴全部使用张大头电机 */
    AllZdt = 0,
    /** 四轴全部使用正点原子电机 */
    AllAtk = 1,
    /** X 和 Y 使用张大头，Z 和 Roll 使用正点原子 */
    Mixed = 2,
};

/** 当前电机组合 */
constexpr MotorLayout kMotorLayout = MotorLayout::AllAtk;

static_assert(kMotorLayout == MotorLayout::AllZdt ||
                  kMotorLayout == MotorLayout::AllAtk ||
                  kMotorLayout == MotorLayout::Mixed,
              "Unsupported motor layout");

/**
 * @brief 单轴电机硬件配置
 */
struct AxisMotorConfig {
    /** 电机型号 */
    MotorModel model;
    /** 驱动器地址 */
    uint8_t address;
    /** 张大头电机每圈脉冲数 */
    uint32_t pulsesPerRevolution;
    /** 是否反转逻辑方向 */
    bool invertDirection;
};

/** 电机总线波特率 */
constexpr uint32_t kMotorBaudRate = 115200;
/** Sol 完成后的蜂鸣时长 */
constexpr uint32_t kBuzzerDurationMs = 200;
/** X 轴电机配置 */
constexpr AxisMotorConfig kXMotor = {
    kMotorLayout == MotorLayout::AllAtk ? MotorModel::Atk : MotorModel::Zdt,
    1,
    3200,
    false,
};
/** Y 轴电机配置 */
constexpr AxisMotorConfig kYMotor = {
    kMotorLayout == MotorLayout::AllAtk ? MotorModel::Atk : MotorModel::Zdt,
    2,
    3200,
    false,
};
/** Z 轴电机配置 */
constexpr AxisMotorConfig kZMotor = {
    kMotorLayout == MotorLayout::AllZdt ? MotorModel::Zdt : MotorModel::Atk,
    3,
    3200,
    false,
};
/** Roll 轴电机配置 */
constexpr AxisMotorConfig kRollMotor = {
    kMotorLayout == MotorLayout::AllZdt ? MotorModel::Zdt : MotorModel::Atk,
    4,
    3200,
    false,
};

#if defined(BOARD_ESP32_S3_CBH)

/** ESP32-S3 CBH 控制板引脚 */
constexpr Pins kPins = {
    // IPS_SCL
    17,
    // IPS_RST
    4,
    // IPS_SDA
    16,
    // IPS_BL
    15,
    // IPS_DC
    5,
    // IPS_CS
    14,
    // MOS
    38,
    // BUZZER
    8,
    // S1
    9,
    // S2
    10,
    // S3
    11,
    // S4
    39,
    // IMU_SDA 未分配
    -1,
    // IMU_SCL 未分配
    -1,
    // RXD1
    41,
    // TXD1
    40,
    // RXD0
    44,
    // GPIO42
    42,
    // TXD1
    40,
    // TXD0
    43,
    // RXD2
    12,
    // TXD2
    13,
};

#elif defined(CONFIG_IDF_TARGET_ESP32S3)

/** ESP32-S3 N16R8 控制板引脚 */
constexpr Pins kPins = {
    // IPS_SCL
    16,
    // IPS_RST
    4,
    // IPS_SDA
    17,
    // IPS_BL
    15,
    // IPS_DC
    5,
    // IPS_CS
    14,
    // GPIO7
    7,
    // BUZZER
    8,
    // S1
    0,
    // S2
    10,
    // S3
    11,
    // S4
    39,
    // MPU_SDA
    21,
    // MPU_SCL
    18,
    // RXD1
    41,
    // GPIO42
    42,
    // GPIO1
    1,
    // GPIO2
    2,
    // TXD1
    40,
    // 正点原子独立发送未分配
    -1,
    // RXD2
    12,
    // TXD2
    13,
};

#elif defined(CONFIG_IDF_TARGET_ESP32)

/** ESP32 控制板引脚 */
constexpr Pins kPins = {
    // IPS_SCL
    18,
    // IPS_RST
    33,
    // IPS_SDA
    23,
    // IPS_BL
    12,
    // IPS_DC
    27,
    // IPS_CS
    32,
    // LED3
    10,
    // BUZZER
    4,
    // S1
    0,
    // S2
    35,
    // S3
    34,
    // S4
    39,
    // MPU_SDA
    21,
    // MPU_SCL
    22,
    // RXD1
    25,
    // GPIO13
    13,
    // GPIO14
    14,
    // GPIO19
    19,
    // TXD1
    26,
    // 正点原子独立发送未分配
    -1,
    // RXD2
    16,
    // TXD2
    17,
};

#else
#error "Unsupported ESP32 target"
#endif

}
