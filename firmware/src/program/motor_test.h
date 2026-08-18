#pragma once

#include <Arduino.h>

#include "comm/link.h"
#include "motion/gantry.h"
#include "runtime/program.h"

namespace program {

/** 单轴速度命令消息类型 */
const uint8_t kMotorSpeedMessage = 0x01;

/** 单轴绝对位置命令消息类型 */
const uint8_t kMotorPositionMessage = 0x02;

/**
 * @brief 接收上位机单轴速度和绝对位置命令
 */
class MotorTestProgram final : public runtime::Program {
public:
    /**
     * @brief 绑定上位机串口和运动机构
     * @param serial 上位机通信串口
     * @param gantry 运动机构
     * @param config 串口链路配置
     */
    MotorTestProgram(
        HardwareSerial& serial, motion::Gantry& gantry,
        const comm::LinkConfig& config = comm::LinkConfig());

    /**
     * @brief 启动单轴电机测试
     * @param display 屏幕
     * @param state 共享系统状态
     */
    void start(Adafruit_GFX& display, runtime::SystemState& state) override;

    /**
     * @brief 接收并执行单轴速度或绝对位置命令
     * @param display 屏幕
     * @param state 共享系统状态
     * @param event 本轮菜单事件
     */
    void update(Adafruit_GFX& display, runtime::SystemState& state,
                ui::Event event) override;

    /**
     * @brief 请求停止全部电机并退出
     */
    void requestExit() override;

    /**
     * @brief 判断电机是否已经停止
     * @return 可以退出时返回 true
     */
    bool readyToExit() const override;

    /**
     * @brief 停止通信和运动
     * @param state 共享系统状态
     */
    void stop(runtime::SystemState& state) override;

private:
    enum class State : uint8_t {
        Ready,
        Idle,
        Error,
    };

    bool applySpeed(const comm::MessageView& message);
    bool applyPosition(const comm::MessageView& message);
    void render(Adafruit_GFX& display, const char* status) const;

    comm::Link<7> link_;
    motion::Gantry& gantry_;
    State state_;
    char axis_;
    int16_t rpm_;
    float degrees_;
    bool positionMode_;
    bool exitReady_;
};

}
