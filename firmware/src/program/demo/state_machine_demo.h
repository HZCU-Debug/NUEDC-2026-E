#pragma once

#include <Arduino.h>

#include "runtime/program.h"
#include "runtime/state_machine.h"

namespace program {

/**
 * @brief 串口状态机示例使用的状态
 */
enum class DemoState {
    /** 第一个入口 */
    EntryA,
    /** 第二个入口 */
    EntryB,
    /** 两个入口共享的状态 */
    Shared,
    /** 与共享状态形成循环的状态 */
    Loop,
};

/**
 * @brief 状态机示例启动配置
 */
struct DemoConfiguration {
    /**
     * @brief 创建启动配置
     * @param entry 状态机入口
     */
    explicit DemoConfiguration(DemoState entry) : entry(entry) {}

    /** 状态机入口 */
    DemoState entry;
};

/**
 * @brief 使用串口展示多入口和循环跳转的 Program
 */
class StateMachineDemoProgram final : public runtime::Program {
public:
    /**
     * @brief 绑定串口和共享状态机
     * @param serial 输出状态轨迹的串口
     * @param stateMachine 状态机实例
     */
    StateMachineDemoProgram(HardwareSerial& serial,
                            runtime::StateMachine<DemoState>& stateMachine);

    /**
     * @brief 设置下次启动使用的入口
     * @param configuration 启动配置
     */
    void configure(const DemoConfiguration& configuration);

    /**
     * @brief 从配置入口启动示例
     * @param display 图形屏幕
     * @param state 共享系统状态
     */
    void start(Adafruit_GFX& display, runtime::SystemState& state) override;

    /**
     * @brief 处理当前状态并完成一次跳转
     * @param display 图形屏幕
     * @param state 共享系统状态
     * @param event 本轮输入事件
     */
    void update(Adafruit_GFX& display, runtime::SystemState& state,
                ui::Event event) override;

private:
    void trace(const char* action, DemoState state);
    void transitionTo(DemoState next);
    void render(Adafruit_GFX& display) const;

    HardwareSerial& serial_;
    runtime::StateMachine<DemoState>& stateMachine_;
    DemoConfiguration configuration_;
};

}
