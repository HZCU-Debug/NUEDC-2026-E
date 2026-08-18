#pragma once

#include "runtime/program.h"

namespace runtime {

/**
 * @brief 保证同一时间只有一个 Program 运行
 */
class ProgramRunner {
public:
    /**
     * @brief 绑定共享系统状态
     * @param state 跨 Program 共享的系统状态
     */
    explicit ProgramRunner(SystemState& state);

    /**
     * @brief 停止旧 Program 并启动新 Program
     * @param program 即将启动的 Program
     * @param display 图形屏幕
     */
    void start(Program& program, Adafruit_GFX& display);

    /**
     * @brief 更新当前 Program
     * @param display 图形屏幕
     * @param event 本轮输入事件
     */
    void update(Adafruit_GFX& display, ui::Event event);

    /**
     * @brief 停止当前 Program
     */
    void stop();

private:
    SystemState& state_;
    Program* active_;
};

}
