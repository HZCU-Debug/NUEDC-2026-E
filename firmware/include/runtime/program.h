#pragma once

#include <Adafruit_GFX.h>

#include "runtime/system_state.h"
#include "ui/menu.h"

namespace runtime {

/**
 * @brief 可独占运行并绘制活动页面的程序
 */
class Program {
public:
    /**
     * @brief 销毁程序
     */
    virtual ~Program() = default;

    /**
     * @brief 启动程序并初始化本轮运行上下文
     * @param display 图形屏幕
     * @param state 共享系统状态
     */
    virtual void start(Adafruit_GFX& display, SystemState& state) = 0;

    /**
     * @brief 更新程序逻辑和活动页面
     * @param display 图形屏幕
     * @param state 共享系统状态
     * @param event 本轮输入事件
     */
    virtual void update(Adafruit_GFX& display, SystemState& state,
                        ui::Event event) = 0;

    /**
     * @brief 请求程序结束并执行必要的异步收尾
     */
    virtual void requestExit() {}

    /**
     * @brief 判断程序是否可以结束
     * @return 可以结束时返回 true
     */
    virtual bool readyToExit() const { return true; }

    /**
     * @brief 停止程序并结束业务活动
     * @param state 共享系统状态
     */
    virtual void stop(SystemState&) {}
};

}
