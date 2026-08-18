#pragma once

#include <Arduino.h>

#include "comm/link.h"
#include "motion/pick_place_runner.h"
#include "runtime/program.h"

namespace program {

/** 拼图数量消息类型 */
const uint8_t kPieceCountMessage = 0x01;
/** 单块拼图数据消息类型 */
const uint8_t kPieceDataMessage = 0x02;
/** ESP32 向视觉端发送的题号消息类型 */
const uint8_t kModeRequestMessage = 0x03;

/**
 * @brief 请求视觉模式，并在拼图模式下接收结果和执行搬运
 */
class MoveTaskProgram final : public runtime::Program {
public:
    /**
     * @brief 绑定视觉端串口和搬运状态机
     * @param serial 视觉端通信串口
     * @param runner 搬运状态机
     * @param config 串口链路配置
     * @param logVisionCoordinates 是否通过调试串口打印视觉坐标
     */
    MoveTaskProgram(HardwareSerial& serial, motion::PickPlaceRunner& runner,
                    const comm::LinkConfig& config = comm::LinkConfig(),
                    bool logVisionCoordinates = false);

    /**
     * @brief 配置本次启动的赛题模式
     * @param mode 模式号，0 为视觉标定，1 为 T1，2 为 T2-1，3 为 T2-2
     */
    void configure(uint8_t mode);

    /**
     * @brief 初始化通信并显示等待页面
     * @param display 屏幕
     * @param state 共享系统状态
     */
    void start(Adafruit_GFX& display, runtime::SystemState& state) override;

    /**
     * @brief 处理视觉标定交互或推进搬运状态机
     * @param display 屏幕
     * @param state 共享系统状态
     * @param event 本轮菜单事件
     */
    void update(Adafruit_GFX& display, runtime::SystemState& state,
                ui::Event event) override;

    /**
     * @brief 停止当前动作并开始三轴回零
     */
    void requestExit() override;

    /**
     * @brief 判断三轴是否已经完成回零
     * @return 可以返回菜单时返回 true
     */
    bool readyToExit() const override;

    /**
     * @brief 停止搬运和可靠消息重传
     * @param state 共享系统状态
     */
    void stop(runtime::SystemState& state) override;

private:
    enum class State : uint8_t {
        SendingMode,
        WaitingTask,
        Running,
        Returning,
        Finished,
        Error,
    };

    bool receiveCount(const comm::MessageView& message);
    bool receivePiece(const comm::MessageView& message);
    bool requestMode();
    bool returnHome();
    void finish(Adafruit_GFX& display, const motion::RunnerEvent& event);
    void renderFailure(Adafruit_GFX& display,
                       const motion::RunnerEvent& event) const;
    void render(Adafruit_GFX& display, const char* status) const;
    void renderRunner(Adafruit_GFX& display);

    comm::Link<16> link_;
    motion::PickPlaceRunner& runner_;
    motion::MoveTask task_;
    uint8_t receivedPieces_;
    uint8_t receivedMask_;
    uint8_t selectedMode_;
    State state_;
    motion::RunnerStage renderedStage_;
    motion::PollDiagnostic renderedPollDiagnostic_;
    uint32_t now_;
    bool exitReady_;
    bool calibrationFinishSent_;
    bool logVisionCoordinates_;
    bool usesVisionUart_;
};

/**
 * @brief 获取菜单共用的搬运 Program
 * @return 搬运 Program 实例
 */
MoveTaskProgram& moveTask();

/**
 * @brief 获取 Host 固定数据 Demo 使用的搬运 Program
 * @return 搬运 Demo Program 实例
 */
MoveTaskProgram& moveTaskDemo();

}
