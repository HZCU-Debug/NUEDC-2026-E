#pragma once

#include <stdint.h>

#include "motion/gantry.h"
#include "storage/calibration.h"

namespace motion {

/**
 * @brief A4 纸面坐标，单位为 mm
 *
 * X 沿 A4 短边，Y 沿 A4 长边
 */
struct PaperPoint {
    PaperPoint() : x(0), y(0) {}
    PaperPoint(uint16_t x, uint16_t y) : x(x), y(y) {}

    /** A4 短边方向坐标 */
    uint16_t x;
    /** A4 长边方向坐标 */
    uint16_t y;
};

/**
 * @brief 一个物块的原坐标和目标坐标
 */
struct PieceMove {
    /** 原坐标 */
    PaperPoint source;
    /** 目标坐标 */
    PaperPoint target;
    /** 从源姿态到目标姿态的相对旋转角度，顺时针为正 */
    float rotationDegrees;
};

/**
 * @brief 一次最多包含四个物块的搬运任务
 */
struct MoveTask {
    MoveTask() : count(0), pieces() {}

    /** 有效物块数量 */
    uint8_t count;
    /** 按执行顺序排列的物块 */
    PieceMove pieces[4];
};

/**
 * @brief 将 A4 纸面坐标换算为电机角度
 * @param point A4 纸面坐标
 * @param calibration 纸面标定结果
 * @return X 和 Y 电机的绝对目标角度
 */
XYPosition mapPaperPoint(const PaperPoint& point,
                         const storage::Calibration& calibration);

/**
 * @brief 搬运任务结果码
 */
enum class TaskResult : uint8_t {
    Completed = 0,
    InvalidTask = 1,
    NotCalibrated = 2,
    None = 0xFF,
};

/**
 * @brief 搬运状态机产生的一次事件
 */
struct RunnerEvent {
    RunnerEvent(TaskResult result = TaskResult::None, uint8_t pieceIndex = 0xFF)
        : result(result), pieceIndex(pieceIndex) {}

    /** 本次结果，没有事件时为 TaskResult::None */
    TaskResult result;
    /** 相关物块序号，没有具体物块时为 0xFF */
    uint8_t pieceIndex;
};

/**
 * @brief 搬运状态机当前阶段
 */
enum class RunnerStage : uint8_t {
    /** 没有活动运动 */
    Idle,
    /** 前往物块原坐标 */
    SourceXY,
    /** 原坐标 Z 轴下降 */
    SourceDown,
    /** 原坐标 Z 轴回升 */
    SourceUp,
    /** 前往物块目标坐标 */
    TargetXY,
    /** 目标坐标 Z 轴下降 */
    TargetDown,
    /** 目标坐标 Z 轴回升 */
    TargetUp,
    /** 退出时 Z 轴回零 */
    ReturnZ,
    /** 退出时 X 和 Y 轴回零 */
    ReturnXY,
};

/**
 * @brief 依次执行物块源点和目标点三轴动作
 */
class PickPlaceRunner {
public:
    /**
     * @brief 绑定共享三轴机构
     * @param gantry 三轴机构
     */
    explicit PickPlaceRunner(Gantry& gantry);

    /**
     * @brief 校验并启动一次搬运任务
     * @param task 搬运任务
     * @param calibration 坐标和 Z 轴校准结果
     * @param now 当前主循环时间
     * @return 启动错误或无事件
     */
    RunnerEvent start(const MoveTask& task,
                      const storage::Calibration& calibration, uint32_t now);

    /**
     * @brief 查询到位状态并推进搬运任务
     * @param now 当前主循环时间
     * @return 完成或无事件，运动异常会自动重试当前阶段
     */
    RunnerEvent update(uint32_t now);

    /**
     * @brief 停止当前动作并开始按 Z、XY 顺序回到电机零点
     * @param now 当前主循环时间
     * @return 无事件，运动异常会自动重试当前阶段
     */
    RunnerEvent returnHome(uint32_t now);

    /**
     * @brief 获取当前运动阶段
     * @return 当前运动阶段
     */
    RunnerStage stage() const;

    /**
     * @brief 获取当前物块序号
     * @return 从 0 开始的物块序号
     */
    uint8_t pieceIndex() const;

    /**
     * @brief 获取任务中的物块数量
     * @return 物块数量
     */
    uint8_t pieceCount() const;

    /**
     * @brief 获取最近一次轴状态查询的诊断信息
     * @return 最近一次查询诊断信息
     */
    PollDiagnostic pollDiagnostic() const;

    /**
     * @brief 停止三轴并清空任务
     */
    void stop();

private:
    bool valid(const MoveTask& task) const;
    motor::Status moveXY(const PaperPoint& point);
    motor::Status moveTarget(const PieceMove& piece);
    motor::Status moveZ(float degrees);
    motor::Status issueCurrentStage();
    RunnerEvent advance(uint32_t now);
    RunnerEvent retryCurrentStage(uint32_t now);

    Gantry& gantry_;
    MoveTask task_;
    storage::Calibration calibration_;
    RunnerStage stage_;
    uint8_t pieceIndex_;
    uint32_t lastPollAt_;
    uint32_t stageStartedAt_;
    float targetRotation_;
    bool commandIssued_;
};

}
