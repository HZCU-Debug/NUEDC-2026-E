#include "motion/pick_place_runner.h"

#include <Arduino.h>

#include "config/hardware.h"
#include "config/parameters.h"

namespace motion {
namespace {

float mapCoordinate(uint16_t coordinate, uint16_t referenceStart,
                    uint16_t referenceEnd, float start, float end) {
    return start + (static_cast<float>(coordinate) - referenceStart) /
                       (referenceEnd - referenceStart) * (end - start);
}

}

XYPosition mapPaperPoint(const PaperPoint& point,
                         const storage::Calibration& calibration) {
    return XYPosition(
        mapCoordinate(point.x, config::kCalibrationStart, config::kPaperWidth,
                      calibration.x0, calibration.x1),
        mapCoordinate(point.y, config::kCalibrationStart, config::kPaperHeight,
                      calibration.y0, calibration.y1));
}

PickPlaceRunner::PickPlaceRunner(Gantry& gantry)
    : gantry_(gantry),
      task_(),
      calibration_(),
      stage_(RunnerStage::Idle),
      pieceIndex_(0),
      lastPollAt_(0),
      stageStartedAt_(0),
      targetRotation_(0.0f),
      commandIssued_(false) {}

RunnerEvent PickPlaceRunner::start(
    const MoveTask& task, const storage::Calibration& calibration,
    uint32_t now) {
    if (stage_ != RunnerStage::Idle || task.count == 0 || task.count > 4) {
        return RunnerEvent(TaskResult::InvalidTask);
    }
    if (!calibration.paperValid || !calibration.zValid) {
        return RunnerEvent(TaskResult::NotCalibrated);
    }
    if (!valid(task)) {
        return RunnerEvent(TaskResult::InvalidTask);
    }

    task_ = task;
    calibration_ = calibration;
    pieceIndex_ = 0;
    targetRotation_ = 0.0f;
    lastPollAt_ = now;
    stage_ = RunnerStage::SourceXY;
    stageStartedAt_ = now;
    commandIssued_ = gantry_.setEnabled() && issueCurrentStage();
    return commandIssued_ ? RunnerEvent() : retryCurrentStage(now);
}

RunnerEvent PickPlaceRunner::update(uint32_t now) {
    if (stage_ == RunnerStage::Idle ||
        now - lastPollAt_ < config::kMotorPollIntervalMs) {
        return RunnerEvent();
    }
    lastPollAt_ = now;
    if (!commandIssued_) {
        return retryCurrentStage(now);
    }

    Arrival arrival;
    if (stage_ == RunnerStage::TargetXY || stage_ == RunnerStage::ReturnXY) {
        arrival = gantry_.pollXYRotation();
    } else if (stage_ == RunnerStage::SourceXY) {
        arrival = gantry_.pollXY();
    } else {
        arrival = gantry_.pollZ();
    }
    if (arrival == Arrival::Reached) {
        return advance(now);
    }
    return now - stageStartedAt_ >= config::kMotorStageTimeoutMs
               ? retryCurrentStage(now)
               : RunnerEvent();
}

RunnerEvent PickPlaceRunner::returnHome(uint32_t now) {
    digitalWrite(config::kPins.electromagnet, LOW);
    gantry_.stop();
    lastPollAt_ = now;
    stage_ = RunnerStage::ReturnZ;
    stageStartedAt_ = now;
    commandIssued_ = gantry_.setEnabled() && issueCurrentStage();
    return commandIssued_ ? RunnerEvent() : retryCurrentStage(now);
}

RunnerStage PickPlaceRunner::stage() const { return stage_; }

uint8_t PickPlaceRunner::pieceIndex() const { return pieceIndex_; }

uint8_t PickPlaceRunner::pieceCount() const { return task_.count; }

PollDiagnostic PickPlaceRunner::pollDiagnostic() const {
    return gantry_.pollDiagnostic();
}

void PickPlaceRunner::stop() {
    digitalWrite(config::kPins.electromagnet, LOW);
    gantry_.stop();
    stage_ = RunnerStage::Idle;
    commandIssued_ = false;
}

bool PickPlaceRunner::valid(const MoveTask& task) const {
    for (uint8_t index = 0; index < task.count; ++index) {
        const PieceMove& piece = task.pieces[index];
        if (piece.source.x > config::kPaperWidth ||
            piece.target.x > config::kPaperWidth ||
            piece.source.y > config::kPaperHeight ||
            piece.target.y > config::kPaperHeight) {
            return false;
        }
    }
    return true;
}

motor::Status PickPlaceRunner::moveXY(const PaperPoint& point) {
    const XYPosition target = mapPaperPoint(point, calibration_);
    return gantry_.moveXY(
        target.x, target.y,
        motor::MotionOptions(config::kXyMotionRpm,
                             config::kXyMotionAcceleration));
}

motor::Status PickPlaceRunner::moveZ(float degrees) {
    return gantry_.moveZ(
        degrees, motor::MotionOptions(config::kZMotionRpm,
                                      config::kZMotionAcceleration));
}

motor::Status PickPlaceRunner::moveTarget(const PieceMove& piece) {
    const XYPosition target = mapPaperPoint(piece.target, calibration_);
    return gantry_.moveXYRotation(
        target.x, target.y, targetRotation_,
        motor::MotionOptions(config::kXyMotionRpm,
                             config::kXyMotionAcceleration),
        motor::MotionOptions(config::kRollMotionRpm,
                             config::kRollMotionAcceleration));
}

motor::Status PickPlaceRunner::issueCurrentStage() {
    const bool carrying = stage_ == RunnerStage::SourceUp ||
                          stage_ == RunnerStage::TargetXY ||
                          stage_ == RunnerStage::TargetDown;
    digitalWrite(config::kPins.electromagnet, carrying ? HIGH : LOW);
    switch (stage_) {
        case RunnerStage::SourceXY:
            return moveXY(task_.pieces[pieceIndex_].source);
        case RunnerStage::SourceDown:
        case RunnerStage::TargetDown:
            return moveZ(calibration_.zDown);
        case RunnerStage::SourceUp:
        case RunnerStage::TargetUp:
            return moveZ(0.0f);
        case RunnerStage::TargetXY:
            return moveTarget(task_.pieces[pieceIndex_]);
        case RunnerStage::ReturnZ:
            return moveZ(0.0f);
        case RunnerStage::ReturnXY:
            return gantry_.moveXYRotation(
                0.0f, 0.0f, 0.0f,
                motor::MotionOptions(config::kXyMotionRpm,
                                     config::kXyMotionAcceleration),
                motor::MotionOptions(config::kRollMotionRpm,
                                     config::kRollMotionAcceleration));
        case RunnerStage::Idle:
            return motor::Status();
    }
    return motor::Status(motor::Error::InvalidArgument);
}

RunnerEvent PickPlaceRunner::advance(uint32_t now) {
    switch (stage_) {
        case RunnerStage::SourceXY:
            stage_ = RunnerStage::SourceDown;
            break;
        case RunnerStage::SourceDown:
            stage_ = RunnerStage::SourceUp;
            break;
        case RunnerStage::SourceUp:
            targetRotation_ += task_.pieces[pieceIndex_].rotationDegrees;
            stage_ = RunnerStage::TargetXY;
            break;
        case RunnerStage::TargetXY:
            stage_ = RunnerStage::TargetDown;
            break;
        case RunnerStage::TargetDown:
            stage_ = RunnerStage::TargetUp;
            break;
        case RunnerStage::TargetUp:
            ++pieceIndex_;
            if (pieceIndex_ == task_.count) {
                stage_ = RunnerStage::Idle;
                commandIssued_ = false;
                return RunnerEvent(TaskResult::Completed, 0xFF);
            }
            stage_ = RunnerStage::SourceXY;
            break;
        case RunnerStage::ReturnZ:
            stage_ = RunnerStage::ReturnXY;
            break;
        case RunnerStage::ReturnXY:
            stage_ = RunnerStage::Idle;
            commandIssued_ = false;
            return RunnerEvent();
        case RunnerStage::Idle:
            return RunnerEvent();
    }
    commandIssued_ = static_cast<bool>(issueCurrentStage());
    if (!commandIssued_) {
        return retryCurrentStage(now);
    }
    stageStartedAt_ = now;
    return RunnerEvent();
}

RunnerEvent PickPlaceRunner::retryCurrentStage(uint32_t now) {
    // ponytail: 持续故障会重复当前阶段，需要跳过物块时再增加重试上限
    gantry_.stop();
    lastPollAt_ = now;
    commandIssued_ = gantry_.setEnabled() && issueCurrentStage();
    if (commandIssued_) {
        stageStartedAt_ = now;
    }
    return RunnerEvent();
}

}
