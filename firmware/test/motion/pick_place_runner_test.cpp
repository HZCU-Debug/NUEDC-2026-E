#include <cassert>

#include <Arduino.h>

#include "RecordingMotor.h"
#include "config/parameters.h"
#include "motion/pick_place_runner.h"

HardwareSerial Serial2;

namespace {

motion::RunnerEvent updateReachedXY(motion::PickPlaceRunner& runner,
                                    RecordingMotor& x, RecordingMotor& y,
                                    uint32_t now) {
    x.state.reached = true;
    y.state.reached = true;
    return runner.update(now);
}

motion::RunnerEvent updateReachedZ(motion::PickPlaceRunner& runner,
                                   RecordingMotor& z, uint32_t now) {
    z.state.reached = true;
    return runner.update(now);
}

motion::RunnerEvent updateReachedXYRotation(motion::PickPlaceRunner& runner,
                                            RecordingMotor& x,
                                            RecordingMotor& y,
                                            RecordingMotor& rotation,
                                            uint32_t now) {
    x.state.reached = true;
    y.state.reached = true;
    rotation.state.reached = true;
    return runner.update(now);
}

}

int main() {
    RecordingMotor x;
    RecordingMotor y;
    RecordingMotor z;
    RecordingMotor rotation;
    motion::Gantry gantry(x, y, z, rotation);
    assert(gantry.begin());

    storage::Calibration calibration;
    calibration.x0 = 10.0f;
    calibration.y0 = 20.0f;
    calibration.x1 = 220.0f;
    calibration.y1 = 317.0f;
    calibration.zDown = 90.0f;
    calibration.paperValid = true;
    calibration.zValid = true;

    motion::MoveTask task;
    task.count = 1;
    task.pieces[0].source = motion::PaperPoint(100, 100);
    task.pieces[0].target = motion::PaperPoint(110, 120);
    task.pieces[0].rotationDegrees = 12.34f;

    motion::PickPlaceRunner runner(gantry);
    assert(runner.start(task, calibration, 0).result == motion::TaskResult::None);
    assert(x.absoluteCount == 1 && y.absoluteCount == 1);
    assert(x.lastDegrees == 10.0f && y.lastDegrees == 20.0f);
    assert(pinValues()[10] == LOW);

    assert(runner.update(100).result == motion::TaskResult::None);
    assert(updateReachedXY(runner, x, y, 200).result == motion::TaskResult::None);
    assert(updateReachedZ(runner, z, 300).result == motion::TaskResult::None);
    assert(pinValues()[10] == HIGH);
    assert(updateReachedZ(runner, z, 400).result == motion::TaskResult::None);
    assert(rotation.absoluteCount == 1 && rotation.relativeCount == 0);
    assert(runner.update(3400).result == motion::TaskResult::None);
    assert(runner.stage() == motion::RunnerStage::TargetXY);
    assert(rotation.absoluteCount == 2 && rotation.relativeCount == 0);
    assert(rotation.lastDegrees == 12.34f);
    assert(pinValues()[10] == HIGH);
    assert(updateReachedXYRotation(runner, x, y, rotation, 3500).result ==
           motion::TaskResult::None);
    assert(updateReachedZ(runner, z, 3600).result == motion::TaskResult::None);
    assert(pinValues()[10] == LOW);
    assert(updateReachedZ(runner, z, 3700).result ==
           motion::TaskResult::Completed);

    assert(runner.returnHome(3800).result == motion::TaskResult::None);
    z.position = 10.0f;
    assert(runner.stage() == motion::RunnerStage::ReturnZ);
    assert(runner.update(3900).result == motion::TaskResult::None);
    assert(runner.stage() == motion::RunnerStage::ReturnZ);
    assert(updateReachedZ(runner, z, 4000).result == motion::TaskResult::None);
    assert(runner.stage() == motion::RunnerStage::ReturnXY);
    assert(updateReachedXYRotation(runner, x, y, rotation, 4100).result ==
           motion::TaskResult::None);
    assert(runner.stage() == motion::RunnerStage::Idle);

    motion::MoveTask invalid;
    assert(runner.start(invalid, calibration, 800).result ==
           motion::TaskResult::InvalidTask);

    calibration.paperValid = false;
    assert(runner.start(task, calibration, 800).result ==
           motion::TaskResult::NotCalibrated);

    calibration.paperValid = true;
    RecordingMotor stalledX;
    RecordingMotor stalledY;
    RecordingMotor stalledZ;
    RecordingMotor stalledRotation;
    motion::Gantry stalledGantry(stalledX, stalledY, stalledZ,
                                 stalledRotation);
    assert(stalledGantry.begin());
    motion::PickPlaceRunner stalledRunner(stalledGantry);
    assert(stalledRunner.start(task, calibration, 0).result ==
           motion::TaskResult::None);
    assert(stalledRunner.update(config::kMotorStageTimeoutMs).result ==
           motion::TaskResult::None);
    assert(stalledRunner.stage() == motion::RunnerStage::SourceXY);
    assert(stalledX.absoluteCount == 2 && stalledY.absoluteCount == 2);
    assert(stalledX.stopCount == 1 && stalledY.stopCount == 1 &&
           stalledZ.stopCount == 1 && stalledRotation.stopCount == 1);
    assert(updateReachedXY(stalledRunner, stalledX, stalledY,
                           config::kMotorStageTimeoutMs + 100)
               .result == motion::TaskResult::None);
    assert(stalledRunner.stage() == motion::RunnerStage::SourceDown);
}
