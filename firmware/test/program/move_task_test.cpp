#include <algorithm>
#include <cassert>
#include <math.h>
#include <string>

#include "RecordingMotor.h"
#include "comm/link.h"
#include "config/parameters.h"
#include "program/move_task.h"

HardwareSerial Serial;
HardwareSerial Serial1;
HardwareSerial Serial2;

namespace {

void transfer(HardwareSerial& from, HardwareSerial& to) {
    to.receive(from.transmitted);
    from.transmitted.clear();
}

bool displayed(const Adafruit_GFX& display, const char* text) {
    for (const std::string& printed : display.printed) {
        if (printed == text) {
            return true;
        }
    }
    return false;
}

void updateReachedXY(program::MoveTaskProgram& program,
                     runtime::SystemState& state, RecordingMotor& x,
                     RecordingMotor& y,
                     Adafruit_GFX& display) {
    x.state.reached = true;
    y.state.reached = true;
    state.now += 100;
    program.update(display, state, ui::Event::None);
}

void updateReachedZ(program::MoveTaskProgram& program,
                    runtime::SystemState& state, RecordingMotor& z,
                    Adafruit_GFX& display) {
    z.state.reached = true;
    state.now += 100;
    program.update(display, state, ui::Event::None);
}

void updateReachedXYRotation(program::MoveTaskProgram& program,
                             runtime::SystemState& state,
                             RecordingMotor& x, RecordingMotor& y,
                             RecordingMotor& rotation,
                             Adafruit_GFX& display) {
    x.state.reached = true;
    y.state.reached = true;
    rotation.state.reached = true;
    state.now += 100;
    program.update(display, state, ui::Event::None);
}

}

int main() {
    HardwareSerial deviceSerial;
    HardwareSerial hostSerial;
    RecordingMotor x;
    RecordingMotor y;
    RecordingMotor z;
    RecordingMotor rotation;
    motion::Gantry gantry(x, y, z, rotation);
    assert(gantry.begin());
    motion::PickPlaceRunner runner(gantry);
    program::MoveTaskProgram program(deviceSerial, runner,
                                     comm::LinkConfig(), true);
    HardwareSerial demoSerial;
    HardwareSerial demoHostSerial;
    program::MoveTaskProgram demoProgram(demoSerial, runner);

    runtime::SystemState state;
    state.calibration.x1 = 210.0f;
    state.calibration.y1 = 297.0f;
    state.calibration.zDown = 90.0f;
    state.calibration.paperValid = true;
    state.calibration.zValid = true;
    Adafruit_GFX display(240, 135);
    pinValues()[config::kPins.buzzer] = HIGH;
    const unsigned long delayBeforeCalibration = delayedMilliseconds();

    comm::Link<16> demoHostLink(demoHostSerial);
    assert(demoHostLink.begin());

    demoProgram.configure(0);
    demoProgram.start(display, state);
    transfer(demoSerial, demoHostSerial);
    const comm::Event calibrationStart = demoHostLink.poll();
    assert(calibrationStart.type == comm::EventType::Message);
    assert(calibrationStart.message.type == program::kModeRequestMessage);
    assert(calibrationStart.message.payload[0] == 0);
    transfer(demoHostSerial, demoSerial);
    demoProgram.update(display, state, ui::Event::None);
    assert(displayed(display, "S3 Finish"));

    demoProgram.update(display, state, ui::Event::Select);
    assert(displayed(display, "Sending finish"));
    transfer(demoSerial, demoHostSerial);
    const comm::Event calibrationFinish = demoHostLink.poll();
    assert(calibrationFinish.type == comm::EventType::Message);
    assert(calibrationFinish.message.type == program::kModeRequestMessage);
    assert(calibrationFinish.message.payload[0] == 0);
    transfer(demoHostSerial, demoSerial);
    demoProgram.update(display, state, ui::Event::None);
    assert(displayed(display, "Return Z 0"));
    assert(!demoProgram.readyToExit());
    updateReachedZ(demoProgram, state, z, display);
    assert(displayed(display, "Return XY 0,0"));
    updateReachedXYRotation(demoProgram, state, x, y, rotation, display);
    assert(displayed(display, "At origin"));
    assert(demoProgram.readyToExit());
    assert(pinValues()[config::kPins.buzzer] == HIGH);
    assert(delayedMilliseconds() == delayBeforeCalibration);
    demoProgram.stop(state);

    demoProgram.configure(1);
    demoProgram.start(display, state);
    assert(displayed(display, "Sending mode"));
    transfer(demoSerial, demoHostSerial);
    const comm::Event demoModeRequest = demoHostLink.poll();
    assert(demoModeRequest.type == comm::EventType::Message);
    assert(demoModeRequest.message.type == program::kModeRequestMessage);
    assert(demoModeRequest.message.payload[0] == 1);

    transfer(demoHostSerial, demoSerial);
    demoProgram.update(display, state, ui::Event::None);
    const uint8_t demoCount[] = {1};
    assert(demoHostLink.send(program::kPieceCountMessage, demoCount,
                             sizeof(demoCount), comm::Delivery::Reliable) ==
           comm::SendResult::Accepted);
    transfer(demoHostSerial, demoSerial);
    demoProgram.update(display, state, ui::Event::None);
    transfer(demoSerial, demoHostSerial);
    assert(demoHostLink.poll().type == comm::EventType::Delivered);
    const uint8_t demoPiece[] = {
        1,
        0x00, 0x0A, 0x00, 0x14,
        0x00, 0x1E, 0x00, 0x28,
        0x00, 0x00,
    };
    assert(demoHostLink.send(program::kPieceDataMessage, demoPiece,
                             sizeof(demoPiece), comm::Delivery::Reliable) ==
           comm::SendResult::Accepted);
    transfer(demoHostSerial, demoSerial);
    demoProgram.update(display, state, ui::Event::None);
    z.nextStatus = motor::Status(motor::Error::Timeout);
    updateReachedXY(demoProgram, state, x, y, display);
    assert(runner.stage() == motion::RunnerStage::SourceDown);
    z.nextStatus = motor::Status();
    state.now += config::kMotorStageTimeoutMs;
    demoProgram.update(display, state, ui::Event::None);
    updateReachedZ(demoProgram, state, z, display);
    assert(runner.stage() == motion::RunnerStage::SourceUp);
    demoProgram.stop(state);

    comm::Link<16> hostLink(hostSerial);
    assert(hostLink.begin());
    program.configure(2);
    program.start(display, state);
    assert(displayed(display, "Sending mode"));

    transfer(deviceSerial, hostSerial);
    const comm::Event modeRequest = hostLink.poll();
    assert(modeRequest.type == comm::EventType::Message);
    assert(modeRequest.message.type == program::kModeRequestMessage);
    assert(modeRequest.message.delivery == comm::Delivery::Reliable);
    assert(modeRequest.message.size == 1);
    assert(modeRequest.message.payload[0] == 2);
    transfer(hostSerial, deviceSerial);
    program.update(display, state, ui::Event::None);
    assert(displayed(display, "Waiting vision"));
    assert(deviceSerial.transmitted.empty());

    const uint8_t count[] = {1};
    assert(hostLink.send(program::kPieceCountMessage, count, sizeof(count),
                         comm::Delivery::Reliable) ==
           comm::SendResult::Accepted);
    transfer(hostSerial, deviceSerial);
    program.update(display, state, ui::Event::None);
    assert(displayed(display, "Waiting pieces"));
    assert(runner.stage() == motion::RunnerStage::Idle);
    transfer(deviceSerial, hostSerial);
    assert(hostLink.poll().type == comm::EventType::Delivered);

    const uint8_t piece[] = {
        1,
        0x00, 0x0A, 0x00, 0x14,
        0x00, 0x1E, 0x00, 0x28,
        0xFB, 0x2E,
    };
    assert(hostLink.send(program::kPieceDataMessage, piece, sizeof(piece),
                         comm::Delivery::Reliable) ==
           comm::SendResult::Accepted);
    transfer(hostSerial, deviceSerial);
    program.update(display, state, ui::Event::None);
    assert(displayed(display, "Piece 1/1"));
    assert(displayed(display, "Source XY"));
    const std::string visionLog(Serial.transmitted.begin(),
                                Serial.transmitted.end());
    assert(visionLog.find("VISION id=1 S=(10,20)mm T=(30,40)mm\n") !=
           std::string::npos);
    transfer(deviceSerial, hostSerial);
    assert(hostLink.poll().type == comm::EventType::Delivered);

    state.now += 100;
    program.update(display, state, ui::Event::None);
    updateReachedXY(program, state, x, y, display);
    assert(displayed(display, "Source Z down"));
    updateReachedZ(program, state, z, display);
    assert(displayed(display, "Source Z up"));
    updateReachedZ(program, state, z, display);
    assert(displayed(display, "Target XY"));
    assert(rotation.relativeCount == 0 &&
           fabsf(rotation.lastDegrees + 12.34f) < 0.001f);
    updateReachedXYRotation(program, state, x, y, rotation, display);
    assert(displayed(display, "Target Z down"));
    updateReachedZ(program, state, z, display);
    assert(displayed(display, "Target Z up"));
    updateReachedZ(program, state, z, display);
    assert(displayed(display, "Return Z 0"));
    assert(!program.readyToExit());
    assert(deviceSerial.transmitted.empty());

    z.state.reached = true;
    state.now += 100;
    program.update(display, state, ui::Event::None);
    assert(!program.readyToExit());
    assert(displayed(display, "Return XY 0,0"));
    assert(rotation.lastDegrees == 0.0f);

    x.state.reached = true;
    y.state.reached = true;
    rotation.state.reached = true;
    state.now += 100;
    const unsigned long delayBeforeCompletion = delayedMilliseconds();
    program.update(display, state, ui::Event::None);
    assert(program.readyToExit());
    assert(displayed(display, "At origin"));
    assert(pinValues()[config::kPins.buzzer] == HIGH);
    assert(delayedMilliseconds() ==
           delayBeforeCompletion + config::kBuzzerDurationMs);

    program.requestExit();
    assert(program.readyToExit());

    program.stop(state);
    program.configure(3);
    program.start(display, state);
    transfer(deviceSerial, hostSerial);
    const comm::Event nextModeRequest = hostLink.poll();
    assert(nextModeRequest.type == comm::EventType::Message);
    assert(nextModeRequest.message.type == program::kModeRequestMessage);
    assert(nextModeRequest.message.payload[0] == 3);
}
