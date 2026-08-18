#include <algorithm>
#include <cassert>
#include <cstring>
#include <string>

#include "RecordingMotor.h"
#include "comm/link.h"
#include "program/motor_test.h"

HardwareSerial Serial;
HardwareSerial Serial2;

namespace {

void transfer(HardwareSerial& from, HardwareSerial& to) {
    to.receive(from.transmitted);
    from.transmitted.clear();
}

bool displayed(const Adafruit_GFX& display, const char* text) {
    return std::find(display.printed.begin(), display.printed.end(), text) !=
           display.printed.end();
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
    program::MotorTestProgram program(deviceSerial, gantry);
    comm::Link<7> hostLink(hostSerial);
    assert(hostLink.begin());

    runtime::SystemState state;
    Adafruit_GFX display(240, 135);
    program.start(display, state);
    assert(displayed(display, "Ready"));
    assert(!x.enabled && !y.enabled && !z.enabled && !rotation.enabled);

    const uint8_t speed[] = {'X', 0xFE, 0xD4};
    assert(hostLink.send(program::kMotorSpeedMessage, speed, sizeof(speed),
                         comm::Delivery::Reliable) ==
           comm::SendResult::Accepted);
    transfer(hostSerial, deviceSerial);
    program.update(display, state, ui::Event::None);
    assert(displayed(display, "Running"));
    assert(displayed(display, "X:-300 RPM"));
    assert(x.enabled && !y.enabled && !z.enabled && !rotation.enabled);
    assert(x.runCount == 1 && x.lastRpm == -300);
    assert(y.runCount == 0 && z.runCount == 0 && rotation.runCount == 0);
    transfer(deviceSerial, hostSerial);
    assert(hostLink.poll().type == comm::EventType::Delivered);

    const float targetDegrees = 90.0f;
    uint32_t encodedDegrees;
    std::memcpy(&encodedDegrees, &targetDegrees, sizeof(encodedDegrees));
    const uint8_t position[] = {
        'X', static_cast<uint8_t>(encodedDegrees >> 24),
        static_cast<uint8_t>(encodedDegrees >> 16),
        static_cast<uint8_t>(encodedDegrees >> 8),
        static_cast<uint8_t>(encodedDegrees), 0x01, 0x2C};
    assert(hostLink.send(program::kMotorPositionMessage, position,
                         sizeof(position), comm::Delivery::Reliable) ==
           comm::SendResult::Accepted);
    transfer(hostSerial, deviceSerial);
    program.update(display, state, ui::Event::None);
    assert(displayed(display, "X:90.00 deg"));
    assert(x.absoluteCount == 1 && x.lastDegrees == 90.0f);
    assert(x.lastOptions.rpm == 300 && x.lastOptions.acceleration == 100);
    transfer(deviceSerial, hostSerial);
    assert(hostLink.poll().type == comm::EventType::Delivered);

    const uint8_t invalid[] = {'A', 0x01, 0x2C};
    assert(hostLink.send(program::kMotorSpeedMessage, invalid, sizeof(invalid),
                         comm::Delivery::Reliable) ==
           comm::SendResult::Accepted);
    transfer(hostSerial, deviceSerial);
    program.update(display, state, ui::Event::None);
    assert(displayed(display, "Bad command"));
    assert(x.runCount == 1 && y.runCount == 0);
    transfer(deviceSerial, hostSerial);
    assert(hostLink.poll().type == comm::EventType::Delivered);

    const uint8_t stop[] = {'X', 0x00, 0x00};
    assert(hostLink.send(program::kMotorSpeedMessage, stop, sizeof(stop),
                         comm::Delivery::Reliable) ==
           comm::SendResult::Accepted);
    transfer(hostSerial, deviceSerial);
    program.update(display, state, ui::Event::None);
    assert(x.runCount == 2 && x.lastRpm == 0);
    assert(x.stopCount == 0);

    program.requestExit();
    assert(program.readyToExit());
    assert(x.runCount == 3 && x.lastRpm == 0);
    assert(x.stopCount == 0 && y.stopCount == 0 && z.stopCount == 0 &&
           rotation.stopCount == 0);
    program.stop(state);
}
