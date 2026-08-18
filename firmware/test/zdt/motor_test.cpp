#include <cassert>
#include <cmath>
#include <vector>

#include "zdt/motor.h"

static void expectBytes(const std::vector<uint8_t>& actual,
                        std::initializer_list<uint8_t> expected) {
    assert(actual == std::vector<uint8_t>(expected));
}

int main() {
    HardwareSerial serial;
    zdt::Bus bus(serial, zdt::BusConfig(115200, 16, 17, 20));
    zdt::Motor motor(bus, zdt::MotorConfig(1, 3200));

    assert(bus.begin());

    const unsigned long delayBeforeCommand = delayedMilliseconds();
    assert(motor.enable());
    assert(delayedMilliseconds() - delayBeforeCommand >= 10);
    expectBytes(serial.transmitted, {0x01, 0xF3, 0xAB, 0x01, 0x00, 0x6B});

    serial.transmitted.clear();
    assert(motor.disable());
    expectBytes(serial.transmitted, {0x01, 0xF3, 0xAB, 0x00, 0x00, 0x6B});

    serial.transmitted.clear();
    assert(motor.clearPosition());
    expectBytes(serial.transmitted, {0x01, 0x0A, 0x6D, 0x6B});

    serial.transmitted.clear();
    assert(motor.moveAbsolute(0.0f, zdt::MotionOptions(300)));
    expectBytes(serial.transmitted,
                {0x01, 0xFD, 0x00, 0x01, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00,
                 0x01, 0x00, 0x6B});
    assert(!motor.moveAbsolute(0.01f, zdt::MotionOptions(300)));

    serial.transmitted.clear();
    assert(motor.moveRelative(-90.0f, zdt::MotionOptions(300, 10, zdt::Start::Synchronized)));
    expectBytes(serial.transmitted,
                {0x01, 0xFD, 0x01, 0x01, 0x2C, 0x0A, 0x00, 0x00, 0x03, 0x20,
                 0x00, 0x01, 0x6B});

    serial.transmitted.clear();
    assert(bus.clearPositions());
    expectBytes(serial.transmitted, {0x00, 0x0A, 0x6D, 0x6B});

    serial.transmitted.clear();
    assert(bus.enableAll(false));
    expectBytes(serial.transmitted, {0x00, 0xF3, 0xAB, 0x00, 0x00, 0x6B});

    serial.transmitted.clear();
    assert(bus.enableAll());
    expectBytes(serial.transmitted, {0x00, 0xF3, 0xAB, 0x01, 0x00, 0x6B});

    serial.transmitted.clear();
    serial.respondWith({0x01, 0x3A, 0x03, 0x6B});
    serial.respondWith({0x01, 0x3B, 0x03, 0x6B});
    const zdt::Result<zdt::MotorState> state = motor.readState();
    assert(state);
    assert(state.value.enabled);
    assert(state.value.reached);
    assert(!state.value.stalled);
    assert(!state.value.homing);
    expectBytes(serial.transmitted, {0x01, 0x3A, 0x6B, 0x01, 0x3B, 0x6B});

    serial.transmitted.clear();
    serial.respondWith({0x01, 0xF3, 0x02, 0x6B,
                        0x01, 0x36, 0x00, 0x00, 0x00, 0x40, 0x00, 0x6B});
    const zdt::Result<float> position = motor.readPositionDegrees();
    assert(position);
    assert(std::fabs(position.value - 90.0f) < 0.001f);

    return 0;
}
