#include <cassert>
#include <cmath>
#include <initializer_list>
#include <vector>

#include "atk/motor.h"

static void expectBytes(const std::vector<uint8_t>& actual,
                        std::initializer_list<uint8_t> expected) {
    assert(actual == std::vector<uint8_t>(expected));
}

int main() {
    HardwareSerial invalidSerial;
    atk::Bus invalidBus(invalidSerial,
                        atk::BusConfig(115200, 16, 17, 20, 1));
    assert(!invalidBus.begin());

    HardwareSerial serial;
    atk::Bus bus(serial, atk::BusConfig(115200, 16, 17, 20));
    atk::Motor motor(bus, atk::MotorConfig(1, false, 18));
    assert(bus.begin());

    const unsigned long delayBeforeCommand = delayedMilliseconds();
    serial.respondWith({0xC5, 0x01, 0xFA, 0x01, 0x00, 0xC1, 0x5C});
    assert(motor.enable());
    assert(serial.rxPin == 18);
    assert(delayedMilliseconds() - delayBeforeCommand >= 2);
    expectBytes(serial.transmitted, {0xC5, 0x01, 0xFA, 0x00, 0xC0, 0x5C});

    serial.transmitted.clear();
    serial.respondWith({0xC5, 0x01, 0xFA, 0x01, 0x01, 0xC2, 0x5C});
    assert(motor.disable());
    expectBytes(serial.transmitted, {0xC5, 0x01, 0xFA, 0x01, 0xC1, 0x5C});

    serial.transmitted.clear();
    serial.respondWith({0xC5, 0x01, 0xF8, 0x01, 0xBF, 0x5C});
    assert(motor.clearPosition());
    expectBytes(serial.transmitted, {0xC5, 0x01, 0xF8, 0xBE, 0x5C});

    serial.transmitted.clear();
    serial.respondWith({0xC5, 0x01, 0xF3, 0x01, 0xBA, 0x5C});
    assert(motor.moveRelative(-360.0f, atk::MotionOptions(300, 10)));
    expectBytes(serial.transmitted,
                {0xC5, 0x01, 0xF3, 0x01, 0x0A, 0x01, 0x2C, 0x00, 0x00,
                 0xC8, 0x00, 0xB9, 0x5C});
    assert(!motor.moveRelative(0.0f, atk::MotionOptions(300)));
    assert(!motor.moveAbsolute(90.0f, atk::MotionOptions(300, 201)));

    serial.transmitted.clear();
    serial.respondWith({0xC5, 0x01, 0xF1, 0x01, 0xB8, 0x5C});
    assert(motor.run(-300, 10));
    expectBytes(serial.transmitted,
                {0xC5, 0x01, 0xF1, 0x01, 0x0A, 0x43, 0x96, 0x00, 0x00,
                 0x9B, 0x5C});

    serial.transmitted.clear();
    serial.respondWith({0xC5, 0x01, 0xF1, 0x01, 0xB8, 0x5C});
    assert(motor.run(0, 10));
    expectBytes(serial.transmitted,
                {0xC5, 0x01, 0xF1, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,
                 0xC1, 0x5C});

    serial.respondWith({0xC5, 0x01, 0x30, 0x01, 0x01, 0xF8, 0x5C,
                        0xC5, 0x01, 0x2C, 0x01, 0x02, 0xF5, 0x5C});
    serial.respondWith({0xC5, 0x01, 0x2F, 0x01, 0x00, 0xF6, 0x5C});
    serial.respondWith({0xC5, 0x01, 0x30, 0x01, 0x01, 0xF8, 0x5C});
    serial.respondWith({0xC5, 0x01, 0x2D, 0x01, 0x00, 0xF4, 0x5C});
    const atk::Result<atk::MotorState> state = motor.readState();
    assert(state);
    assert(state.value.enabled);
    assert(state.value.reached);
    assert(!state.value.faulted);
    assert(!state.value.stalled);

    serial.respondWith(
        {0xC5, 0x01, 0x2A, 0x01, 0x00, 0x00, 0x64, 0x00, 0x55, 0x5C});
    const atk::Result<float> position = motor.readPositionDegrees();
    assert(position);
    assert(std::fabs(position.value - 180.0f) < 0.001f);

    serial.transmitted.clear();
    serial.respondWith({0xC5, 0x01, 0x2A, 0xE4, 0xD4, 0x5C});
    serial.respondWith(
        {0xC5, 0x01, 0x2A, 0x01, 0x00, 0x00, 0x64, 0x00, 0x55, 0x5C});
    const unsigned long delayBeforeRetry = delayedMilliseconds();
    assert(motor.readPositionDegrees());
    assert(delayedMilliseconds() - delayBeforeRetry >= 7);
    expectBytes(serial.transmitted,
                {0xC5, 0x01, 0x2A, 0xF0, 0x5C,
                 0xC5, 0x01, 0x2A, 0xF0, 0x5C});

    serial.transmitted.clear();
    serial.respondWith({0xC5, 0x01, 0x2A, 0xE6, 0xD6, 0x5C});
    serial.respondWith({0xC5, 0x01, 0x2A, 0xE6, 0xD6, 0x5C});
    serial.respondWith({0xC5, 0x01, 0x2A, 0xE6, 0xD6, 0x5C});
    assert(motor.readPositionDegrees().error == atk::Error::DeviceRejected);
    expectBytes(serial.transmitted,
                {0xC5, 0x01, 0x2A, 0xF0, 0x5C,
                 0xC5, 0x01, 0x2A, 0xF0, 0x5C,
                 0xC5, 0x01, 0x2A, 0xF0, 0x5C});

    serial.transmitted.clear();
    assert(motor.readPositionDegrees().error == atk::Error::Timeout);
    expectBytes(serial.transmitted, {0xC5, 0x01, 0x2A, 0xF0, 0x5C});

    serial.respondWith(
        {0xC5, 0x01, 0x29, 0x01, 0xFE, 0xD4, 0xC2, 0x5C});
    const atk::Result<float> speed = motor.readSpeedRpm();
    assert(speed);
    assert(std::fabs(speed.value + 300.0f) < 0.001f);

    serial.transmitted.clear();
    serial.respondWith({0xC5, 0x01, 0xF1, 0x01, 0xB8, 0x5C});
    assert(motor.stop());
    expectBytes(serial.transmitted,
                {0xC5, 0x01, 0xF1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                 0xB7, 0x5C});

    atk::Motor secondMotor(bus, atk::MotorConfig(2, false, 19));
    serial.respondWith({0xC5, 0x02, 0xFA, 0x01, 0x00, 0xC2, 0x5C});
    assert(secondMotor.enable());
    assert(serial.rxPin == 19);
    serial.respondWith({0xC5, 0x01, 0xFA, 0x01, 0x00, 0xC1, 0x5C});
    assert(motor.enable());
    assert(serial.rxPin == 18);

    return 0;
}
