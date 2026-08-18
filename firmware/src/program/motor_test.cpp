#include "program/motor_test.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "config/hardware.h"
#include "config/parameters.h"
#include "ui/view.h"

extern HardwareSerial Serial;

namespace program {
namespace {

int16_t decodeInt16(const uint8_t* data) {
    const uint16_t encoded =
        static_cast<uint16_t>(static_cast<uint16_t>(data[0]) << 8) | data[1];
    return encoded <= INT16_MAX
               ? static_cast<int16_t>(encoded)
               : static_cast<int16_t>(static_cast<int32_t>(encoded) - 0x10000L);
}

float decodeFloat(const uint8_t* data) {
    const uint32_t encoded = static_cast<uint32_t>(data[0]) << 24 |
                             static_cast<uint32_t>(data[1]) << 16 |
                             static_cast<uint32_t>(data[2]) << 8 | data[3];
    float value;
    memcpy(&value, &encoded, sizeof(value));
    return value;
}

bool decodeAxis(uint8_t value, motion::Axis& axis) {
    switch (value) {
        case 'X':
            axis = motion::Axis::X;
            return true;
        case 'Y':
            axis = motion::Axis::Y;
            return true;
        case 'Z':
            axis = motion::Axis::Z;
            return true;
        case 'R':
            axis = motion::Axis::Rotation;
            return true;
        default:
            return false;
    }
}

}

MotorTestProgram::MotorTestProgram(
    HardwareSerial& serial, motion::Gantry& gantry,
    const comm::LinkConfig& config)
    : link_(serial, config),
      gantry_(gantry),
      state_(State::Idle),
      axis_('-'),
      rpm_(0),
      degrees_(0.0f),
      positionMode_(false),
      exitReady_(true) {}

void MotorTestProgram::start(Adafruit_GFX& display, runtime::SystemState&) {
    ui::view::beginPage(display, "Test");
    digitalWrite(config::kPins.electromagnet, LOW);
    axis_ = '-';
    rpm_ = 0;
    degrees_ = 0.0f;
    positionMode_ = false;
    exitReady_ = true;
    if (!link_.begin()) {
        state_ = State::Error;
        render(display, "Start failed");
        return;
    }
    state_ = State::Ready;
    render(display, "Ready");
}

void MotorTestProgram::update(Adafruit_GFX& display, runtime::SystemState&,
                              ui::Event) {
    if (state_ != State::Ready) return;
    const comm::Event event = link_.poll();
    if (event.type != comm::EventType::Message) return;
    const bool accepted =
        applySpeed(event.message) || applyPosition(event.message);
    render(display, accepted ? (positionMode_ ? "Moving" : "Running")
                             : "Bad command");
}

void MotorTestProgram::requestExit() {
    link_.cancel();
    digitalWrite(config::kPins.electromagnet, LOW);
    motion::Axis axis;
    if (decodeAxis(static_cast<uint8_t>(axis_), axis)) {
        gantry_.run(axis, 0, config::kMotorTestAcceleration);
    }
    state_ = State::Idle;
    exitReady_ = true;
}

bool MotorTestProgram::readyToExit() const { return exitReady_; }

void MotorTestProgram::stop(runtime::SystemState&) {
    requestExit();
}

bool MotorTestProgram::applySpeed(const comm::MessageView& message) {
    if (message.type != kMotorSpeedMessage ||
        message.delivery != comm::Delivery::Reliable || message.size != 3) {
        return false;
    }
    motion::Axis axis;
    const int16_t rpm = decodeInt16(message.payload + 1);
    if (!decodeAxis(message.payload[0], axis) ||
        rpm < -config::kMotorTestMaximumRpm ||
        rpm > config::kMotorTestMaximumRpm || !gantry_.setEnabled(axis) ||
        !gantry_.run(axis, rpm, config::kMotorTestAcceleration)) {
        return false;
    }
    axis_ = static_cast<char>(message.payload[0]);
    rpm_ = rpm;
    positionMode_ = false;
    return true;
}

bool MotorTestProgram::applyPosition(const comm::MessageView& message) {
    if (message.type != kMotorPositionMessage ||
        message.delivery != comm::Delivery::Reliable || message.size != 7) {
        return false;
    }
    motion::Axis axis;
    const float degrees = decodeFloat(message.payload + 1);
    const uint16_t rpm = static_cast<uint16_t>(message.payload[5]) << 8 |
                         message.payload[6];
    if (!decodeAxis(message.payload[0], axis) || !isfinite(degrees) ||
        rpm == 0 ||
        rpm > static_cast<uint16_t>(config::kMotorTestMaximumRpm) ||
        !gantry_.setEnabled(axis) ||
        !gantry_.moveAbsolute(axis, degrees,
                              motor::MotionOptions(
                                  rpm, config::kMotorTestAcceleration))) {
        return false;
    }
    axis_ = static_cast<char>(message.payload[0]);
    rpm_ = static_cast<int16_t>(rpm);
    degrees_ = degrees;
    positionMode_ = true;
    return true;
}

void MotorTestProgram::render(Adafruit_GFX& display,
                              const char* status) const {
    ui::view::beginBody(display);
    display.setCursor(6, 40);
    display.print(status);
    char command[24];
    if (positionMode_) {
        snprintf(command, sizeof(command), "%c:%.2f deg", axis_, degrees_);
    } else {
        snprintf(command, sizeof(command), "%c:%d RPM", axis_, rpm_);
    }
    display.setCursor(6, 70);
    display.print(command);
    display.setCursor(6, 106);
    display.print("S4 Back");
}

runtime::Program& motorTest() {
    static MotorTestProgram program(Serial, motion::systemGantry());
    return program;
}

}
