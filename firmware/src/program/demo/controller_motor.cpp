/**
 * @file controller_motor.cpp
 * @brief 接收 Python 手柄发送的速度消息并驱动电机持续旋转
 */
#include "program/programs.h"

#include <Arduino.h>

#include "config/hardware.h"
#include "comm/link.h"
#include "ui/view.h"
#include "zdt/motor.h"

namespace program {
namespace {

const uint32_t kSerialBaudRate = 115200;
const uint8_t kMotorAddress = 1;
const uint32_t kPulsesPerRevolution = 3200;
const uint16_t kMaximumMotorRpm = 300;
const uint8_t kAcceleration = 0;
const uint32_t kCommandTimeoutMs = 500;
const uint8_t kVelocityMessage = 1;

class ControllerMotorProgram final : public runtime::Program {
public:
    ControllerMotorProgram()
        : motorBus_(Serial2,
                    zdt::BusConfig(kSerialBaudRate, config::kPins.motorRx,
                                   config::kPins.motorTx)),
          motor_(motorBus_,
                 zdt::MotorConfig(kMotorAddress, kPulsesPerRevolution)),
          controllerLink_(Serial, comm::LinkConfig(kSerialBaudRate)),
          motorReady_(false),
          motorRunning_(false),
          targetRpm_(0),
          lastCommandAt_(0) {}

    void start(Adafruit_GFX& display, runtime::SystemState&) override {
        controllerLink_.begin();
        zdt::Status status = motorBus_.begin();
        if (status) {
            delay(2000);
            status = motor_.enable();
        }
        if (status) {
            const zdt::Result<float> position = motor_.readPositionDegrees();
            status = zdt::Status(position.error);
        }
        motorReady_ = static_cast<bool>(status);
        motorRunning_ = false;
        targetRpm_ = 0;
        lastCommandAt_ = millis();
        ui::view::beginPage(display, "Controller Motor");
        render(display);
    }

    void update(Adafruit_GFX& display, runtime::SystemState&,
                ui::Event) override {
        bool changed = readCommands();
        if (motorRunning_ && millis() - lastCommandAt_ >= kCommandTimeoutMs) {
            motor_.stop();
            motorRunning_ = false;
            targetRpm_ = 0;
            changed = true;
        }
        if (changed) {
            render(display);
        }
    }

    void stop(runtime::SystemState&) override {
        controllerLink_.cancel();
        motor_.stop();
        motorRunning_ = false;
        targetRpm_ = 0;
    }

private:
    bool decodeVelocity(const uint8_t* payload, size_t size,
                        int16_t& velocity) const {
        if (payload == nullptr || size != 2) {
            return false;
        }

        const uint16_t encoded =
            static_cast<uint16_t>(static_cast<uint16_t>(payload[0]) << 8) |
            payload[1];
        const int32_t decoded =
            encoded <= INT16_MAX ? encoded : static_cast<int32_t>(encoded) - 0x10000L;
        if (decoded < -1000 || decoded > 1000) {
            return false;
        }

        velocity = static_cast<int16_t>(decoded);
        return true;
    }

    bool applyCommand(const comm::MessageView& message) {
        int16_t velocity = 0;
        if (!motorReady_ || message.type != kVelocityMessage ||
            !decodeVelocity(message.payload, message.size, velocity)) {
            return false;
        }

        const int16_t rpm = static_cast<int16_t>(
            static_cast<int32_t>(velocity) * kMaximumMotorRpm / 1000);
        const zdt::Status status =
            rpm == 0 ? motor_.stop() : motor_.run(rpm, kAcceleration);
        if (!status) {
            return false;
        }
        motorRunning_ = rpm != 0;
        targetRpm_ = rpm;
        lastCommandAt_ = millis();
        return true;
    }

    bool readCommands() {
        bool changed = false;
        for (;;) {
            const comm::Event event = controllerLink_.poll();
            if (event.type == comm::EventType::None) {
                return changed;
            }
            if (event.type == comm::EventType::Message) {
                changed = applyCommand(event.message) || changed;
            }
        }
    }

    void render(Adafruit_GFX& display) const {
        ui::view::beginBody(display);
        display.setCursor(6, 42);
        display.print(motorReady_ ? "Motor ready" : "Motor error");
        display.setCursor(6, 68);
        display.print("RPM: ");
        display.print(targetRpm_);
    }

    zdt::Bus motorBus_;
    zdt::Motor motor_;
    comm::Link<2> controllerLink_;
    bool motorReady_;
    bool motorRunning_;
    int16_t targetRpm_;
    uint32_t lastCommandAt_;
};

ControllerMotorProgram program;

}

runtime::Program& controllerMotor() { return program; }

}
