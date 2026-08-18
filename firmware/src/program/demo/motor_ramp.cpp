/**
 * @file motor_ramp.cpp
 * @brief 驱动电机以 10 RPM 为步长在 -300 至 300 RPM 之间每 100 ms 往返变化
 */
#include "program/programs.h"

#include <Arduino.h>

#include "config/hardware.h"
#include "ui/view.h"
#include "zdt/motor.h"

namespace program {
namespace {

const uint32_t kSerialBaudRate = 115200;
const uint8_t kMotorAddress = 1;
const uint32_t kPulsesPerRevolution = 3200;
const uint32_t kStepIntervalMs = 100;
const int16_t kSpeedStepRpm = 10;
const int16_t kMaximumRpm = 300;

class MotorRampProgram final : public runtime::Program {
public:
    MotorRampProgram()
        : motorBus_(Serial2,
                    zdt::BusConfig(kSerialBaudRate, config::kPins.motorRx,
                                   config::kPins.motorTx)),
          motor_(motorBus_,
                 zdt::MotorConfig(kMotorAddress, kPulsesPerRevolution)),
          motorReady_(false),
          targetRpm_(0),
          direction_(1),
          lastStepAt_(0) {}

    void start(Adafruit_GFX& display, runtime::SystemState&) override {
        zdt::Status status = motorBus_.begin();
        if (status) {
            delay(2000);
            status = motor_.enable();
        }
        motorReady_ = static_cast<bool>(status);
        targetRpm_ = 0;
        direction_ = 1;
        lastStepAt_ = millis();
        ui::view::beginPage(display, "Motor Ramp");
        render(display);
    }

    void update(Adafruit_GFX& display, runtime::SystemState&,
                ui::Event) override {
        const uint32_t now = millis();
        if (!motorReady_ || now - lastStepAt_ < kStepIntervalMs) {
            return;
        }

        targetRpm_ = static_cast<int16_t>(targetRpm_ + direction_ * kSpeedStepRpm);
        if (targetRpm_ == kMaximumRpm) {
            direction_ = -1;
        } else if (targetRpm_ == -kMaximumRpm) {
            direction_ = 1;
        }

        if (targetRpm_ == 0) {
            motor_.stop();
        } else {
            motor_.run(targetRpm_);
        }
        lastStepAt_ = now;
        render(display);
    }

    void stop(runtime::SystemState&) override {
        motor_.stop();
        targetRpm_ = 0;
    }

private:
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
    bool motorReady_;
    int16_t targetRpm_;
    int8_t direction_;
    uint32_t lastStepAt_;
};

MotorRampProgram program;

}

runtime::Program& motorRamp() { return program; }

}
