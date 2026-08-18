/**
 * @file quaternion.cpp
 * @brief 使用陀螺仪积分并向调试串口输出四元数
 */
#include "program/programs.h"

#include <Arduino.h>

#include "control/quaternion.h"
#include "imu/gyroscope.h"
#include "ui/view.h"

namespace program {
namespace {

const uint32_t kSerialBaudRate = 115200;

class QuaternionProgram final : public runtime::Program {
public:
    QuaternionProgram() : ready_(false) {}

    void start(Adafruit_GFX& display, runtime::SystemState&) override {
        Serial.begin(kSerialBaudRate);
        integrator_.reset();
        ready_ = gyroscope::begin();
        ui::view::beginPage(display, "Quaternion");
        ui::view::beginBody(display);
        display.setCursor(6, 42);
        display.print(ready_ ? "Streaming" : "IMU error");
    }

    void update(Adafruit_GFX&, runtime::SystemState&, ui::Event) override {
        if (!ready_) {
            return;
        }

        gyroscope::Sample sample;
        if (!gyroscope::read(sample)) {
            return;
        }

        integrator_.update(sample.gyroX, sample.gyroY, sample.gyroZ,
                           sample.dtSeconds);

        const control::Quaternion& value = integrator_.value();
        Serial.print(value.scalar, 6);
        Serial.print(',');
        Serial.print(value.x, 6);
        Serial.print(',');
        Serial.print(value.y, 6);
        Serial.print(',');
        Serial.println(value.z, 6);
    }

    void stop(runtime::SystemState&) override { gyroscope::end(); }

private:
    control::QuaternionIntegrator integrator_;
    bool ready_;
};

QuaternionProgram program;

}

runtime::Program& quaternion() { return program; }

}
