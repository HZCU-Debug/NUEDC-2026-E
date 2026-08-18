#pragma once

#include "motor/motor.h"

class RecordingMotor : public motor::Motor {
public:
    RecordingMotor()
        : beginCount(0),
          enableCount(0),
          clearCount(0),
          runCount(0),
          absoluteCount(0),
          relativeCount(0),
          stopCount(0),
          enabled(false),
          lastRpm(0),
          lastDegrees(0.0f),
          lastOptions(1),
          state(),
          position(0.0f),
          nextStatus() {}

    motor::Status begin() override {
        ++beginCount;
        return nextStatus;
    }

    motor::Status enable(bool value = true) override {
        ++enableCount;
        enabled = value;
        return nextStatus;
    }

    motor::Status clearPosition() override {
        ++clearCount;
        position = 0.0f;
        return nextStatus;
    }

    motor::Status run(int16_t signedRpm, uint8_t acceleration = 0) override {
        ++runCount;
        lastRpm = signedRpm;
        lastOptions = motor::MotionOptions(
            static_cast<uint16_t>(signedRpm < 0 ? -signedRpm : signedRpm),
            acceleration);
        return nextStatus;
    }

    motor::Status moveRelative(
        float degrees, const motor::MotionOptions& options) override {
        ++relativeCount;
        lastDegrees = degrees;
        lastOptions = options;
        state.reached = false;
        return nextStatus;
    }

    motor::Status moveAbsolute(
        float degrees, const motor::MotionOptions& options) override {
        ++absoluteCount;
        lastDegrees = degrees;
        lastOptions = options;
        state.reached = false;
        return nextStatus;
    }

    motor::Status stop() override {
        ++stopCount;
        return nextStatus;
    }

    motor::Result<motor::State> readState() override {
        return motor::Result<motor::State>(state);
    }

    motor::Result<float> readPositionDegrees() override {
        return motor::Result<float>(position);
    }

    int beginCount;
    int enableCount;
    int clearCount;
    int runCount;
    int absoluteCount;
    int relativeCount;
    int stopCount;
    bool enabled;
    int16_t lastRpm;
    float lastDegrees;
    motor::MotionOptions lastOptions;
    motor::State state;
    float position;
    motor::Status nextStatus;
};
