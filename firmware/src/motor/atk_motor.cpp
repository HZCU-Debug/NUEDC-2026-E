#include "motor/atk_motor.h"

namespace motor {
namespace {

Error convert(atk::Error error) {
    switch (error) {
        case atk::Error::None: return Error::None;
        case atk::Error::NotStarted: return Error::NotStarted;
        case atk::Error::InvalidArgument: return Error::InvalidArgument;
        case atk::Error::WriteFailed: return Error::WriteFailed;
        case atk::Error::Timeout: return Error::Timeout;
        case atk::Error::InvalidResponse: return Error::InvalidResponse;
        case atk::Error::DeviceRejected: return Error::DeviceRejected;
    }
    return Error::InvalidResponse;
}

Status convert(const atk::Status& status) {
    return Status(convert(status.error), static_cast<uint8_t>(status.error));
}

template <typename T>
Result<T> failure(atk::Error error) {
    return Result<T>(convert(error), static_cast<uint8_t>(error));
}

}

AtkMotor::AtkMotor(atk::Bus& bus, const atk::MotorConfig& config)
    : bus_(bus), motor_(bus, config) {}

Status AtkMotor::begin() { return convert(bus_.begin()); }

Status AtkMotor::enable(bool enabled) {
    return convert(enabled ? motor_.enable() : motor_.disable());
}

Status AtkMotor::clearPosition() { return convert(motor_.clearPosition()); }

Status AtkMotor::run(int16_t signedRpm, uint8_t accelerationValue) {
    return convert(motor_.run(signedRpm, accelerationValue));
}

Status AtkMotor::moveRelative(float degrees, const MotionOptions& options) {
    return convert(motor_.moveRelative(
        degrees, atk::MotionOptions(options.rpm, options.acceleration)));
}

Status AtkMotor::moveAbsolute(float degrees, const MotionOptions& options) {
    return convert(motor_.moveAbsolute(
        degrees, atk::MotionOptions(options.rpm, options.acceleration)));
}

Status AtkMotor::stop() { return convert(motor_.stop()); }

Result<State> AtkMotor::readState() {
    const atk::Result<atk::MotorState> result = motor_.readState();
    if (!result) {
        return failure<State>(result.error);
    }
    State state;
    state.enabled = result.value.enabled;
    state.reached = result.value.reached;
    state.faulted = result.value.faulted;
    state.stalled = result.value.stalled;
    return Result<State>(state);
}

Result<float> AtkMotor::readPositionDegrees() {
    const atk::Result<float> result = motor_.readPositionDegrees();
    return result ? Result<float>(result.value) : failure<float>(result.error);
}

}
