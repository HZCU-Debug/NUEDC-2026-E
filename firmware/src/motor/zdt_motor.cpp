#include "motor/zdt_motor.h"

namespace motor {
namespace {

Error convert(zdt::Error error) {
    switch (error) {
        case zdt::Error::None: return Error::None;
        case zdt::Error::NotStarted: return Error::NotStarted;
        case zdt::Error::InvalidArgument: return Error::InvalidArgument;
        case zdt::Error::WriteFailed: return Error::WriteFailed;
        case zdt::Error::Timeout: return Error::Timeout;
        case zdt::Error::InvalidResponse: return Error::InvalidResponse;
    }
    return Error::InvalidResponse;
}

Status convert(const zdt::Status& status) {
    return Status(convert(status.error), static_cast<uint8_t>(status.error));
}

template <typename T>
Result<T> failure(zdt::Error error) {
    return Result<T>(convert(error), static_cast<uint8_t>(error));
}

}

ZdtMotor::ZdtMotor(zdt::Bus& bus, const zdt::MotorConfig& config)
    : bus_(bus), motor_(bus, config) {}

Status ZdtMotor::begin() { return convert(bus_.begin()); }

Status ZdtMotor::enable(bool enabled) {
    return convert(motor_.enable(enabled));
}

Status ZdtMotor::clearPosition() { return convert(motor_.clearPosition()); }

Status ZdtMotor::run(int16_t signedRpm, uint8_t acceleration) {
    return convert(motor_.run(signedRpm, acceleration));
}

Status ZdtMotor::moveRelative(float degrees, const MotionOptions& options) {
    return convert(motor_.moveRelative(
        degrees, zdt::MotionOptions(options.rpm, options.acceleration)));
}

Status ZdtMotor::moveAbsolute(float degrees, const MotionOptions& options) {
    return convert(motor_.moveAbsolute(
        degrees, zdt::MotionOptions(options.rpm, options.acceleration)));
}

Status ZdtMotor::stop() { return convert(motor_.stop()); }

Result<State> ZdtMotor::readState() {
    const zdt::Result<zdt::MotorState> result = motor_.readMotionState();
    if (!result) {
        return failure<State>(result.error);
    }
    State state;
    state.enabled = result.value.enabled;
    state.reached = result.value.reached;
    state.stalled = result.value.stalled;
    state.faulted = result.value.stalled || result.value.stallProtected;
    return Result<State>(state);
}

Result<float> ZdtMotor::readPositionDegrees() {
    const zdt::Result<float> result = motor_.readPositionDegrees();
    return result ? Result<float>(result.value) : failure<float>(result.error);
}

}
