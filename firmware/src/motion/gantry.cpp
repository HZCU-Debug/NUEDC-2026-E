#include "motion/gantry.h"

#include <Arduino.h>
#include <math.h>

#include "config/parameters.h"

namespace motion {
namespace {

bool faulted(const motor::Result<motor::State>& state) {
    return state && state.value.faulted;
}

uint8_t flags(const motor::State& state) {
    return static_cast<uint8_t>(state.enabled) |
           static_cast<uint8_t>(state.reached) << 1 |
           static_cast<uint8_t>(state.stalled) << 2 |
           static_cast<uint8_t>(state.faulted) << 3;
}

bool atTarget(float position, float target, float tolerance) {
    return fabsf(position - target) <= tolerance;
}

}

Gantry::Gantry(motor::Motor& x, motor::Motor& y, motor::Motor& z,
               motor::Motor& rotation)
    : x_(x),
      y_(y),
      z_(z),
      rotation_(rotation),
      pollDiagnostic_(),
      targetX_(0.0f),
      targetY_(0.0f),
      targetZ_(0.0f),
      targetRotation_(0.0f) {}

motor::Status Gantry::begin() {
    motor::Status status = record('X', x_.begin());
    if (!status) return status;
    status = record('Y', y_.begin());
    if (!status) return status;
    status = record('Z', z_.begin());
    if (!status) return status;
    status = record('R', rotation_.begin());
    if (!status) return status;

    delay(2000);
    status = record('X', x_.clearPosition());
    if (!status) return status;
    status = record('Y', y_.clearPosition());
    if (!status) return status;
    status = record('Z', z_.clearPosition());
    return status ? record('R', rotation_.clearPosition()) : status;
}

motor::Status Gantry::setEnabled(bool enabled) {
    motor::Status status = record('X', x_.enable(enabled));
    if (!status) return status;
    status = record('Y', y_.enable(enabled));
    if (!status) return status;
    status = record('Z', z_.enable(enabled));
    return status ? record('R', rotation_.enable(enabled)) : status;
}

motor::Status Gantry::setEnabled(Axis axis, bool enabled) {
    return record(axis == Axis::Rotation ? 'R' : "XYZ"[static_cast<uint8_t>(axis)],
                  motorFor(axis).enable(enabled));
}

motor::Status Gantry::record(char axis, const motor::Status& status) {
    if (!status) {
        pollDiagnostic_ = PollDiagnostic(axis, status.error, status.detail);
    }
    return status;
}

motor::Motor& Gantry::motorFor(Axis axis) {
    switch (axis) {
        case Axis::X:
            return x_;
        case Axis::Y:
            return y_;
        case Axis::Z:
            return z_;
        case Axis::Rotation:
            return rotation_;
    }
    return x_;
}

motor::Status Gantry::run(Axis axis, int16_t signedRpm,
                          uint8_t acceleration) {
    return record(axis == Axis::Rotation ? 'R' : "XYZ"[static_cast<uint8_t>(axis)],
                  motorFor(axis).run(signedRpm, acceleration));
}

motor::Status Gantry::moveAbsolute(
    Axis axis, float degrees, const motor::MotionOptions& options) {
    return record(axis == Axis::Rotation ? 'R' : "XYZ"[static_cast<uint8_t>(axis)],
                  motorFor(axis).moveAbsolute(degrees, options));
}

motor::Status Gantry::moveXY(float xDegrees, float yDegrees,
                             const motor::MotionOptions& options) {
    motor::Status status = record('X', x_.moveAbsolute(xDegrees, options));
    if (!status) return status;
    status = record('Y', y_.moveAbsolute(yDegrees, options));
    if (status) {
        targetX_ = xDegrees;
        targetY_ = yDegrees;
    }
    return status;
}

motor::Status Gantry::moveZ(float degrees,
                            const motor::MotionOptions& options) {
    const motor::Status status = record('Z', z_.moveAbsolute(degrees, options));
    if (status) targetZ_ = degrees;
    return status;
}

motor::Status Gantry::moveXYRotation(
    float xDegrees, float yDegrees, float rotationDegrees,
    const motor::MotionOptions& xyOptions,
    const motor::MotionOptions& rotationOptions) {
    return moveXYRotationImpl(xDegrees, yDegrees, rotationDegrees, xyOptions,
                              rotationOptions, false);
}

motor::Status Gantry::moveXYRelativeRotation(
    float xDegrees, float yDegrees, float rotationDegrees,
    const motor::MotionOptions& xyOptions,
    const motor::MotionOptions& rotationOptions) {
    return moveXYRotationImpl(xDegrees, yDegrees, rotationDegrees, xyOptions,
                              rotationOptions, true);
}

motor::Status Gantry::moveXYRotationImpl(
    float xDegrees, float yDegrees, float rotationDegrees,
    const motor::MotionOptions& xyOptions,
    const motor::MotionOptions& rotationOptions, bool relativeRotation) {
    motor::Status status = record('X', x_.moveAbsolute(xDegrees, xyOptions));
    if (!status) return status;
    status = record('Y', y_.moveAbsolute(yDegrees, xyOptions));
    if (!status) return status;
    if (!relativeRotation || rotationDegrees != 0.0f) {
        status = record(
            'R', relativeRotation
                     ? rotation_.moveRelative(rotationDegrees, rotationOptions)
                     : rotation_.moveAbsolute(rotationDegrees, rotationOptions));
        if (!status) return status;
    }
    targetX_ = xDegrees;
    targetY_ = yDegrees;
    targetRotation_ = relativeRotation ? targetRotation_ + rotationDegrees
                                       : rotationDegrees;
    return status;
}

Arrival Gantry::pollXY() {
    const motor::Result<motor::State> xState = x_.readState();
    const motor::Result<motor::State> yState = y_.readState();
    if (!xState) {
        pollDiagnostic_ = PollDiagnostic('X', xState.error, xState.detail);
    } else if (!yState) {
        pollDiagnostic_ = PollDiagnostic('Y', yState.error, yState.detail);
    } else if (!xState.value.reached) {
        pollDiagnostic_ = PollDiagnostic('X', motor::Error::None, 0,
                                         flags(xState.value));
    } else {
        pollDiagnostic_ = PollDiagnostic('Y', motor::Error::None, 0,
                                         flags(yState.value));
    }
    if (faulted(xState) || faulted(yState)) return Arrival::Fault;
    if (xState && yState && xState.value.reached && yState.value.reached) {
        return Arrival::Reached;
    }

    const motor::Result<float> xPosition = x_.readPositionDegrees();
    if (!xPosition) {
        pollDiagnostic_ =
            PollDiagnostic('X', xPosition.error, xPosition.detail);
        return Arrival::Pending;
    }
    const motor::Result<float> yPosition = y_.readPositionDegrees();
    if (!yPosition) {
        pollDiagnostic_ =
            PollDiagnostic('Y', yPosition.error, yPosition.detail);
        return Arrival::Pending;
    }
    return atTarget(xPosition.value, targetX_,
                    config::kLinearArrivalToleranceDegrees) &&
                   atTarget(yPosition.value, targetY_,
                            config::kLinearArrivalToleranceDegrees)
               ? Arrival::Reached
               : Arrival::Pending;
}

Arrival Gantry::pollZ() {
    const motor::Result<motor::State> state = z_.readState();
    pollDiagnostic_ = state
                          ? PollDiagnostic('Z', motor::Error::None, 0,
                                           flags(state.value))
                          : PollDiagnostic('Z', state.error, state.detail);
    if (faulted(state)) return Arrival::Fault;
    if (state && state.value.reached) return Arrival::Reached;
    const motor::Result<float> position = z_.readPositionDegrees();
    if (!position) {
        pollDiagnostic_ = PollDiagnostic('Z', position.error, position.detail);
        return Arrival::Pending;
    }
    return atTarget(position.value, targetZ_,
                    config::kLinearArrivalToleranceDegrees)
               ? Arrival::Reached
               : Arrival::Pending;
}

Arrival Gantry::pollXYRotation() {
    const Arrival xy = pollXY();
    const PollDiagnostic xyDiagnostic = pollDiagnostic_;
    const motor::Result<motor::State> rotationState = rotation_.readState();
    pollDiagnostic_ =
        rotationState
            ? PollDiagnostic('R', motor::Error::None, 0,
                             flags(rotationState.value))
            : PollDiagnostic('R', rotationState.error, rotationState.detail);
    if (xy == Arrival::Fault || faulted(rotationState)) {
        if (xy == Arrival::Fault) pollDiagnostic_ = xyDiagnostic;
        return Arrival::Fault;
    }
    if (xy != Arrival::Reached) {
        pollDiagnostic_ = xyDiagnostic;
        return Arrival::Pending;
    }
    if (rotationState && rotationState.value.reached) return Arrival::Reached;
    const motor::Result<float> position = rotation_.readPositionDegrees();
    if (!position) {
        pollDiagnostic_ = PollDiagnostic('R', position.error, position.detail);
        return Arrival::Pending;
    }
    return atTarget(position.value, targetRotation_,
                    config::kRollArrivalToleranceDegrees)
               ? Arrival::Reached
               : Arrival::Pending;
}

PollDiagnostic Gantry::pollDiagnostic() const { return pollDiagnostic_; }

motor::Result<XYPosition> Gantry::readXY() {
    const motor::Result<float> x = x_.readPositionDegrees();
    if (!x) return motor::Result<XYPosition>(x.error, x.detail);
    const motor::Result<float> y = y_.readPositionDegrees();
    return y ? motor::Result<XYPosition>(XYPosition(x.value, y.value))
             : motor::Result<XYPosition>(y.error, y.detail);
}

motor::Result<float> Gantry::readZ() { return z_.readPositionDegrees(); }

motor::Status Gantry::stop() {
    const motor::Status xStatus = record('X', x_.stop());
    const motor::Status yStatus = record('Y', y_.stop());
    const motor::Status zStatus = record('Z', z_.stop());
    const motor::Status rotationStatus = record('R', rotation_.stop());
    if (!xStatus) return xStatus;
    if (!yStatus) return yStatus;
    return zStatus ? rotationStatus : zStatus;
}

}
