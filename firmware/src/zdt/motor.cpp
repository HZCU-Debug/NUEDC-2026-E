#include "zdt/motor.h"

#include <math.h>

#include "protocol.h"

namespace zdt {
namespace {

const uint16_t kMaxRpm = 5000;
const uint32_t kCommandDelayMs = 10;

}

Bus::Bus(HardwareSerial& serial, const BusConfig& config)
    : serial_(serial), config_(config), started_(false) {}

Status Bus::begin() {
    if (started_) {
        return Status();
    }
    if (config_.baudRate == 0 || config_.timeoutMs == 0 ||
        config_.queryIntervalMs == 0) {
        return Status(Error::InvalidArgument);
    }
    serial_.begin(config_.baudRate, SERIAL_8N1, config_.rxPin, config_.txPin);
    started_ = true;
    discardInput();
    return Status();
}

Status Bus::triggerSynchronized() {
    const protocol::Frame frame = protocol::synchronizedTrigger();
    return command(frame.bytes, frame.size);
}

Status Bus::clearPositions() {
    const protocol::Frame frame = protocol::clearPosition(0);
    return command(frame.bytes, frame.size);
}

Status Bus::enableAll(bool enabled) {
    const protocol::Frame frame = protocol::enable(0, enabled);
    return command(frame.bytes, frame.size);
}

Status Bus::command(const uint8_t* frame, size_t size) {
    if (frame == nullptr || size < 2) {
        return Status(Error::InvalidArgument);
    }
    discardInput();
    const Status status = send(frame, size);
    if (status) {
        delay(kCommandDelayMs);
    }
    return status;
}

Status Bus::query(uint8_t address, uint8_t function, const uint8_t* frame, size_t frameSize,
                  uint8_t* response, size_t responseSize) {
    delay(config_.queryIntervalMs);
    discardInput();
    const Status sent = send(frame, frameSize);
    return sent ? receive(address, function, response, responseSize) : sent;
}

Status Bus::send(const uint8_t* frame, size_t size) {
    if (!started_) {
        return Status(Error::NotStarted);
    }
    if (serial_.write(frame, size) != size) {
        return Status(Error::WriteFailed);
    }
    serial_.flush();
    return Status();
}

Status Bus::receive(uint8_t address, uint8_t function, uint8_t* response, size_t size) {
    size_t received = 0;
    uint8_t recent[4];
    size_t recentSize = 0;
    const unsigned long startedAt = millis();
    while (received < size && millis() - startedAt < config_.timeoutMs) {
        if (serial_.available()) {
            const uint8_t byte = static_cast<uint8_t>(serial_.read());
            if (recentSize < sizeof(recent)) {
                recent[recentSize++] = byte;
            } else {
                recent[0] = recent[1];
                recent[1] = recent[2];
                recent[2] = recent[3];
                recent[3] = byte;
            }
            if (protocol::isProtocolError(recent, recentSize, address)) {
                return Status(Error::InvalidResponse);
            }

            if (received == 0) {
                if (byte == address) {
                    response[received++] = byte;
                }
            } else if (received == 1) {
                if (byte == function) {
                    response[received++] = byte;
                } else if (byte != address) {
                    received = 0;
                }
            } else {
                response[received++] = byte;
            }
        } else {
            delay(1);
        }
    }
    if (received != size) {
        return Status(Error::Timeout);
    }
    return Status(protocol::validateResponse(response, size, address, function));
}

void Bus::discardInput() {
    while (serial_.available()) {
        serial_.read();
    }
}

Motor::Motor(Bus& bus, const MotorConfig& config) : bus_(bus), config_(config) {}

Status Motor::enable(bool enabled) {
    if (!valid()) {
        return Status(Error::InvalidArgument);
    }
    const protocol::Frame frame = protocol::enable(config_.address, enabled);
    return bus_.command(frame.bytes, frame.size);
}

Status Motor::clearPosition() {
    if (!valid()) {
        return Status(Error::InvalidArgument);
    }
    const protocol::Frame frame = protocol::clearPosition(config_.address);
    return bus_.command(frame.bytes, frame.size);
}

Status Motor::run(int16_t signedRpm, uint8_t acceleration, Start start) {
    if (!valid() || signedRpm == 0 || signedRpm < -static_cast<int16_t>(kMaxRpm) ||
        signedRpm > static_cast<int16_t>(kMaxRpm)) {
        return Status(Error::InvalidArgument);
    }

    const bool negative = signedRpm < 0;
    const uint16_t rpm = static_cast<uint16_t>(negative ? -signedRpm : signedRpm);
    const protocol::Frame frame =
        protocol::run(config_.address, directionFor(negative), rpm, acceleration, start);
    return bus_.command(frame.bytes, frame.size);
}

Status Motor::moveRelative(float degrees, const MotionOptions& options) {
    return move(degrees, options, false);
}

Status Motor::moveAbsolute(float degrees, const MotionOptions& options) {
    return move(degrees, options, true);
}

Status Motor::move(float degrees, const MotionOptions& options, bool absolute) {
    if (!valid() || !isfinite(degrees) || (!absolute && degrees == 0.0f) ||
        options.rpm == 0 ||
        options.rpm > kMaxRpm) {
        return Status(Error::InvalidArgument);
    }

    const bool negative = degrees < 0.0f;
    const double pulsesValue = fabs(static_cast<double>(degrees)) *
                               config_.pulsesPerRevolution / 360.0;
    if ((degrees != 0.0f && pulsesValue < 0.5) ||
        pulsesValue > 4294967295.0) {
        return Status(Error::InvalidArgument);
    }

    const protocol::Frame frame = protocol::move(
        config_.address, directionFor(negative), options.rpm, options.acceleration,
        static_cast<uint32_t>(pulsesValue + 0.5), absolute, options.start);
    return bus_.command(frame.bytes, frame.size);
}

Status Motor::stop(Start start) {
    if (!valid()) {
        return Status(Error::InvalidArgument);
    }
    const protocol::Frame frame = protocol::stop(config_.address, start);
    return bus_.command(frame.bytes, frame.size);
}

Status Motor::home(HomeMode mode, Start start) {
    if (!valid()) {
        return Status(Error::InvalidArgument);
    }
    const protocol::Frame frame = protocol::home(config_.address, mode, start);
    return bus_.command(frame.bytes, frame.size);
}

Result<MotorState> Motor::readState() {
    const Result<MotorState> motorState = readMotionState();
    if (!motorState) {
        return motorState;
    }

    const protocol::Frame homeRequest =
        protocol::query(config_.address, protocol::Query::HomeState);
    uint8_t homeResponse[4];
    const Status status = bus_.query(
        config_.address, homeRequest.function, homeRequest.bytes,
        homeRequest.size, homeResponse, sizeof(homeResponse));
    if (!status) {
        return Result<MotorState>(status.error);
    }
    MotorState combined = motorState.value;
    const MotorState homeState = protocol::motorState(0, homeResponse[2]);
    combined.homing = homeState.homing;
    combined.homingFailed = homeState.homingFailed;
    return Result<MotorState>(combined);
}

Result<MotorState> Motor::readMotionState() {
    if (!valid()) {
        return Result<MotorState>(Error::InvalidArgument);
    }

    const protocol::Frame motorRequest = protocol::query(config_.address, protocol::Query::State);
    uint8_t motorResponse[4];
    const Status status = bus_.query(
        config_.address, motorRequest.function, motorRequest.bytes,
        motorRequest.size, motorResponse, sizeof(motorResponse));
    if (!status) {
        return Result<MotorState>(status.error);
    }
    return Result<MotorState>(protocol::motorState(motorResponse[2], 0));
}

Result<float> Motor::readPositionDegrees() {
    if (!valid()) {
        return Result<float>(Error::InvalidArgument);
    }
    const protocol::Frame request = protocol::query(config_.address, protocol::Query::Position);
    uint8_t response[8];
    const Status status = bus_.query(config_.address, request.function, request.bytes, request.size,
                                     response, sizeof(response));
    return status ? Result<float>(protocol::positionDegrees(response, config_.invertDirection))
                  : Result<float>(status.error);
}

Result<float> Motor::readSpeedRpm() {
    if (!valid()) {
        return Result<float>(Error::InvalidArgument);
    }
    const protocol::Frame request = protocol::query(config_.address, protocol::Query::Speed);
    uint8_t response[6];
    const Status status = bus_.query(config_.address, request.function, request.bytes, request.size,
                                     response, sizeof(response));
    return status ? Result<float>(protocol::speedRpm(response, config_.invertDirection))
                  : Result<float>(status.error);
}

Result<MotorSnapshot> Motor::readSnapshot() {
    const Result<MotorState> state = readState();
    if (!state) {
        return Result<MotorSnapshot>(state.error);
    }
    const Result<float> position = readPositionDegrees();
    if (!position) {
        return Result<MotorSnapshot>(position.error);
    }
    const Result<float> speed = readSpeedRpm();
    if (!speed) {
        return Result<MotorSnapshot>(speed.error);
    }

    MotorSnapshot snapshot;
    snapshot.state = state.value;
    snapshot.positionDegrees = position.value;
    snapshot.speedRpm = speed.value;
    return Result<MotorSnapshot>(snapshot);
}

bool Motor::valid() const {
    return config_.address != 0 && config_.pulsesPerRevolution != 0;
}

uint8_t Motor::directionFor(bool negative) const {
    return static_cast<uint8_t>(negative != config_.invertDirection);
}

}
