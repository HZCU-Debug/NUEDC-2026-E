#include "atk/motor.h"

#include <math.h>
#include <string.h>

namespace atk {
namespace {

const uint8_t kFrameHeader = 0xC5;
const uint8_t kFrameFooter = 0x5C;
const uint8_t kMaxPayloadSize = 8;
const uint8_t kMaxFrameSize = kMaxPayloadSize + 5;
const uint8_t kOperationSucceeded = 0x01;
const uint8_t kRejectedRetryCount = 2;
const uint32_t kRejectedRetryDelayMs = 5;

const uint16_t kMaxRpm = 6000;
const uint8_t kMaxAcceleration = 200;
const uint32_t kCountsPerRevolution = 51200;

const uint8_t kReadSpeed = 0x29;
const uint8_t kReadPosition = 0x2A;
const uint8_t kReadState = 0x2C;
const uint8_t kReadStalled = 0x2D;
const uint8_t kReadEnabled = 0x2F;
const uint8_t kReadReached = 0x30;
const uint8_t kTriggerHome = 0x92;
const uint8_t kVelocityMode = 0xF1;
const uint8_t kAbsolutePositionMode = 0xF2;
const uint8_t kRelativePositionMode = 0xF3;
const uint8_t kClearPosition = 0xF8;
const uint8_t kEnable = 0xFA;

uint8_t checksum(const uint8_t* data, size_t size) {
    uint8_t value = 0;
    for (size_t i = 0; i < size; ++i) {
        value = static_cast<uint8_t>(value + data[i]);
    }
    return value;
}

void putUint16(uint8_t* destination, uint16_t value) {
    destination[0] = static_cast<uint8_t>(value >> 8);
    destination[1] = static_cast<uint8_t>(value);
}

void putUint32(uint8_t* destination, uint32_t value) {
    destination[0] = static_cast<uint8_t>(value >> 24);
    destination[1] = static_cast<uint8_t>(value >> 16);
    destination[2] = static_cast<uint8_t>(value >> 8);
    destination[3] = static_cast<uint8_t>(value);
}

uint16_t readUint16(const uint8_t* source) {
    return static_cast<uint16_t>(source[0]) << 8 | source[1];
}

uint32_t readUint32(const uint8_t* source) {
    return static_cast<uint32_t>(source[0]) << 24 |
           static_cast<uint32_t>(source[1]) << 16 |
           static_cast<uint32_t>(source[2]) << 8 | source[3];
}

void putFloat(uint8_t* destination, float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    putUint32(destination, bits);
}

}

Bus::Bus(HardwareSerial& serial, const BusConfig& config)
    : serial_(serial), config_(config), started_(false) {}

Status Bus::begin() {
    if (started_) {
        return Status();
    }
    if (config_.baudRate == 0 || config_.timeoutMs == 0 ||
        config_.commandIntervalMs < 2) {
        return Status(Error::InvalidArgument);
    }
    serial_.begin(config_.baudRate, SERIAL_8N1, config_.rxPin, config_.txPin);
    started_ = true;
    discardInput();
    return Status();
}

Status Bus::command(uint8_t address, int8_t rxPin, uint8_t function,
                    const uint8_t* payload, uint8_t payloadSize,
                    uint8_t* response, uint8_t responseSize) {
    if (address == 0 || payloadSize > kMaxPayloadSize || responseSize == 0 ||
        responseSize > kMaxPayloadSize ||
        (payloadSize != 0 && payload == nullptr) || response == nullptr) {
        return Status(Error::InvalidArgument);
    }
    if (rxPin >= 0 && !serial_.setPins(rxPin, -1)) {
        return Status(Error::InvalidArgument);
    }

    uint8_t frame[kMaxFrameSize] = {kFrameHeader, address, function};
    for (uint8_t i = 0; i < payloadSize; ++i) {
        frame[3 + i] = payload[i];
    }
    frame[3 + payloadSize] = checksum(frame, 3 + payloadSize);
    frame[4 + payloadSize] = kFrameFooter;

    for (uint8_t attempt = 0; attempt <= kRejectedRetryCount; ++attempt) {
        delay(attempt == 0 ? config_.commandIntervalMs
                           : kRejectedRetryDelayMs);
        discardInput();
        const Status sent = send(frame, payloadSize + 5);
        if (!sent) {
            return sent;
        }
        const Status received =
            receive(address, function, response, responseSize);
        if (!received) {
            return received;
        }
        if (response[0] == kOperationSucceeded) {
            return Status();
        }
    }
    return Status(Error::DeviceRejected);
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

Status Bus::receive(uint8_t address, uint8_t function, uint8_t* response,
                    uint8_t responseSize) {
    uint8_t expectedSize = responseSize + 5;
    uint8_t frame[kMaxFrameSize];
    uint8_t received = 0;
    const unsigned long startedAt = millis();
    while (received < expectedSize &&
           millis() - startedAt < config_.timeoutMs) {
        if (!serial_.available()) {
            delay(1);
            continue;
        }

        const uint8_t byte = static_cast<uint8_t>(serial_.read());
        if (received == 0) {
            if (byte == kFrameHeader) {
                frame[received++] = byte;
            }
            continue;
        }
        if (received == 1 && byte != address) {
            received = byte == kFrameHeader ? 1 : 0;
            continue;
        }
        if (received == 2 && byte != function) {
            received = byte == kFrameHeader ? 1 : 0;
            continue;
        }
        frame[received++] = byte;
        if (received == 4 && byte >= 0xE1 && byte <= 0xE6) {
            expectedSize = 6;
        }
    }

    if (received != expectedSize) {
        return Status(Error::Timeout);
    }
    if (frame[expectedSize - 1] != kFrameFooter ||
        frame[expectedSize - 2] != checksum(frame, expectedSize - 2)) {
        return Status(Error::InvalidResponse);
    }
    const uint8_t actualResponseSize = expectedSize - 5;
    for (uint8_t i = 0; i < actualResponseSize; ++i) {
        response[i] = frame[3 + i];
    }
    return Status();
}

void Bus::discardInput() {
    while (serial_.available()) {
        serial_.read();
    }
}

Motor::Motor(Bus& bus, const MotorConfig& config) : bus_(bus), config_(config) {}

Status Motor::command(uint8_t function, const uint8_t* payload,
                      uint8_t payloadSize, uint8_t* response,
                      uint8_t responseSize) {
    return bus_.command(config_.address, config_.rxPin, function, payload,
                        payloadSize, response, responseSize);
}

Status Motor::enable() {
    if (!valid()) {
        return Status(Error::InvalidArgument);
    }
    const uint8_t request[] = {0};
    uint8_t response[2];
    const Status status = command(kEnable, request, sizeof(request), response,
                                  sizeof(response));
    return status && response[1] != 0 ? Status(Error::InvalidResponse) : status;
}

Status Motor::disable() {
    if (!valid()) {
        return Status(Error::InvalidArgument);
    }
    const uint8_t request[] = {1};
    uint8_t response[2];
    const Status status = command(kEnable, request, sizeof(request), response,
                                  sizeof(response));
    return status && response[1] != 1 ? Status(Error::InvalidResponse) : status;
}

Status Motor::clearPosition() {
    if (!valid()) {
        return Status(Error::InvalidArgument);
    }
    uint8_t response[1];
    return command(kClearPosition, nullptr, 0, response, sizeof(response));
}

Status Motor::run(int16_t signedRpm, uint8_t acceleration) {
    if (!valid() || signedRpm < -static_cast<int16_t>(kMaxRpm) ||
        signedRpm > static_cast<int16_t>(kMaxRpm) ||
        acceleration > kMaxAcceleration) {
        return Status(Error::InvalidArgument);
    }

    const bool negative = signedRpm < 0;
    const float rpm = static_cast<float>(negative ? -signedRpm : signedRpm);
    uint8_t request[6] = {static_cast<uint8_t>(directionFor(negative)),
                          acceleration, 0, 0, 0, 0};
    putFloat(&request[2], rpm);
    uint8_t response[1];
    return command(kVelocityMode, request, sizeof(request), response,
                   sizeof(response));
}

Status Motor::moveRelative(float degrees, const MotionOptions& options) {
    return move(degrees, options, false);
}

Status Motor::moveAbsolute(float degrees, const MotionOptions& options) {
    return move(degrees, options, true);
}

Status Motor::move(float degrees, const MotionOptions& options, bool absolute) {
    if (!valid() || !isfinite(degrees) || (!absolute && degrees == 0.0f) ||
        options.rpm == 0 || options.rpm > kMaxRpm ||
        options.acceleration > kMaxAcceleration) {
        return Status(Error::InvalidArgument);
    }

    const bool negative = degrees < 0.0f;
    const double countsValue = fabs(static_cast<double>(degrees)) *
                               kCountsPerRevolution / 360.0;
    if ((degrees != 0.0f && countsValue < 0.5) || countsValue > 4294967295.0) {
        return Status(Error::InvalidArgument);
    }

    uint8_t request[8] = {static_cast<uint8_t>(directionFor(negative)),
                          options.acceleration, 0, 0, 0, 0, 0, 0};
    putUint16(&request[2], options.rpm);
    putUint32(&request[4], static_cast<uint32_t>(countsValue + 0.5));
    uint8_t response[1];
    return command(absolute ? kAbsolutePositionMode : kRelativePositionMode,
                   request, sizeof(request), response, sizeof(response));
}

Status Motor::stop() {
    return run(0, 0);
}

Status Motor::home(HomeMode mode) {
    const uint8_t value = static_cast<uint8_t>(mode);
    if (!valid() || value > static_cast<uint8_t>(HomeMode::MultiTurn)) {
        return Status(Error::InvalidArgument);
    }
    const uint8_t request[] = {value};
    uint8_t response[2];
    const Status status = command(kTriggerHome, request, sizeof(request),
                                  response, sizeof(response));
    return status && response[1] != value ? Status(Error::InvalidResponse)
                                          : status;
}

Result<MotorState> Motor::readState() {
    if (!valid()) {
        return Result<MotorState>(Error::InvalidArgument);
    }

    uint8_t running[2];
    Status status = command(kReadState, nullptr, 0, running, sizeof(running));
    if (!status) {
        return Result<MotorState>(status.error);
    }
    uint8_t enabled[2];
    status = command(kReadEnabled, nullptr, 0, enabled, sizeof(enabled));
    if (!status) {
        return Result<MotorState>(status.error);
    }
    uint8_t reached[2];
    status = command(kReadReached, nullptr, 0, reached, sizeof(reached));
    if (!status) {
        return Result<MotorState>(status.error);
    }
    uint8_t stalled[2];
    status = command(kReadStalled, nullptr, 0, stalled, sizeof(stalled));
    if (!status) {
        return Result<MotorState>(status.error);
    }

    MotorState state;
    state.enabled = enabled[1] == 0;
    state.reached = reached[1] != 0;
    state.faulted = running[1] >= 3;
    state.stalled = running[1] == 4 || stalled[1] != 0;
    return Result<MotorState>(state);
}

Result<float> Motor::readPositionDegrees() {
    if (!valid()) {
        return Result<float>(Error::InvalidArgument);
    }
    uint8_t response[5];
    const Status status =
        command(kReadPosition, nullptr, 0, response, sizeof(response));
    if (!status) {
        return Result<float>(status.error);
    }
    float degrees = static_cast<int32_t>(readUint32(&response[1])) *
                    (360.0f / kCountsPerRevolution);
    if (config_.invertDirection) {
        degrees = -degrees;
    }
    return Result<float>(degrees);
}

Result<float> Motor::readSpeedRpm() {
    if (!valid()) {
        return Result<float>(Error::InvalidArgument);
    }
    uint8_t response[3];
    const Status status =
        command(kReadSpeed, nullptr, 0, response, sizeof(response));
    if (!status) {
        return Result<float>(status.error);
    }
    float rpm = static_cast<int16_t>(readUint16(&response[1]));
    if (config_.invertDirection) {
        rpm = -rpm;
    }
    return Result<float>(rpm);
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

bool Motor::valid() const { return config_.address != 0; }

bool Motor::directionFor(bool negative) const {
    return negative != config_.invertDirection;
}

}
