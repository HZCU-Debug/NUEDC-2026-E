#include "protocol.h"

namespace zdt {
namespace protocol {
namespace {

const uint8_t kChecksum = 0x6B;

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

}

Frame synchronizedTrigger() {
    Frame frame = {{0x00, 0xFF, 0x66, kChecksum}, 4, 0xFF};
    return frame;
}

Frame clearPosition(uint8_t address) {
    Frame frame = {{address, 0x0A, 0x6D, kChecksum}, 4, 0x0A};
    return frame;
}

Frame enable(uint8_t address, bool enabled) {
    Frame frame = {{address, 0xF3, 0xAB, static_cast<uint8_t>(enabled), 0x00,
                    kChecksum},
                   6,
                   0xF3};
    return frame;
}

Frame run(uint8_t address, uint8_t direction, uint16_t rpm, uint8_t acceleration,
          Start start) {
    Frame frame = {{address, 0xF6, direction, 0x00, 0x00, acceleration,
                    static_cast<uint8_t>(start), kChecksum},
                   8,
                   0xF6};
    putUint16(&frame.bytes[3], rpm);
    return frame;
}

Frame move(uint8_t address, uint8_t direction, uint16_t rpm, uint8_t acceleration,
           uint32_t pulses, bool absolute, Start start) {
    Frame frame = {{address, 0xFD, direction, 0x00, 0x00, acceleration, 0x00, 0x00, 0x00,
                    0x00, static_cast<uint8_t>(absolute), static_cast<uint8_t>(start),
                    kChecksum},
                   13,
                   0xFD};
    putUint16(&frame.bytes[3], rpm);
    putUint32(&frame.bytes[6], pulses);
    return frame;
}

Frame stop(uint8_t address, Start start) {
    Frame frame = {{address, 0xFE, 0x98, static_cast<uint8_t>(start), kChecksum}, 5, 0xFE};
    return frame;
}

Frame home(uint8_t address, HomeMode mode, Start start) {
    Frame frame = {{address, 0x9A, static_cast<uint8_t>(mode), static_cast<uint8_t>(start),
                    kChecksum},
                   5,
                   0x9A};
    return frame;
}

Frame query(uint8_t address, Query query) {
    const uint8_t function = static_cast<uint8_t>(query);
    Frame frame = {{address, function, kChecksum}, 3, function};
    return frame;
}

bool isProtocolError(const uint8_t* response, size_t size, uint8_t address) {
    return size == 4 && response[0] == address && response[1] == 0x00 &&
           response[2] == 0xEE && response[3] == kChecksum;
}

Error validateResponse(const uint8_t* response, size_t size, uint8_t address,
                       uint8_t function) {
    if (response[0] != address || response[1] != function || response[size - 1] != kChecksum) {
        return Error::InvalidResponse;
    }
    return Error::None;
}

MotorState motorState(uint8_t motorFlags, uint8_t homeFlags) {
    MotorState state;
    state.enabled = motorFlags & 0x01;
    state.reached = motorFlags & 0x02;
    state.stalled = motorFlags & 0x04;
    state.stallProtected = motorFlags & 0x08;
    state.homing = homeFlags & 0x04;
    state.homingFailed = homeFlags & 0x08;
    return state;
}

float positionDegrees(const uint8_t* response, bool invertDirection) {
    float degrees = readUint32(&response[3]) * (360.0f / 65536.0f);
    if ((response[2] != 0) != invertDirection) {
        degrees = -degrees;
    }
    return degrees;
}

float speedRpm(const uint8_t* response, bool invertDirection) {
    float rpm = readUint16(&response[3]);
    if ((response[2] != 0) != invertDirection) {
        rpm = -rpm;
    }
    return rpm;
}

}
}
