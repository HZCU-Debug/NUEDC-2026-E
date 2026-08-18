#pragma once

#include <stddef.h>
#include <stdint.h>

#include "zdt/motor.h"

namespace zdt {
namespace protocol {

const size_t kMaxFrameSize = 13;

enum class Query : uint8_t {
    Speed = 0x35,
    Position = 0x36,
    State = 0x3A,
    HomeState = 0x3B,
};

struct Frame {
    uint8_t bytes[kMaxFrameSize];
    size_t size;
    uint8_t function;
};

Frame synchronizedTrigger();
Frame clearPosition(uint8_t address);
Frame enable(uint8_t address, bool enabled);
Frame run(uint8_t address, uint8_t direction, uint16_t rpm, uint8_t acceleration, Start start);
Frame move(uint8_t address, uint8_t direction, uint16_t rpm, uint8_t acceleration,
           uint32_t pulses, bool absolute, Start start);
Frame stop(uint8_t address, Start start);
Frame home(uint8_t address, HomeMode mode, Start start);
Frame query(uint8_t address, Query query);

bool isProtocolError(const uint8_t* response, size_t size, uint8_t address);
Error validateResponse(const uint8_t* response, size_t size, uint8_t address,
                       uint8_t function);
MotorState motorState(uint8_t motorFlags, uint8_t homeFlags);
float positionDegrees(const uint8_t* response, bool invertDirection);
float speedRpm(const uint8_t* response, bool invertDirection);

}
}
