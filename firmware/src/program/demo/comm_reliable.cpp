/**
 * @file comm_reliable.cpp
 * @brief 通过 USB 串口可靠发送计数消息并在确认后等待 500 ms 再发送下一条
 */
#include "program/programs.h"

#include <Arduino.h>

#include "comm/link.h"
#include "ui/view.h"

namespace program {
namespace {

const uint32_t kSerialBaudRate = 115200;
const uint32_t kSendIntervalMs = 500;
const uint8_t kCounterMessage = 1;

class CommReliableProgram final : public runtime::Program {
public:
    CommReliableProgram()
        : link_(Serial, comm::LinkConfig(kSerialBaudRate)),
          counter_(0),
          lastDeliveredAt_(0),
          waiting_(false),
          firstMessage_(true) {}

    void start(Adafruit_GFX& display, runtime::SystemState&) override {
        counter_ = 0;
        waiting_ = false;
        firstMessage_ = true;
        link_.begin();
        lastDeliveredAt_ = millis();
        ui::view::beginPage(display, "Comm Reliable");
        render(display);
    }

    void update(Adafruit_GFX& display, runtime::SystemState&,
                ui::Event) override {
        const comm::Event event = link_.poll();
        if (event.type == comm::EventType::Delivered) {
            waiting_ = false;
            firstMessage_ = false;
            ++counter_;
            lastDeliveredAt_ = millis();
            render(display);
        }

        const uint32_t now = millis();
        if (waiting_ || (!firstMessage_ && now - lastDeliveredAt_ < kSendIntervalMs)) {
            return;
        }

        uint8_t payload[4];
        encodeCounter(payload);
        waiting_ = link_.send(kCounterMessage, payload, sizeof(payload),
                              comm::Delivery::Reliable) ==
                   comm::SendResult::Accepted;
        render(display);
    }

    void stop(runtime::SystemState&) override {
        link_.cancel();
        waiting_ = false;
    }

private:
    void encodeCounter(uint8_t* payload) const {
        payload[0] = static_cast<uint8_t>(counter_ >> 24);
        payload[1] = static_cast<uint8_t>(counter_ >> 16);
        payload[2] = static_cast<uint8_t>(counter_ >> 8);
        payload[3] = static_cast<uint8_t>(counter_);
    }

    void render(Adafruit_GFX& display) const {
        ui::view::beginBody(display);
        display.setCursor(6, 42);
        display.print("Count: ");
        display.print(counter_);
        display.setCursor(6, 68);
        display.print(waiting_ ? "Waiting" : firstMessage_ ? "Ready" : "Delivered");
    }

    comm::Link<4> link_;
    uint32_t counter_;
    uint32_t lastDeliveredAt_;
    bool waiting_;
    bool firstMessage_;
};

CommReliableProgram program;

}

runtime::Program& commReliable() { return program; }

}
