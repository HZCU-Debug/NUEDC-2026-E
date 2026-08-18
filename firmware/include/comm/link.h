#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

namespace comm {

/**
 * @brief 消息的投递方式
 */
enum class Delivery : uint8_t {
    /** 发送一次，不等待对端确认 */
    Unreliable,
    /** 持续重传，直到对端确认或调用 Link::cancel() */
    Reliable,
};

/**
 * @brief Link::send() 的受理结果
 */
enum class SendResult : uint8_t {
    /** 消息已经写入串口；可靠消息进入等待确认状态 */
    Accepted,
    /** 仍有可靠消息等待确认 */
    Busy,
    /** 链路未初始化，或消息类型、载荷指针、投递方式无效 */
    InvalidArgument,
    /** 载荷超过 Link 的 Capacity */
    PayloadTooLarge,
    /** 串口未能写入完整消息帧 */
    WriteFailed,
};

/**
 * @brief Link::poll() 返回的事件类型
 */
enum class EventType : uint8_t {
    /** 没有可处理的事件 */
    None,
    /** 收到一条有效消息 */
    Message,
    /** 待确认的可靠消息已经送达 */
    Delivered,
};

/**
 * @brief 收到的消息视图
 */
struct MessageView {
    MessageView()
        : type(0), delivery(Delivery::Unreliable), payload(NULL), size(0) {}

    /** 业务消息类型，范围 1–127 */
    uint8_t type;
    /** 发送端选择的投递方式 */
    Delivery delivery;
    /** 只读载荷；内存仅在下一次 Link::poll() 前有效 */
    const uint8_t* payload;
    /** 载荷字节数 */
    size_t size;
};

/**
 * @brief Link::poll() 返回的一次通信事件
 */
struct Event {
    Event() : type(EventType::None), message() {}

    /** 事件类型 */
    EventType type;
    /** EventType::Message 时有效的消息内容 */
    MessageView message;
};

/**
 * @brief Link 使用的 UART 和可靠投递配置
 */
struct LinkConfig {
    LinkConfig(uint32_t baudRate = 115200, int8_t rxPin = -1, int8_t txPin = -1,
               uint32_t retryIntervalMs = 50)
        : baudRate(baudRate),
          rxPin(rxPin),
          txPin(txPin),
          retryIntervalMs(retryIntervalMs) {}

    /** UART 波特率，必须与对端一致且不能为 0 */
    uint32_t baudRate;
    /** ESP32 RX 引脚，-1 表示使用串口默认引脚 */
    int8_t rxPin;
    /** ESP32 TX 引脚，-1 表示使用串口默认引脚 */
    int8_t txPin;
    /** 可靠消息未确认时的重传间隔，单位毫秒，不能为 0 */
    uint32_t retryIntervalMs;
};

namespace detail {

inline uint8_t crc8(const uint8_t* data, size_t size) {
    uint8_t crc = 0;
    for (size_t index = 0; index < size; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = static_cast<uint8_t>(crc & 0x80 ? (crc << 1) ^ 0x07 : crc << 1);
        }
    }
    return crc;
}

inline size_t cobsEncode(const uint8_t* input, size_t size, uint8_t* output) {
    size_t read = 0;
    size_t write = 1;
    size_t codeIndex = 0;
    uint8_t code = 1;

    while (read < size) {
        if (input[read] == 0) {
            output[codeIndex] = code;
            codeIndex = write++;
            code = 1;
            ++read;
        } else {
            output[write++] = input[read++];
            if (++code == 0xFF) {
                output[codeIndex] = code;
                codeIndex = write++;
                code = 1;
            }
        }
    }
    output[codeIndex] = code;
    return write;
}

inline bool cobsDecode(uint8_t* data, size_t size, size_t& decodedSize) {
    size_t read = 0;
    size_t write = 0;
    while (read < size) {
        const uint8_t code = data[read++];
        if (code == 0 || read + code - 1 > size) {
            return false;
        }
        for (uint8_t index = 1; index < code; ++index) {
            data[write++] = data[read++];
        }
        if (code != 0xFF && read < size) {
            data[write++] = 0;
        }
    }
    decodedSize = write;
    return true;
}

}

/**
 * @brief 通过 HardwareSerial 收发带校验的消息，并可选择可靠投递
 * @tparam Capacity 当前端点支持的最大业务载荷字节数，必须大于 0
 *
 * 收发双方必须使用相同的串口配置，并定期调用 poll() 处理接收、确认和重传。
 * 同一时刻最多有一条本端发送的可靠消息等待确认。
 */
template <size_t Capacity>
class Link {
public:
    /**
     * @brief 绑定硬件串口和链路配置
     * @param serial Arduino 硬件串口
     * @param config UART 和可靠投递配置
     */
    Link(HardwareSerial& serial, const LinkConfig& config = LinkConfig())
        : serial_(serial),
          config_(config),
          started_(false),
          waiting_(false),
          nextSequence_(0),
          pendingSize_(0),
          lastSentAt_(0),
          receivedSize_(0),
          discarding_(false),
          hasReceivedSequence_(false),
          lastReceivedSequence_(0) {
        static_assert(Capacity > 0, "Link capacity must be positive");
    }

    /**
     * @brief 初始化 UART 并清空当前收发状态
     * @return 配置有效时返回 true
     */
    bool begin() {
        if (config_.baudRate == 0 || config_.retryIntervalMs == 0) {
            return false;
        }
        serial_.begin(config_.baudRate, SERIAL_8N1, config_.rxPin, config_.txPin);
        started_ = true;
        waiting_ = false;
        pendingSize_ = 0;
        receivedSize_ = 0;
        discarding_ = false;
        return true;
    }

    /**
     * @brief 发送一条业务消息
     * @param type 业务消息类型，范围 1–127
     * @param payload 载荷；size 为 0 时可以为 NULL
     * @param size 载荷字节数，不能超过 Capacity
     * @param delivery 投递方式
     * @return 消息受理结果；Accepted 只表示消息帧已写入串口
     */
    SendResult send(uint8_t type, const uint8_t* payload, size_t size, Delivery delivery) {
        if (!started_) {
            return SendResult::InvalidArgument;
        }
        if (waiting_) {
            return SendResult::Busy;
        }
        if (type == 0 || type > 0x7F || (size != 0 && payload == NULL) ||
            (delivery != Delivery::Unreliable && delivery != Delivery::Reliable)) {
            return SendResult::InvalidArgument;
        }
        if (size > Capacity) {
            return SendResult::PayloadTooLarge;
        }

        uint8_t raw[Capacity + 3];
        const bool reliable = delivery == Delivery::Reliable;
        raw[0] = static_cast<uint8_t>(type | (reliable ? 0x80 : 0));
        const size_t payloadOffset = reliable ? 2 : 1;
        if (reliable) {
            raw[1] = nextSequence_;
        }
        for (size_t index = 0; index < size; ++index) {
            raw[index + payloadOffset] = payload[index];
        }
        const size_t rawSize = size + payloadOffset + 1;
        raw[rawSize - 1] = detail::crc8(raw, rawSize - 1);

        uint8_t* encoded = pending_;
        const size_t encodedSize = detail::cobsEncode(raw, rawSize, encoded);
        encoded[encodedSize] = 0;
        if (serial_.write(encoded, encodedSize + 1) != encodedSize + 1) {
            return SendResult::WriteFailed;
        }
        if (reliable) {
            waiting_ = true;
            pendingSize_ = encodedSize + 1;
            lastSentAt_ = millis();
            ++nextSequence_;
        }
        return SendResult::Accepted;
    }

    /**
     * @brief 处理串口输入、可靠消息确认和定时重传
     * @return 一次通信事件；没有事件时 type 为 EventType::None
     *
     * 每次最多返回一个事件，应在 loop() 中持续调用。返回消息中的载荷仅在下一次
     * poll() 前有效。
     */
    Event poll() {
        if (!started_) {
            return Event();
        }
        while (serial_.available()) {
            const uint8_t byte = static_cast<uint8_t>(serial_.read());
            if (byte == 0) {
                if (!discarding_ && receivedSize_ != 0) {
                    size_t decodedSize = 0;
                    const bool decoded = detail::cobsDecode(received_, receivedSize_, decodedSize);
                    receivedSize_ = 0;
                    if (decoded && decodedSize >= 2 &&
                        detail::crc8(received_, decodedSize - 1) ==
                            received_[decodedSize - 1]) {
                        const Event event = handleFrame(decodedSize);
                        if (event.type != EventType::None) {
                            return event;
                        }
                    }
                }
                receivedSize_ = 0;
                discarding_ = false;
            } else if (!discarding_) {
                if (receivedSize_ < kFrameCapacity) {
                    received_[receivedSize_++] = byte;
                } else {
                    receivedSize_ = 0;
                    discarding_ = true;
                }
            }
        }

        const unsigned long now = millis();
        if (waiting_ && now - lastSentAt_ >= config_.retryIntervalMs) {
            serial_.write(pending_, pendingSize_);
            lastSentAt_ = now;
        }
        return Event();
    }

    /**
     * @brief 停止等待并重传当前可靠消息
     *
     * 已写入串口的消息无法撤回；没有消息等待确认时调用不产生效果。
     */
    void cancel() {
        waiting_ = false;
        pendingSize_ = 0;
    }

private:
    static const size_t kFrameCapacity = Capacity + 3 + (Capacity + 3) / 254 + 2;

    Event handleFrame(size_t size) {
        Event event;
        const uint8_t wireType = received_[0];
        if (wireType == 0) {
            if (size == 3 && waiting_ &&
                received_[1] == static_cast<uint8_t>(nextSequence_ - 1)) {
                waiting_ = false;
                pendingSize_ = 0;
                event.type = EventType::Delivered;
            }
            return event;
        }

        const bool reliable = wireType & 0x80;
        const uint8_t type = static_cast<uint8_t>(wireType & 0x7F);
        if (type == 0 || size < (reliable ? 3U : 2U)) {
            return event;
        }

        const size_t payloadOffset = reliable ? 2 : 1;
        if (size - payloadOffset - 1 > Capacity) {
            return event;
        }
        if (reliable) {
            const uint8_t sequence = received_[1];
            sendReceipt(sequence);
            if (hasReceivedSequence_ && sequence == lastReceivedSequence_) {
                return event;
            }
            hasReceivedSequence_ = true;
            lastReceivedSequence_ = sequence;
        }

        event.type = EventType::Message;
        event.message.type = type;
        event.message.delivery = reliable ? Delivery::Reliable : Delivery::Unreliable;
        event.message.payload = &received_[payloadOffset];
        event.message.size = size - payloadOffset - 1;
        return event;
    }

    void sendReceipt(uint8_t sequence) {
        uint8_t raw[] = {0, sequence, 0};
        uint8_t encoded[5];
        raw[2] = detail::crc8(raw, 2);
        const size_t encodedSize = detail::cobsEncode(raw, sizeof(raw), encoded);
        encoded[encodedSize] = 0;
        serial_.write(encoded, encodedSize + 1);
    }

    HardwareSerial& serial_;
    LinkConfig config_;
    bool started_;
    bool waiting_;
    uint8_t nextSequence_;
    size_t pendingSize_;
    unsigned long lastSentAt_;
    size_t receivedSize_;
    bool discarding_;
    bool hasReceivedSequence_;
    uint8_t lastReceivedSequence_;
    uint8_t pending_[kFrameCapacity];
    uint8_t received_[kFrameCapacity];
};

}
