#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <initializer_list>
#include <vector>

#define SERIAL_8N1 0
#define HIGH 1
#define LOW 0
#define OUTPUT 1

inline std::vector<uint8_t>& pinValues() {
    static std::vector<uint8_t> values(40, LOW);
    return values;
}

inline void pinMode(uint8_t, uint8_t) {}

inline void digitalWrite(uint8_t pin, uint8_t value) {
    pinValues()[pin] = value;
}

inline unsigned long millis() {
    static unsigned long now = 0;
    return now++;
}

inline unsigned long& delayedMilliseconds() {
    static unsigned long value = 0;
    return value;
}

inline void delay(unsigned long milliseconds) {
    delayedMilliseconds() += milliseconds;
}

class HardwareSerial {
public:
    HardwareSerial() : rxPin(-1), txPin(-1) {}

    void begin(unsigned long) {}

    void begin(unsigned long, uint32_t, int8_t newRxPin, int8_t newTxPin) {
        rxPin = newRxPin;
        txPin = newTxPin;
    }

    bool setPins(int8_t newRxPin, int8_t newTxPin, int8_t = -1,
                 int8_t = -1) {
        if (newRxPin >= 0) rxPin = newRxPin;
        if (newTxPin >= 0) txPin = newTxPin;
        return true;
    }

    size_t write(const uint8_t* data, size_t size) {
        transmitted.insert(transmitted.end(), data, data + size);
        if (!responses.empty()) {
            received.insert(received.end(), responses.front().begin(), responses.front().end());
            responses.pop_front();
        }
        return size;
    }

    int available() const { return static_cast<int>(received.size()); }

    int read() {
        if (received.empty()) {
            return -1;
        }
        const uint8_t value = received.front();
        received.pop_front();
        return value;
    }

    void flush() {}

    void print(const char* text) {
        const size_t size = std::strlen(text);
        transmitted.insert(transmitted.end(), text, text + size);
    }

    void respondWith(std::initializer_list<uint8_t> response) {
        responses.emplace_back(response);
    }

    void receive(std::initializer_list<uint8_t> data) {
        received.insert(received.end(), data.begin(), data.end());
    }

    void receive(const std::vector<uint8_t>& data) {
        received.insert(received.end(), data.begin(), data.end());
    }

    std::vector<uint8_t> transmitted;
    int8_t rxPin;
    int8_t txPin;

private:
    std::deque<uint8_t> received;
    std::deque<std::vector<uint8_t>> responses;
};
