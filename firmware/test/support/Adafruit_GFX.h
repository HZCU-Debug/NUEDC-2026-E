#pragma once

#include <cstdint>
#include <string>
#include <vector>

class Adafruit_GFX {
public:
    Adafruit_GFX(int16_t width, int16_t height)
        : fillColor(0), width_(width), height_(height) {}

    int16_t width() const { return width_; }
    int16_t height() const { return height_; }

    void fillScreen(uint16_t color) {
        fillColor = color;
        printed.clear();
    }

    void fillRect(int16_t, int16_t, int16_t, int16_t, uint16_t) {}
    void setTextColor(uint16_t) {}
    void setTextSize(uint8_t) {}
    void setCursor(int16_t, int16_t) {}
    void print(const char* text) { printed.push_back(text); }
    void print(float value, int) { printed.push_back(std::to_string(value)); }

    uint16_t fillColor;
    std::vector<std::string> printed;

private:
    int16_t width_;
    int16_t height_;
};
