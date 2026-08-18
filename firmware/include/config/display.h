#pragma once

#include <Adafruit_ST7789.h>
#include <stdint.h>

namespace config {

/**
 * @brief 屏幕初始化参数
 */
struct DisplayConfig {
    /** 屏幕宽度 */
    uint16_t width;
    /** 屏幕高度 */
    uint16_t height;
    /** 屏幕旋转方向 */
    uint8_t rotation;
};

/** 当前屏幕型号 */
using Display = Adafruit_ST7789;

/** 当前屏幕参数 */
constexpr DisplayConfig kDisplay = {135, 240, 1};

}
