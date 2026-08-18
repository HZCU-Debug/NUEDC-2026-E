#pragma once

#include <Adafruit_GFX.h>

namespace ui {
namespace view {

/**
 * @brief 清屏并绘制页面标题
 * @param display 图形屏幕
 * @param title 页面标题
 */
inline void beginPage(Adafruit_GFX& display, const char* title) {
    display.fillScreen(0x0000);
    display.setTextSize(2);
    display.setTextColor(0x07FF);
    display.setCursor(6, 6);
    display.print(title);
}

/**
 * @brief 清除页面内容区域并恢复正文样式
 * @param display 图形屏幕
 */
inline void beginBody(Adafruit_GFX& display) {
    display.fillRect(0, 30, display.width(), display.height() - 30, 0x0000);
    display.setTextColor(0xFFFF);
    display.setTextSize(2);
}

}
}
