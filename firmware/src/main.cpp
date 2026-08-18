/**
 * @file main.cpp
 * @brief Arduino 固件入口、屏幕菜单和按钮输入
 */
#include <Arduino.h>
#include <SPI.h>

#include "config/display.h"
#include "config/hardware.h"
#include "config/parameters.h"
#include "program/move_task.h"
#include "program/programs.h"
#include "runtime/program_runner.h"
#include "storage/calibration.h"
#include "ui/menu.h"

namespace {

class Button {
public:
    Button(int8_t pin, ui::Event event)
        : pin_(pin),
          event_(event),
          lastReading_(false),
          stablePressed_(false),
          changedAt_(0) {}

    void begin() {
        pinMode(pin_, config::kButtonsUseInternalPullup ? INPUT_PULLUP : INPUT);
        stablePressed_ = digitalRead(pin_) == LOW;
        lastReading_ = stablePressed_;
        changedAt_ = millis();
    }

    ui::Event poll(uint32_t now) {
        const bool pressed = digitalRead(pin_) == LOW;
        if (pressed != lastReading_) {
            lastReading_ = pressed;
            changedAt_ = now;
        }
        if (pressed == stablePressed_ ||
            now - changedAt_ < config::kButtonDebounceMs) {
            return ui::Event::None;
        }

        stablePressed_ = pressed;
        return pressed ? event_ : ui::Event::None;
    }

private:
    int8_t pin_;
    ui::Event event_;
    bool lastReading_;
    bool stablePressed_;
    uint32_t changedAt_;
};

config::Display display(config::kPins.displayCs, config::kPins.displayDc,
                        config::kPins.displayReset);
Button buttons[] = {
    Button(config::kPins.upButton, ui::Event::Up),
    Button(config::kPins.downButton, ui::Event::Down),
    Button(config::kPins.selectButton, ui::Event::Select),
    Button(config::kPins.backButton, ui::Event::Back),
};
ui::Menu* menu = nullptr;
runtime::SystemState systemState;
runtime::ProgramRunner programRunner(systemState);

ui::Event readEvent(uint32_t now) {
    ui::Event result = ui::Event::None;
    for (size_t index = 0; index < sizeof(buttons) / sizeof(buttons[0]); ++index) {
        const ui::Event event = buttons[index].poll(now);
        if (result == ui::Event::None && event != ui::Event::None) {
            result = event;
        }
    }
    return result;
}

}

void setup() {
    for (size_t index = 0; index < sizeof(buttons) / sizeof(buttons[0]); ++index) {
        buttons[index].begin();
    }

    SPI.begin(config::kPins.displayClock, -1, config::kPins.displayData,
              config::kPins.displayCs);
    display.init(config::kDisplay.width, config::kDisplay.height);
    display.setRotation(config::kDisplay.rotation);
    display.setTextWrap(false);
    pinMode(config::kPins.displayBacklight, OUTPUT);
    digitalWrite(config::kPins.displayBacklight, HIGH);
    pinMode(config::kPins.electromagnet, OUTPUT);
    digitalWrite(config::kPins.electromagnet, LOW);
    digitalWrite(config::kPins.buzzer, HIGH);
    pinMode(config::kPins.buzzer, OUTPUT);

    storage::loadCalibration(systemState.calibration);
    program::initializeMotors();

    static ui::ConfiguredItem<program::MoveTaskProgram, uint8_t>
        visionCalibrationItem("Cal Vision", programRunner,
                              program::moveTask(), 0);
    static ui::Item zCalibrationItem(
        "Cal Z", programRunner, program::zCalibration());
    static ui::Item paperCalibrationItem(
        "Cal XY", programRunner, program::paperCalibration());
    static ui::Item motorTestItem("Test", programRunner,
                                  program::motorTest());
    static ui::ConfiguredItem<program::MoveTaskProgram, uint8_t> moveTaskDemoItem(
        "Mock", programRunner, program::moveTaskDemo(), 1);
    static ui::ConfiguredItem<program::MoveTaskProgram, uint8_t> task1Item(
        "Sol 1", programRunner, program::moveTask(), 1);
    static ui::ConfiguredItem<program::MoveTaskProgram, uint8_t> task21Item(
        "Sol 2-1", programRunner, program::moveTask(), 2);
    static ui::ConfiguredItem<program::MoveTaskProgram, uint8_t> task22Item(
        "Sol 2-2", programRunner, program::moveTask(), 3);
    static ui::Item homeItem("Home", programRunner, program::home());
    static ui::Item* items[] = {
        &visionCalibrationItem,
        &zCalibrationItem,
        &paperCalibrationItem,
        &task1Item,
        &task21Item,
        &task22Item,
        &homeItem,
        &motorTestItem,
        &moveTaskDemoItem,
    };
    static ui::Menu appMenu(display, "NUEDC", items,
                            sizeof(items) / sizeof(items[0]));
    menu = &appMenu;
    menu->begin();
}

void loop() {
    systemState.now = millis();
    menu->loop(readEvent(systemState.now));
}
