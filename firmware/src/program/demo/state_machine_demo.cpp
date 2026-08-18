#include "program/demo/state_machine_demo.h"

#include "ui/view.h"

namespace program {
namespace {

const uint32_t kSerialBaudRate = 115200;

const char* stateName(DemoState state) {
    switch (state) {
        case DemoState::EntryA:
            return "entry-a";
        case DemoState::EntryB:
            return "entry-b";
        case DemoState::Shared:
            return "shared";
        case DemoState::Loop:
            return "loop";
    }
    return "unknown";
}

}

StateMachineDemoProgram::StateMachineDemoProgram(
    HardwareSerial& serial, runtime::StateMachine<DemoState>& stateMachine)
    : serial_(serial),
      stateMachine_(stateMachine),
      configuration_(DemoState::EntryA) {}

void StateMachineDemoProgram::configure(
    const DemoConfiguration& configuration) {
    configuration_ = configuration;
}

void StateMachineDemoProgram::start(Adafruit_GFX& display,
                                    runtime::SystemState&) {
    serial_.begin(kSerialBaudRate);
    stateMachine_.reset(configuration_.entry);
    trace("state", stateMachine_.state());
    ui::view::beginPage(display, "State Machine");
    render(display);
}

void StateMachineDemoProgram::update(Adafruit_GFX& display,
                                     runtime::SystemState&, ui::Event) {
    switch (stateMachine_.state()) {
        case DemoState::EntryA:
            trace("handle", DemoState::EntryA);
            transitionTo(DemoState::Shared);
            break;
        case DemoState::EntryB:
            trace("handle", DemoState::EntryB);
            transitionTo(DemoState::Shared);
            break;
        case DemoState::Shared:
            trace("handle", DemoState::Shared);
            transitionTo(DemoState::Loop);
            break;
        case DemoState::Loop:
            trace("handle", DemoState::Loop);
            transitionTo(DemoState::Shared);
            break;
    }
    render(display);
}

void StateMachineDemoProgram::trace(const char* action, DemoState state) {
    serial_.print(action);
    serial_.print("=");
    serial_.print(stateName(state));
    serial_.print("\n");
}

void StateMachineDemoProgram::transitionTo(DemoState next) {
    stateMachine_.transitionTo(next);
    trace("transition", next);
}

void StateMachineDemoProgram::render(Adafruit_GFX& display) const {
    ui::view::beginBody(display);
    display.setCursor(6, 42);
    display.print(stateName(stateMachine_.state()));
}

}
