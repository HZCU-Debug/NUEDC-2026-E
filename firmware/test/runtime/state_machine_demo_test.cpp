#include <cassert>
#include <string>

#include "program/demo/state_machine_demo.h"

static std::string transmittedText(const HardwareSerial& serial) {
    return std::string(serial.transmitted.begin(), serial.transmitted.end());
}

int main() {
    Adafruit_GFX display(240, 135);
    HardwareSerial serial;
    runtime::SystemState systemState;
    runtime::StateMachine<program::DemoState> stateMachine(
        program::DemoState::EntryA);
    program::StateMachineDemoProgram program(serial, stateMachine);

    program.configure(program::DemoConfiguration(program::DemoState::EntryA));
    program.start(display, systemState);
    program.update(display, systemState, ui::Event::None);
    program.update(display, systemState, ui::Event::None);
    program.update(display, systemState, ui::Event::None);
    assert(transmittedText(serial) ==
           "state=entry-a\n"
           "handle=entry-a\ntransition=shared\n"
           "handle=shared\ntransition=loop\n"
           "handle=loop\ntransition=shared\n");

    serial.transmitted.clear();
    program.configure(program::DemoConfiguration(program::DemoState::EntryB));
    program.start(display, systemState);
    program.update(display, systemState, ui::Event::None);
    assert(transmittedText(serial) ==
           "state=entry-b\n"
           "handle=entry-b\ntransition=shared\n");

    return 0;
}
