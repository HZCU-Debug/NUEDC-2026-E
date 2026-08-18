#include <cassert>

#include "runtime/program_runner.h"

class RecordingProgram final : public runtime::Program {
public:
    RecordingProgram() : starts(0), updates(0), stops(0), state(nullptr) {}

    void start(Adafruit_GFX&, runtime::SystemState& systemState) override {
        ++starts;
        state = &systemState;
    }

    void update(Adafruit_GFX&, runtime::SystemState& systemState,
                ui::Event) override {
        ++updates;
        state = &systemState;
    }

    void stop(runtime::SystemState& systemState) override {
        ++stops;
        state = &systemState;
    }

    int starts;
    int updates;
    int stops;
    runtime::SystemState* state;
};

int main() {
    Adafruit_GFX display(240, 135);
    runtime::SystemState state;
    runtime::ProgramRunner runner(state);
    RecordingProgram first;
    RecordingProgram second;

    runner.start(first, display);
    runner.update(display, ui::Event::Select);
    assert(first.starts == 1);
    assert(first.updates == 1);
    assert(first.state == &state);

    runner.start(second, display);
    assert(first.stops == 1);
    assert(second.starts == 1);

    runner.update(display, ui::Event::None);
    runner.stop();
    runner.update(display, ui::Event::None);
    assert(second.updates == 1);
    assert(second.stops == 1);

    return 0;
}
