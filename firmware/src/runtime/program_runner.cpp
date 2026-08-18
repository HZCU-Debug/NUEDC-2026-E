#include "runtime/program_runner.h"

namespace runtime {

ProgramRunner::ProgramRunner(SystemState& state) : state_(state), active_(nullptr) {}

void ProgramRunner::start(Program& program, Adafruit_GFX& display) {
    stop();
    active_ = &program;
    active_->start(display, state_);
}

void ProgramRunner::update(Adafruit_GFX& display, ui::Event event) {
    if (active_ != nullptr) {
        active_->update(display, state_, event);
    }
}

void ProgramRunner::stop() {
    if (active_ == nullptr) {
        return;
    }
    active_->stop(state_);
    active_ = nullptr;
}

}
