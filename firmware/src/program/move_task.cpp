#include "program/move_task.h"

#include <stdio.h>

#include "config/hardware.h"
#include "motion/gantry.h"
#include "ui/view.h"

extern HardwareSerial Serial;
extern HardwareSerial Serial1;

namespace program {
namespace {

const uint32_t kSerialBaudRate = 115200;

int16_t decodeInt16(const uint8_t* data) {
    int32_t value =
        (static_cast<int32_t>(data[0]) << 8) | static_cast<int32_t>(data[1]);
    if (value >= 0x8000) {
        value -= 0x10000;
    }
    return static_cast<int16_t>(value);
}

const char* stageLabel(motion::RunnerStage stage) {
    switch (stage) {
        case motion::RunnerStage::SourceXY:
            return "Source XY";
        case motion::RunnerStage::SourceDown:
            return "Source Z down";
        case motion::RunnerStage::SourceUp:
            return "Source Z up";
        case motion::RunnerStage::TargetXY:
            return "Target XY";
        case motion::RunnerStage::TargetDown:
            return "Target Z down";
        case motion::RunnerStage::TargetUp:
            return "Target Z up";
        case motion::RunnerStage::ReturnZ:
            return "Return Z 0";
        case motion::RunnerStage::ReturnXY:
            return "Return XY 0,0";
        case motion::RunnerStage::Idle:
            return "Idle";
    }
    return "Unknown";
}

const char* errorLabel(motor::Error error) {
    switch (error) {
        case motor::Error::None:
            return "ok";
        case motor::Error::NotStarted:
            return "not started";
        case motor::Error::InvalidArgument:
            return "bad argument";
        case motor::Error::WriteFailed:
            return "write failed";
        case motor::Error::Timeout:
            return "timeout";
        case motor::Error::InvalidResponse:
            return "invalid";
        case motor::Error::DeviceRejected:
            return "rejected";
    }
    return "unknown";
}

const char* taskResultLabel(motion::TaskResult result) {
    switch (result) {
        case motion::TaskResult::InvalidTask:
            return "Invalid task";
        case motion::TaskResult::NotCalibrated:
            return "Not calibrated";
        case motion::TaskResult::Completed:
            return "Completed";
        case motion::TaskResult::None:
            return "No error";
    }
    return "Unknown error";
}

bool sameDiagnostic(const motion::PollDiagnostic& left,
                    const motion::PollDiagnostic& right) {
    return left.axis == right.axis && left.error == right.error &&
           left.detail == right.detail && left.flags == right.flags;
}

class HomeProgram final : public runtime::Program {
public:
    HomeProgram() : runner_(motion::systemGantry()), returning_(false) {}

    void start(Adafruit_GFX& display, runtime::SystemState& state) override {
        ui::view::beginPage(display, "Home");
        returning_ = runner_.returnHome(state.now).result ==
                     motion::TaskResult::None;
        render(display, returning_ ? "Returning" : "Return failed");
    }

    void update(Adafruit_GFX& display, runtime::SystemState& state,
                ui::Event) override {
        if (!returning_) {
            return;
        }
        const motion::RunnerEvent event = runner_.update(state.now);
        if (event.result != motion::TaskResult::None) {
            returning_ = false;
            render(display, "Return failed");
        } else if (runner_.stage() == motion::RunnerStage::Idle) {
            returning_ = false;
            render(display, "At origin");
        }
    }

    bool readyToExit() const override { return !returning_; }

    void stop(runtime::SystemState&) override { runner_.stop(); }

private:
    void render(Adafruit_GFX& display, const char* status) const {
        ui::view::beginBody(display);
        display.setCursor(6, 42);
        display.print(status);
        display.setCursor(6, 72);
        display.print("S4 Back");
    }

    motion::PickPlaceRunner runner_;
    bool returning_;
};

}

MoveTaskProgram::MoveTaskProgram(HardwareSerial& serial,
                                 motion::PickPlaceRunner& runner,
                                 const comm::LinkConfig& config,
                                 bool logVisionCoordinates)
    : link_(serial, config),
      runner_(runner),
      task_(),
      receivedPieces_(0),
      receivedMask_(0),
      selectedMode_(0xFF),
      state_(State::Finished),
      renderedStage_(motion::RunnerStage::Idle),
      renderedPollDiagnostic_(),
      now_(0),
      exitReady_(true),
      calibrationFinishSent_(false),
      logVisionCoordinates_(logVisionCoordinates),
      usesVisionUart_(&serial == &Serial1) {}

void MoveTaskProgram::configure(uint8_t mode) { selectedMode_ = mode; }

void MoveTaskProgram::start(Adafruit_GFX& display,
                            runtime::SystemState& state) {
    now_ = state.now;
    renderedStage_ = motion::RunnerStage::Idle;
    renderedPollDiagnostic_ = motion::PollDiagnostic();
    exitReady_ = true;
    calibrationFinishSent_ = false;
    ui::view::beginPage(display,
                        selectedMode_ == 0 ? "Cal Vision" : "Move Task");
    if (selectedMode_ > 3 ||
        (usesVisionUart_ &&
         (config::kPins.visionRx < 0 || config::kPins.visionTx < 0))) {
        state_ = State::Error;
        render(display, "Vision UART error");
        return;
    }
    if (!link_.begin()) {
        state_ = State::Error;
        render(display, "Link error");
        return;
    }

    task_ = motion::MoveTask();
    receivedPieces_ = 0;
    receivedMask_ = 0;
    if (!requestMode()) {
        state_ = State::Error;
        render(display, "Mode send failed");
        return;
    }
    state_ = State::SendingMode;
    render(display, "Sending mode");
}

void MoveTaskProgram::update(Adafruit_GFX& display,
                             runtime::SystemState& state,
                             ui::Event inputEvent) {
    now_ = state.now;
    if (state_ == State::Returning) {
        const motion::RunnerEvent returned = runner_.update(state.now);
        if (returned.result != motion::TaskResult::None) {
            state_ = State::Error;
            exitReady_ = true;
            render(display, "Return failed");
            return;
        }
        if (runner_.stage() == motion::RunnerStage::Idle) {
            state_ = State::Finished;
            exitReady_ = true;
            render(display, "At origin");
            if (selectedMode_ >= 1 && selectedMode_ <= 3) {
                digitalWrite(config::kPins.buzzer, LOW);
                delay(config::kBuzzerDurationMs);
                digitalWrite(config::kPins.buzzer, HIGH);
            }
            return;
        }
        if (runner_.stage() != renderedStage_) {
            renderRunner(display);
        } else if (!sameDiagnostic(runner_.pollDiagnostic(),
                                   renderedPollDiagnostic_)) {
            renderRunner(display);
        }
        return;
    }

    const comm::Event event = link_.poll();
    if (state_ == State::SendingMode) {
        if (event.type == comm::EventType::Delivered) {
            if (selectedMode_ == 0 && calibrationFinishSent_) {
                if (returnHome()) {
                    renderRunner(display);
                } else {
                    render(display, "Return failed");
                }
                return;
            }
            state_ = State::WaitingTask;
            if (selectedMode_ == 0) {
                render(display, calibrationFinishSent_ ? "Finish sent"
                                                       : "S3 Finish");
            } else {
                render(display, "Waiting vision");
            }
        }
        return;
    }

    if (selectedMode_ == 0 && state_ == State::WaitingTask) {
        if (inputEvent != ui::Event::Select) {
            return;
        }
        calibrationFinishSent_ = true;
        if (!requestMode()) {
            state_ = State::Error;
            render(display, "Mode send failed");
            return;
        }
        state_ = State::SendingMode;
        render(display, "Sending finish");
        return;
    }

    if (state_ == State::WaitingTask &&
        event.type == comm::EventType::Message) {
        if (event.message.type == kPieceCountMessage) {
            if (!receiveCount(event.message)) {
                finish(display,
                       motion::RunnerEvent(motion::TaskResult::InvalidTask));
                return;
            }
            render(display, "Waiting pieces");
        } else if (event.message.type == kPieceDataMessage) {
            if (!receivePiece(event.message)) {
                finish(display,
                       motion::RunnerEvent(motion::TaskResult::InvalidTask));
                return;
            }
            if (receivedPieces_ != task_.count) {
                return;
            }
            const motion::RunnerEvent started =
                runner_.start(task_, state.calibration, state.now);
            if (started.result != motion::TaskResult::None) {
                finish(display, started);
                return;
            }
            state_ = State::Running;
            renderedStage_ = motion::RunnerStage::Idle;
            renderRunner(display);
        }
    }

    if (state_ != State::Running) {
        return;
    }
    const motion::RunnerEvent result = runner_.update(state.now);
    if (result.result != motion::TaskResult::None) {
        finish(display, result);
    } else if (runner_.stage() != renderedStage_ ||
               !sameDiagnostic(runner_.pollDiagnostic(),
                               renderedPollDiagnostic_)) {
        renderRunner(display);
    }
}

void MoveTaskProgram::requestExit() {
    link_.cancel();
    if (state_ == State::Finished) {
        exitReady_ = true;
        return;
    }
    if (state_ == State::Returning) {
        return;
    }
    returnHome();
}

bool MoveTaskProgram::returnHome() {
    const motion::RunnerEvent returning = runner_.returnHome(now_);
    if (returning.result != motion::TaskResult::None) {
        state_ = State::Error;
        exitReady_ = true;
        return false;
    }
    state_ = State::Returning;
    renderedStage_ = motion::RunnerStage::Idle;
    exitReady_ = false;
    return true;
}

bool MoveTaskProgram::readyToExit() const { return exitReady_; }

void MoveTaskProgram::stop(runtime::SystemState&) {
    if (state_ == State::Running || state_ == State::Returning) {
        runner_.stop();
    }
    link_.cancel();
    task_ = motion::MoveTask();
    receivedPieces_ = 0;
    receivedMask_ = 0;
    calibrationFinishSent_ = false;
    state_ = State::Finished;
}

bool MoveTaskProgram::requestMode() {
    return link_.send(kModeRequestMessage, &selectedMode_, 1,
                      comm::Delivery::Reliable) ==
           comm::SendResult::Accepted;
}

bool MoveTaskProgram::receiveCount(const comm::MessageView& message) {
    if (message.delivery != comm::Delivery::Reliable || message.size != 1 ||
        message.payload[0] == 0 || message.payload[0] > 4) {
        return false;
    }

    task_ = motion::MoveTask();
    task_.count = message.payload[0];
    receivedPieces_ = 0;
    receivedMask_ = 0;
    return true;
}

bool MoveTaskProgram::receivePiece(const comm::MessageView& message) {
    if (message.delivery != comm::Delivery::Reliable || message.size != 11 ||
        task_.count == 0) {
        return false;
    }

    const uint8_t id = message.payload[0];
    if (id == 0 || id > task_.count) {
        return false;
    }
    const uint8_t mask = static_cast<uint8_t>(1U << (id - 1));
    if ((receivedMask_ & mask) != 0) {
        return false;
    }

    const int16_t sourceX = decodeInt16(message.payload + 1);
    const int16_t sourceY = decodeInt16(message.payload + 3);
    const int16_t targetX = decodeInt16(message.payload + 5);
    const int16_t targetY = decodeInt16(message.payload + 7);
    const int16_t rotationHundredths = decodeInt16(message.payload + 9);
    if (sourceX < 0 || sourceX > 210 || targetX < 0 || targetX > 210 ||
        sourceY < 0 || sourceY > 297 || targetY < 0 || targetY > 297) {
        return false;
    }

    motion::PieceMove& piece = task_.pieces[id - 1];
    piece.source = motion::PaperPoint(static_cast<uint16_t>(sourceX),
                                      static_cast<uint16_t>(sourceY));
    piece.target = motion::PaperPoint(static_cast<uint16_t>(targetX),
                                      static_cast<uint16_t>(targetY));
    piece.rotationDegrees = static_cast<float>(rotationHundredths) / 100.0f;
    if (logVisionCoordinates_) {
        char line[72];
        snprintf(line, sizeof(line),
                 "VISION id=%u S=(%d,%d)mm T=(%d,%d)mm\n",
                 static_cast<unsigned>(id), sourceX, sourceY, targetX,
                 targetY);
        Serial.print(line);
    }
    receivedMask_ = static_cast<uint8_t>(receivedMask_ | mask);
    ++receivedPieces_;
    return true;
}

void MoveTaskProgram::finish(Adafruit_GFX& display,
                             const motion::RunnerEvent& event) {
    if (event.result == motion::TaskResult::Completed) {
        if (returnHome()) {
            renderRunner(display);
        } else {
            render(display, "Return failed");
        }
        return;
    }
    state_ = State::Error;
    renderFailure(display, event);
}

void MoveTaskProgram::renderFailure(
    Adafruit_GFX& display, const motion::RunnerEvent& event) const {
    ui::view::beginBody(display);
    display.setCursor(6, 34);
    display.print(taskResultLabel(event.result));
    display.setCursor(6, 58);
    if (event.pieceIndex == 0xFF) {
        display.print(stageLabel(renderedStage_));
    } else {
        char context[24];
        snprintf(context, sizeof(context), "P%u %s",
                 static_cast<unsigned>(event.pieceIndex + 1),
                 stageLabel(renderedStage_));
        display.print(context);
    }

    display.setCursor(6, 110);
    display.print("S4 Back");
}

void MoveTaskProgram::render(Adafruit_GFX& display, const char* status) const {
    ui::view::beginBody(display);
    display.setCursor(6, 42);
    display.print(status);
    display.setCursor(6, 72);
    display.print("S4 Back");
}

void MoveTaskProgram::renderRunner(Adafruit_GFX& display) {
    renderedStage_ = runner_.stage();
    renderedPollDiagnostic_ = runner_.pollDiagnostic();
    ui::view::beginBody(display);
    if (renderedStage_ != motion::RunnerStage::ReturnZ &&
        renderedStage_ != motion::RunnerStage::ReturnXY) {
        char piece[16];
        snprintf(piece, sizeof(piece), "Piece %u/%u",
                 static_cast<unsigned>(runner_.pieceIndex() + 1),
                 static_cast<unsigned>(runner_.pieceCount()));
        display.setCursor(6, 38);
        display.print(piece);
    }
    display.setCursor(6, 66);
    display.print(stageLabel(renderedStage_));
    display.setCursor(6, 94);
    const char axis[] = {renderedPollDiagnostic_.axis, '\0'};
    display.print(axis);
    if (renderedPollDiagnostic_.error == motor::Error::None) {
        char status[12];
        snprintf(status, sizeof(status), " flags %02X",
                 static_cast<unsigned>(renderedPollDiagnostic_.flags));
        display.print(status);
    } else {
        display.print(" query ");
        display.print(errorLabel(renderedPollDiagnostic_.error));
    }
    display.setCursor(6, 118);
    display.print("S4 Back");
}

MoveTaskProgram& moveTask() {
    static motion::PickPlaceRunner runner(motion::systemGantry());
    static MoveTaskProgram program(
        Serial1, runner,
        comm::LinkConfig(kSerialBaudRate, config::kPins.visionRx,
                         config::kPins.visionTx),
        true);
    return program;
}

MoveTaskProgram& moveTaskDemo() {
    static motion::PickPlaceRunner runner(motion::systemGantry());
    static MoveTaskProgram program(Serial, runner);
    return program;
}

runtime::Program& home() {
    static HomeProgram program;
    return program;
}

}
