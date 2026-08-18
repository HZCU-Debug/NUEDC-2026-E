/**
 * @file calibration.cpp
 * @brief 三轴电机零点初始化与设备校准程序
 */
#include "programs.h"

#include <Arduino.h>

#include "config/parameters.h"
#include "motion/gantry.h"
#include "storage/calibration.h"
#include "ui/view.h"

namespace program {
namespace {

const char* motorError() {
    switch (motion::systemGantry().pollDiagnostic().axis) {
        case 'X':
            return "Motor X error";
        case 'Y':
            return "Motor Y error";
        case 'Z':
            return "Motor Z error";
        case 'R':
            return "Motor R error";
        default:
            return "Motor error";
    }
}

bool returnZToZero() {
    return motion::systemGantry().setEnabled() &&
           motion::systemGantry().moveZ(
               0.0f,
               motor::MotionOptions(config::kCalibrationReturnRpm,
                                    config::kCalibrationReturnAcceleration));
}

bool returnAllToZero() {
    const motor::MotionOptions options(config::kCalibrationReturnRpm,
                                       config::kCalibrationReturnAcceleration);
    return motion::systemGantry().setEnabled() &&
           motion::systemGantry().moveZ(0.0f, options) &&
           motion::systemGantry().moveXYRotation(0.0f, 0.0f, 0.0f, options,
                                                 options);
}

class ZCalibrationProgram final : public runtime::Program {
public:
    ZCalibrationProgram()
        : ready_(false), saved_(false), position_(0.0f), message_(nullptr) {}

    void start(Adafruit_GFX& display, runtime::SystemState&) override {
        saved_ = false;
        position_ = 0.0f;
        ready_ = static_cast<bool>(motion::systemGantry().setEnabled(false));
        message_ = ready_ ? nullptr : motorError();
        ui::view::beginPage(display, "Z Calibration");
        render(display);
    }

    void update(Adafruit_GFX& display, runtime::SystemState& state,
                ui::Event event) override {
        if (!ready_ || saved_ || event != ui::Event::Select) {
            return;
        }

        const motor::Result<float> position = motion::systemGantry().readZ();
        if (!position) {
            message_ = "Read error";
            render(display);
            return;
        }

        storage::Calibration updated = state.calibration;
        updated.zDown = position.value;
        updated.zValid = true;
        if (!storage::saveCalibration(updated)) {
            message_ = "Storage error";
            render(display);
            return;
        }

        state.calibration = updated;
        position_ = position.value;
        saved_ = true;
        message_ = "Saved";
        render(display);
    }

    void stop(runtime::SystemState&) override { returnAllToZero(); }

private:
    void render(Adafruit_GFX& display) const {
        ui::view::beginBody(display);
        display.setCursor(6, 38);
        if (message_ != nullptr) {
            display.print(message_);
        } else {
            display.print("Move Z by hand");
        }
        display.setCursor(6, 64);
        if (saved_) {
            display.print("Z: ");
            display.print(position_, 2);
        } else {
            display.print("S3 Save");
        }
        display.setCursor(6, 90);
        display.print("S4 Back");
    }

    bool ready_;
    bool saved_;
    float position_;
    const char* message_;
};

enum class PaperStage : uint8_t {
    Origin,
    Opposite,
    Done,
};

class PaperCalibrationProgram final : public runtime::Program {
public:
    PaperCalibrationProgram()
        : stage_(PaperStage::Origin),
          ready_(false),
          x0_(0.0f),
          y0_(0.0f),
          x1_(0.0f),
          y1_(0.0f),
          message_(nullptr) {}

    void start(Adafruit_GFX& display, runtime::SystemState&) override {
        stage_ = PaperStage::Origin;
        x0_ = 0.0f;
        y0_ = 0.0f;
        x1_ = 0.0f;
        y1_ = 0.0f;
        message_ = nullptr;
        ready_ = returnZToZero() && motion::systemGantry().setEnabled(false);
        if (!ready_) {
            message_ = motorError();
        }
        ui::view::beginPage(display, "Paper Calibration");
        render(display);
    }

    void update(Adafruit_GFX& display, runtime::SystemState& state,
                ui::Event event) override {
        if (!ready_ || stage_ == PaperStage::Done ||
            event != ui::Event::Select) {
            return;
        }

        const motor::Result<motion::XYPosition> position =
            motion::systemGantry().readXY();
        if (!position) {
            message_ = "Read error";
            render(display);
            return;
        }

        if (stage_ == PaperStage::Origin) {
            x0_ = position.value.x;
            y0_ = position.value.y;
            stage_ = PaperStage::Opposite;
            message_ = nullptr;
            render(display);
            return;
        }

        storage::Calibration updated = state.calibration;
        x1_ = position.value.x;
        y1_ = position.value.y;
        updated.x0 = x0_;
        updated.y0 = y0_;
        updated.x1 = x1_;
        updated.y1 = y1_;
        updated.paperValid = true;
        if (!storage::saveCalibration(updated)) {
            message_ = "Invalid or storage";
            render(display);
            return;
        }

        state.calibration = updated;
        stage_ = PaperStage::Done;
        message_ = "Saved";
        render(display);
    }

    void stop(runtime::SystemState&) override {
        returnAllToZero();
    }

private:
    void render(Adafruit_GFX& display) const {
        ui::view::beginBody(display);
        display.setCursor(6, 38);
        if (message_ != nullptr) {
            display.print(message_);
        } else if (stage_ == PaperStage::Origin) {
            display.print("Move to 10cm,10cm");
        } else {
            display.print("Move to opposite");
        }
        display.setCursor(6, 64);
        display.print(stage_ == PaperStage::Done ? "S4 Back" : "S3 Save XY");
        if (stage_ != PaperStage::Origin) {
            display.setCursor(6, 90);
            display.print("P0: ");
            display.print(x0_, 1);
            display.print(",");
            display.print(y0_, 1);
        }
        if (stage_ == PaperStage::Done) {
            display.setCursor(6, 112);
            display.print("P1: ");
            display.print(x1_, 1);
            display.print(",");
            display.print(y1_, 1);
        }
    }

    PaperStage stage_;
    bool ready_;
    float x0_;
    float y0_;
    float x1_;
    float y1_;
    const char* message_;
};

ZCalibrationProgram zCalibrationProgram;
PaperCalibrationProgram paperCalibrationProgram;

}

bool initializeMotors() {
    return static_cast<bool>(motion::systemGantry().begin());
}

runtime::Program& zCalibration() { return zCalibrationProgram; }

runtime::Program& paperCalibration() { return paperCalibrationProgram; }

}
