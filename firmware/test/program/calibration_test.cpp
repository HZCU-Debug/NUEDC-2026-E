#include <algorithm>
#include <cassert>
#include <string>

#include "RecordingMotor.h"
#include "motion/gantry.h"
#include "program/programs.h"

namespace {

bool displayed(const Adafruit_GFX& display, const char* text) {
    return std::find(display.printed.begin(), display.printed.end(), text) !=
           display.printed.end();
}

RecordingMotor x;
RecordingMotor y;
RecordingMotor z;
RecordingMotor rotation;
motion::Gantry gantry(x, y, z, rotation);

}

namespace motion {

Gantry& systemGantry() { return gantry; }

}

namespace storage {

bool saveCalibration(const Calibration&) { return true; }

}

int main() {
    assert(gantry.begin());

    runtime::SystemState state;
    Adafruit_GFX display(240, 135);
    runtime::Program& calibration = program::paperCalibration();
    calibration.start(display, state);

    x.position = 10.0f;
    y.position = 10.0f;
    calibration.update(display, state, ui::Event::Select);
    x.position = 210.0f;
    y.position = 297.0f;
    calibration.update(display, state, ui::Event::Select);

    const int xMovesBeforeExit = x.absoluteCount;
    const int yMovesBeforeExit = y.absoluteCount;
    const int zMovesBeforeExit = z.absoluteCount;
    const int rotationMovesBeforeExit = rotation.absoluteCount;
    calibration.stop(state);
    assert(x.absoluteCount == xMovesBeforeExit + 1);
    assert(y.absoluteCount == yMovesBeforeExit + 1);
    assert(z.absoluteCount == zMovesBeforeExit + 1);
    assert(rotation.absoluteCount == rotationMovesBeforeExit + 1);
    assert(x.lastDegrees == 0.0f && y.lastDegrees == 0.0f &&
           z.lastDegrees == 0.0f && rotation.lastDegrees == 0.0f);

    runtime::Program& zCalibration = program::zCalibration();
    zCalibration.start(display, state);
    z.position = -90.0f;
    zCalibration.update(display, state, ui::Event::Select);

    const int xMovesBeforeZExit = x.absoluteCount;
    const int yMovesBeforeZExit = y.absoluteCount;
    const int zMovesBeforeZExit = z.absoluteCount;
    const int rotationMovesBeforeZExit = rotation.absoluteCount;
    zCalibration.stop(state);
    assert(x.absoluteCount == xMovesBeforeZExit + 1);
    assert(y.absoluteCount == yMovesBeforeZExit + 1);
    assert(z.absoluteCount == zMovesBeforeZExit + 1);
    assert(rotation.absoluteCount == rotationMovesBeforeZExit + 1);

    rotation.nextStatus = motor::Status(motor::Error::Timeout);
    zCalibration.start(display, state);
    assert(displayed(display, "Motor R error"));
}
