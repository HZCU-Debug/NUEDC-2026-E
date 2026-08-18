#include <cassert>

#include <Arduino.h>

#include "RecordingMotor.h"
#include "motion/gantry.h"

HardwareSerial Serial2;

int main() {
    RecordingMotor x;
    RecordingMotor y;
    RecordingMotor z;
    RecordingMotor rotation;
    motion::Gantry gantry(x, y, z, rotation);

    assert(gantry.begin());
    assert(x.beginCount == 1 && y.beginCount == 1 && z.beginCount == 1 &&
           rotation.beginCount == 1);
    assert(x.clearCount == 1 && y.clearCount == 1 && z.clearCount == 1 &&
           rotation.clearCount == 1);
    assert(gantry.setEnabled());
    assert(x.enabled && y.enabled && z.enabled && rotation.enabled);

    assert(gantry.moveXY(90.0f, -45.0f, motor::MotionOptions(300, 10)));
    assert(x.lastDegrees == 90.0f && y.lastDegrees == -45.0f);
    assert(x.lastOptions.rpm == 300 && y.lastOptions.acceleration == 10);

    x.position = 80.0f;
    y.position = -45.0f;
    assert(gantry.pollXY() == motion::Arrival::Pending);
    x.position = 90.0f;
    assert(gantry.pollXY() == motion::Arrival::Reached);

    x.state.reached = true;
    y.state.reached = true;
    assert(gantry.pollXY() == motion::Arrival::Reached);
    assert(gantry.pollDiagnostic().axis == 'Y');
    assert(gantry.pollDiagnostic().error == motor::Error::None);
    assert(gantry.pollDiagnostic().flags == 0x02);

    assert(gantry.moveZ(90.0f, motor::MotionOptions(300, 10)));
    z.state.reached = true;
    assert(gantry.pollZ() == motion::Arrival::Reached);
    z.state.faulted = true;
    assert(gantry.pollZ() == motion::Arrival::Fault);

    assert(gantry.moveXYRelativeRotation(
        30.0f, 60.0f, -90.0f, motor::MotionOptions(300, 10),
        motor::MotionOptions(200, 5)));
    assert(rotation.relativeCount == 1 && rotation.lastDegrees == -90.0f);
    x.state.reached = true;
    y.state.reached = true;
    rotation.state.reached = true;
    assert(gantry.pollXYRotation() == motion::Arrival::Reached);

    assert(gantry.stop());
    assert(x.stopCount == 1 && y.stopCount == 1 && z.stopCount == 1 &&
           rotation.stopCount == 1);
}
