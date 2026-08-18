#include <cassert>

#include "motor/atk_motor.h"
#include "motor/zdt_motor.h"

namespace {

void checkCommonValidation(motor::Motor& motor) {
    assert(motor.begin());
    const motor::Status status =
        motor.moveAbsolute(90.0f, motor::MotionOptions(6001, 10));
    assert(!status);
    assert(status.error == motor::Error::InvalidArgument);
}

}

int main() {
    HardwareSerial zdtSerial;
    zdt::Bus zdtBus(zdtSerial, zdt::BusConfig(115200, 16, 17, 20));
    motor::ZdtMotor zdtMotor(zdtBus, zdt::MotorConfig(1, 3200));
    checkCommonValidation(zdtMotor);

    HardwareSerial atkSerial;
    atk::Bus atkBus(atkSerial, atk::BusConfig(115200, 16, 17, 20));
    motor::AtkMotor atkMotor(atkBus, atk::MotorConfig(1));
    checkCommonValidation(atkMotor);

    atkSerial.respondWith({0xC5, 0x01, 0xFA, 0x01, 0x00, 0xC1, 0x5C});
    assert(atkMotor.enable());
    assert(zdtMotor.enable());

    atkSerial.transmitted.clear();
    atkSerial.respondWith({0xC5, 0x01, 0xF2, 0x01, 0xB9, 0x5C});
    assert(atkMotor.moveAbsolute(90.0f, motor::MotionOptions(300, 22)));
    assert(atkSerial.transmitted[4] == 22);

    atkSerial.transmitted.clear();
    atkSerial.respondWith({0xC5, 0x01, 0xF1, 0x01, 0xB8, 0x5C});
    assert(atkMotor.run(-300, 22));
    assert(atkSerial.transmitted[4] == 22);
}
