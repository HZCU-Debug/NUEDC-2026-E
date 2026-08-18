#include "motion/gantry.h"

#include <Arduino.h>

#include "config/hardware.h"
#include "motor/atk_motor.h"
#include "motor/zdt_motor.h"

extern HardwareSerial Serial1;
extern HardwareSerial Serial2;

namespace motion {
namespace {

constexpr int8_t kMotorRxPins[] = {
    config::kPins.motorRx,
    config::kPins.motorYRx,
    config::kPins.motorZRx,
    config::kPins.motorRollRx,
};

constexpr int8_t kAtkRxPin =
    config::kXMotor.model == config::MotorModel::Atk
        ? config::kPins.motorRx
        : config::kYMotor.model == config::MotorModel::Atk
              ? config::kPins.motorYRx
              : config::kZMotor.model == config::MotorModel::Atk
                    ? config::kPins.motorZRx
                    : config::kPins.motorRollRx;

constexpr int8_t kAtkTxPin = config::kPins.atkMotorTx >= 0
                                 ? config::kPins.atkMotorTx
                                 : config::kPins.motorTx;

HardwareSerial& atkSerial() {
#if defined(BOARD_ESP32_S3_CBH)
    return Serial1;
#else
    return Serial2;
#endif
}

template <Axis axis, config::MotorModel model>
struct ConfiguredMotor;

template <Axis axis>
struct ConfiguredMotor<axis, config::MotorModel::Zdt> {
    static motor::Motor& get(zdt::Bus& zdtBus, atk::Bus&,
                             const config::AxisMotorConfig& motorConfig) {
        static motor::ZdtMotor motor(
            zdtBus,
            zdt::MotorConfig(motorConfig.address,
                             motorConfig.pulsesPerRevolution,
                             motorConfig.invertDirection));
        return motor;
    }
};

template <Axis axis>
struct ConfiguredMotor<axis, config::MotorModel::Atk> {
    static motor::Motor& get(zdt::Bus&, atk::Bus& atkBus,
                             const config::AxisMotorConfig& motorConfig) {
        static motor::AtkMotor motor(
            atkBus,
            atk::MotorConfig(motorConfig.address,
                             motorConfig.invertDirection,
                             kMotorRxPins[static_cast<uint8_t>(axis)]));
        return motor;
    }
};

}

Gantry& systemGantry() {
    static zdt::Bus zdtBus(
        Serial2, zdt::BusConfig(config::kMotorBaudRate,
                                config::kPins.motorRx,
                                config::kPins.motorTx));
    static atk::Bus atkBus(
        atkSerial(),
        atk::BusConfig(config::kMotorBaudRate, kAtkRxPin, kAtkTxPin));
    motor::Motor& x =
        ConfiguredMotor<Axis::X, config::kXMotor.model>::get(
            zdtBus, atkBus, config::kXMotor);
    motor::Motor& y =
        ConfiguredMotor<Axis::Y, config::kYMotor.model>::get(
            zdtBus, atkBus, config::kYMotor);
    motor::Motor& z =
        ConfiguredMotor<Axis::Z, config::kZMotor.model>::get(
            zdtBus, atkBus, config::kZMotor);
    motor::Motor& rotation =
        ConfiguredMotor<Axis::Rotation, config::kRollMotor.model>::get(
            zdtBus, atkBus, config::kRollMotor);
    static Gantry gantry(x, y, z, rotation);
    return gantry;
}

}
