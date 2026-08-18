#include "imu/gyroscope.h"

#include <Arduino.h>
#include <LSM6DSRSensor.h>
#include <Wire.h>

#include "config/hardware.h"

namespace gyroscope {
namespace {

const float kOutputRateHz = 208.0f;
const int32_t kFullScaleDps = 2000;
const float kDegreesToRadians = 0.01745329251994329577f;
const float kGyroBiasXDps = 0.0f;
const float kGyroBiasYDps = 0.0f;
const float kGyroBiasZDps = 0.0f;

LSM6DSRSensor imu(&Wire, LSM6DSR_I2C_ADD_H);
uint32_t lastSampleAt = 0;

}

bool begin() {
    Wire.begin(config::kPins.imuSda, config::kPins.imuScl);
    Wire.setClock(400000);
    uint8_t identity = 0;
    const bool ready =
        imu.begin() == LSM6DSR_OK &&
        imu.ReadID(&identity) == LSM6DSR_OK && identity == LSM6DSR_ID &&
        imu.Set_G_ODR(kOutputRateHz) == LSM6DSR_OK &&
        imu.Set_G_FS(kFullScaleDps) == LSM6DSR_OK &&
        imu.Enable_G() == LSM6DSR_OK;
    lastSampleAt = micros();
    return ready;
}

bool read(Sample& sample) {
    uint8_t dataReady = 0;
    int32_t gyroMdps[3];
    if (imu.Get_G_DRDY_Status(&dataReady) != LSM6DSR_OK || dataReady == 0 ||
        imu.Get_G_Axes(gyroMdps) != LSM6DSR_OK) {
        return false;
    }

    const uint32_t sampledAt = micros();
    const uint32_t elapsedMicros = sampledAt - lastSampleAt;
    lastSampleAt = sampledAt;
    sample.gyroX = (gyroMdps[0] / 1000.0f - kGyroBiasXDps) *
                   kDegreesToRadians;
    sample.gyroY = (gyroMdps[1] / 1000.0f - kGyroBiasYDps) *
                   kDegreesToRadians;
    sample.gyroZ = (gyroMdps[2] / 1000.0f - kGyroBiasZDps) *
                   kDegreesToRadians;
    sample.dtSeconds = elapsedMicros / 1000000.0f;
    return elapsedMicros != 0;
}

void end() { imu.Disable_G(); }

}
