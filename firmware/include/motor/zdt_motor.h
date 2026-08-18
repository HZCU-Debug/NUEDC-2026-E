#pragma once

#include "motor/motor.h"
#include "zdt/motor.h"

namespace motor {

/**
 * @brief 将张大头电机 SDK 适配为公共电机接口
 */
class ZdtMotor : public Motor {
public:
    /**
     * @brief 创建张大头电机适配器
     * @param bus 电机所在的串口总线
     * @param config 电机地址和方向配置
     */
    ZdtMotor(zdt::Bus& bus, const zdt::MotorConfig& config);

    /** @copydoc Motor::begin */
    Status begin() override;
    /** @copydoc Motor::enable */
    Status enable(bool enabled = true) override;
    /** @copydoc Motor::clearPosition */
    Status clearPosition() override;
    /** @copydoc Motor::run */
    Status run(int16_t signedRpm, uint8_t acceleration = 0) override;
    /** @copydoc Motor::moveRelative */
    Status moveRelative(float degrees, const MotionOptions& options) override;
    /** @copydoc Motor::moveAbsolute */
    Status moveAbsolute(float degrees, const MotionOptions& options) override;
    /** @copydoc Motor::stop */
    Status stop() override;
    /** @copydoc Motor::readState */
    Result<State> readState() override;
    /** @copydoc Motor::readPositionDegrees */
    Result<float> readPositionDegrees() override;

private:
    zdt::Bus& bus_;
    zdt::Motor motor_;
};

}
