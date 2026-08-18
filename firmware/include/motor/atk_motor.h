#pragma once

#include "atk/motor.h"
#include "motor/motor.h"

namespace motor {

/**
 * @brief 将正点原子 PDxxS1 SDK 适配为公共电机接口
 */
class AtkMotor : public Motor {
public:
    /**
     * @brief 创建正点原子电机适配器
     * @param bus 电机所在的串口总线
     * @param config 电机地址和方向配置
     */
    AtkMotor(atk::Bus& bus, const atk::MotorConfig& config);

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
    atk::Bus& bus_;
    atk::Motor motor_;
};

}
