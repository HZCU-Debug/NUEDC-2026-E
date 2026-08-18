#pragma once

namespace control {

/**
 * @brief PID 控制器增益
 */
struct PidGains {
    /**
     * @brief 创建 PID 控制器增益
     * @param kp 比例增益
     * @param ki 积分增益
     * @param kd 微分增益
     */
    PidGains(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

    /** 比例增益 */
    float kp;
    /** 积分增益 */
    float ki;
    /** 微分增益 */
    float kd;
};

/**
 * @brief 按误差计算绝对输出的位置式 PID 控制器
 */
class PositionalPid {
public:
    /**
     * @brief 创建位置式 PID 控制器
     * @param gains PID 增益
     */
    explicit PositionalPid(const PidGains& gains)
        : gains_(gains), integral_(0.0f), previousError_(0.0f) {}

    /**
     * @brief 更新 PID 输出
     * @param error 当前误差
     * @param dtSeconds 距上次更新的秒数
     * @return 当前绝对输出
     */
    float update(float error, float dtSeconds) {
        integral_ += error * dtSeconds;
        const float derivative = (error - previousError_) / dtSeconds;
        previousError_ = error;
        return gains_.kp * error + gains_.ki * integral_ +
               gains_.kd * derivative;
    }

    /**
     * @brief 更新 PID 增益
     * @param gains PID 增益
     */
    void setGains(const PidGains& gains) { gains_ = gains; }

    /**
     * @brief 清除累计误差和历史误差
     */
    void reset() {
        integral_ = 0.0f;
        previousError_ = 0.0f;
    }

private:
    PidGains gains_;
    float integral_;
    float previousError_;
};

/**
 * @brief 按误差计算输出增量的增量式 PID 控制器
 */
class IncrementalPid {
public:
    /**
     * @brief 创建增量式 PID 控制器
     * @param gains PID 增益
     */
    explicit IncrementalPid(const PidGains& gains)
        : gains_(gains), output_(0.0f), previousError_(0.0f),
          previousDerivative_(0.0f) {}

    /**
     * @brief 更新 PID 输出
     * @param error 当前误差
     * @param dtSeconds 距上次更新的秒数
     * @return 累计后的当前输出
     */
    float update(float error, float dtSeconds) {
        const float derivative = (error - previousError_) / dtSeconds;
        output_ += gains_.kp * (error - previousError_) +
                   gains_.ki * error * dtSeconds +
                   gains_.kd * (derivative - previousDerivative_);
        previousError_ = error;
        previousDerivative_ = derivative;
        return output_;
    }

    /**
     * @brief 更新 PID 增益
     * @param gains PID 增益
     */
    void setGains(const PidGains& gains) { gains_ = gains; }

    /**
     * @brief 清除历史误差并设置累计输出
     * @param output 重置后的累计输出
     */
    void reset(float output = 0.0f) {
        output_ = output;
        previousError_ = 0.0f;
        previousDerivative_ = 0.0f;
    }

private:
    PidGains gains_;
    float output_;
    float previousError_;
    float previousDerivative_;
};

}
