#pragma once

namespace control {

/**
 * @brief 使用最近三次采样抑制偶发尖峰的中值滤波器
 */
class MedianFilter3 {
public:
    /**
     * @brief 创建中值滤波器
     */
    MedianFilter3()
        : samples_{0.0f, 0.0f, 0.0f}, initialized_(false) {}

    /**
     * @brief 更新中值滤波输出
     * @param sample 当前采样值
     * @return 最近三次采样的中值
     */
    float update(float sample) {
        if (!initialized_) {
            reset(sample);
            return sample;
        }

        samples_[0] = samples_[1];
        samples_[1] = samples_[2];
        samples_[2] = sample;
        return median();
    }

    /**
     * @brief 使用指定值重置采样历史
     * @param value 重置后的采样值
     */
    void reset(float value) {
        samples_[0] = value;
        samples_[1] = value;
        samples_[2] = value;
        initialized_ = true;
    }

private:
    float median() const {
        float low = samples_[0];
        float middle = samples_[1];
        const float high = samples_[2];

        if (low > middle) {
            const float temporary = low;
            low = middle;
            middle = temporary;
        }
        if (middle > high) {
            middle = high;
        }
        if (low > middle) {
            middle = low;
        }
        return middle;
    }

    float samples_[3];
    bool initialized_;
};

/**
 * @brief 按时间常数平滑连续采样的一阶低通滤波器
 */
class LowPassFilter {
public:
    /**
     * @brief 创建一阶低通滤波器
     * @param timeConstantSeconds 平滑时间常数秒数，零表示直接透传
     */
    explicit LowPassFilter(float timeConstantSeconds)
        : timeConstantSeconds_(timeConstantSeconds < 0.0f
                                   ? 0.0f
                                   : timeConstantSeconds),
          value_(0.0f), initialized_(false) {}

    /**
     * @brief 更新低通滤波输出
     * @param sample 当前采样值
     * @param dtSeconds 距上次更新的秒数
     * @return 平滑后的当前输出
     */
    float update(float sample, float dtSeconds) {
        if (!initialized_ || timeConstantSeconds_ == 0.0f) {
            value_ = sample;
            initialized_ = true;
            return value_;
        }
        if (dtSeconds <= 0.0f) {
            return value_;
        }

        const float alpha =
            dtSeconds / (timeConstantSeconds_ + dtSeconds);
        value_ += alpha * (sample - value_);
        return value_;
    }

    /**
     * @brief 使用指定值重置滤波输出
     * @param value 重置后的输出值
     */
    void reset(float value) {
        value_ = value;
        initialized_ = true;
    }

private:
    float timeConstantSeconds_;
    float value_;
    bool initialized_;
};

/**
 * @brief 按每秒最大变化量限制输出变化的速率限幅器
 */
class RateLimiter {
public:
    /**
     * @brief 创建速率限幅器
     * @param maxDeltaPerSecond 每秒允许的最大变化量
     */
    explicit RateLimiter(float maxDeltaPerSecond)
        : maxDeltaPerSecond_(maxDeltaPerSecond < 0.0f
                                 ? 0.0f
                                 : maxDeltaPerSecond),
          value_(0.0f), initialized_(false) {}

    /**
     * @brief 更新限幅输出
     * @param target 当前目标值
     * @param dtSeconds 距上次更新的秒数
     * @return 限幅后的当前输出
     */
    float update(float target, float dtSeconds) {
        if (!initialized_) {
            value_ = target;
            initialized_ = true;
            return value_;
        }
        if (dtSeconds <= 0.0f) {
            return value_;
        }

        const float maxDelta = maxDeltaPerSecond_ * dtSeconds;
        float delta = target - value_;
        if (delta > maxDelta) {
            delta = maxDelta;
        } else if (delta < -maxDelta) {
            delta = -maxDelta;
        }
        value_ += delta;
        return value_;
    }

    /**
     * @brief 使用指定值重置限幅输出
     * @param value 重置后的输出值
     */
    void reset(float value) {
        value_ = value;
        initialized_ = true;
    }

private:
    float maxDeltaPerSecond_;
    float value_;
    bool initialized_;
};

}
