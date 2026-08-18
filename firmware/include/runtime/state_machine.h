#pragma once

namespace runtime {

/**
 * @brief 保存当前状态并提供统一跳转入口
 * @tparam State 状态枚举类型
 */
template <typename State>
class StateMachine {
public:
    /**
     * @brief 创建状态机
     * @param initial 初始状态
     */
    explicit StateMachine(State initial) : current_(initial) {}

    /**
     * @brief 获取当前状态
     * @return 当前状态
     */
    State state() const { return current_; }

    /**
     * @brief 重置入口状态
     * @param entry 新入口状态
     */
    void reset(State entry) { current_ = entry; }

    /**
     * @brief 跳转到下一状态
     * @param next 下一状态
     */
    void transitionTo(State next) { current_ = next; }

private:
    State current_;
};

}
