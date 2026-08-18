#pragma once

#include <stdint.h>

#include "storage/calibration.h"

namespace runtime {

/**
 * @brief 跨 Program 共享的系统状态
 */
struct SystemState {
    SystemState() : now(0), calibration() {}

    /** 当前主循环时间 */
    uint32_t now;
    /** 当前加载的设备校准结果 */
    storage::Calibration calibration;
};

}
