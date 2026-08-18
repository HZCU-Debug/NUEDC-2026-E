#include <cassert>
#include <cmath>

#include "control/pid.h"

static bool near(float actual, float expected) {
    return std::fabs(actual - expected) < 0.0001f;
}

int main() {
    const control::PidGains gains(2.0f, 0.5f, 0.25f);
    control::PositionalPid positional(gains);
    control::IncrementalPid incremental(gains);

    assert(near(positional.update(2.0f, 0.5f), 5.5f));
    assert(near(incremental.update(2.0f, 0.5f), 5.5f));
    assert(near(positional.update(1.0f, 0.25f), 1.625f));
    assert(near(incremental.update(1.0f, 0.25f), 1.625f));

    positional.setGains(control::PidGains(2.0f, 0.0f, 0.0f));
    incremental.setGains(control::PidGains(2.0f, 0.0f, 0.0f));
    positional.reset();
    incremental.reset(3.0f);
    assert(near(positional.update(1.5f, 1.0f), 3.0f));
    assert(near(incremental.update(1.5f, 1.0f), 6.0f));

    return 0;
}
