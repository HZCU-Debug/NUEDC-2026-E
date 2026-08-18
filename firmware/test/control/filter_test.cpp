#include <cassert>
#include <cmath>

#include "control/filter.h"

static bool near(float actual, float expected) {
    return std::fabs(actual - expected) < 0.0001f;
}

int main() {
    control::MedianFilter3 firstMedian;
    assert(near(firstMedian.update(7.0f), 7.0f));

    control::MedianFilter3 median;
    median.reset(10.0f);
    assert(near(median.update(10.0f), 10.0f));
    assert(near(median.update(100.0f), 10.0f));
    assert(near(median.update(10.0f), 10.0f));
    assert(near(median.update(20.0f), 20.0f));

    control::LowPassFilter lowPass(0.5f);
    assert(near(lowPass.update(7.0f, 0.5f), 7.0f));
    lowPass.reset(0.0f);
    assert(near(lowPass.update(2.0f, 0.5f), 1.0f));
    assert(near(lowPass.update(2.0f, 0.25f), 1.3333f));
    assert(near(lowPass.update(10.0f, 0.0f), 1.3333f));
    lowPass.reset(4.0f);
    assert(near(lowPass.update(6.0f, 0.5f), 5.0f));

    control::LowPassFilter passThrough(0.0f);
    passThrough.reset(1.0f);
    assert(near(passThrough.update(3.0f, 0.5f), 3.0f));

    control::RateLimiter rateLimiter(4.0f);
    assert(near(rateLimiter.update(7.0f, 0.5f), 7.0f));
    rateLimiter.reset(0.0f);
    assert(near(rateLimiter.update(3.0f, 0.5f), 2.0f));
    assert(near(rateLimiter.update(-4.0f, 0.5f), 0.0f));
    assert(near(rateLimiter.update(-0.5f, 0.25f), -0.5f));
    assert(near(rateLimiter.update(100.0f, 0.0f), -0.5f));
    rateLimiter.reset(8.0f);
    assert(near(rateLimiter.update(9.0f, 0.5f), 9.0f));

    return 0;
}
