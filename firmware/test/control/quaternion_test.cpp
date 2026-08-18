#include <cassert>
#include <cmath>

#include "control/quaternion.h"

static bool near(float actual, float expected) {
    return std::fabs(actual - expected) < 0.0001f;
}

int main() {
    const float pi = 3.14159265358979323846f;
    control::QuaternionIntegrator integrator;

    integrator.update(0.0f, 0.0f, pi, 0.5f);
    const control::Quaternion halfTurn = integrator.value();
    assert(near(halfTurn.scalar, 0.70710678f));
    assert(near(halfTurn.x, 0.0f));
    assert(near(halfTurn.y, 0.0f));
    assert(near(halfTurn.z, 0.70710678f));

    integrator.update(0.0f, 0.0f, pi, 0.5f);
    const control::Quaternion fullTurn = integrator.value();
    assert(near(fullTurn.scalar, 0.0f));
    assert(near(fullTurn.z, 1.0f));

    integrator.reset();
    integrator.update(pi, 0.0f, 0.0f, 0.0f);
    assert(near(integrator.value().scalar, 1.0f));
    assert(near(integrator.value().x, 0.0f));

    return 0;
}
