#include <Preferences.h>
#include <cassert>

#include "storage/calibration.h"

int main() {
    Preferences::reset();

    storage::Calibration calibration;
    assert(!storage::loadCalibration(calibration));

    calibration.zDown = -42.5f;
    calibration.zValid = true;
    assert(storage::saveCalibration(calibration));

    storage::Calibration loaded;
    assert(storage::loadCalibration(loaded));
    assert(loaded.zValid);
    assert(!loaded.paperValid);
    assert(loaded.zDown == -42.5f);

    calibration.paperValid = true;
    assert(!storage::saveCalibration(calibration));

    calibration.x0 = 10.0f;
    calibration.y0 = 20.0f;
    calibration.x1 = 110.0f;
    calibration.y1 = 220.0f;
    assert(storage::saveCalibration(calibration));
    assert(storage::loadCalibration(loaded));
    assert(loaded.paperValid);
    assert(loaded.x0 == 10.0f);
    assert(loaded.y0 == 20.0f);
    assert(loaded.x1 == 110.0f);
    assert(loaded.y1 == 220.0f);

    return 0;
}
