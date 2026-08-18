#include "storage/calibration.h"

#include <Preferences.h>
#include <math.h>
#include <stdint.h>

namespace storage {
namespace {

const char* const kNamespace = "calibration";
const char* const kKey = "data";
const uint8_t kVersion = 2;
const uint8_t kZValid = 1 << 0;
const uint8_t kPaperValid = 1 << 1;

struct StoredCalibration {
    uint8_t version;
    uint8_t validFlags;
    float zDown;
    float x0;
    float y0;
    float x1;
    float y1;
};

bool valid(const StoredCalibration& stored) {
    if (stored.version != kVersion) {
        return false;
    }
    if ((stored.validFlags & kZValid) != 0 && !isfinite(stored.zDown)) {
        return false;
    }
    return (stored.validFlags & kPaperValid) == 0 ||
           (isfinite(stored.x0) && isfinite(stored.y0) &&
            isfinite(stored.x1) && isfinite(stored.y1) &&
            stored.x0 != stored.x1 && stored.y0 != stored.y1);
}

}

bool loadCalibration(Calibration& calibration) {
    Preferences preferences;
    if (!preferences.begin(kNamespace, true)) {
        return false;
    }

    StoredCalibration stored = {};
    const size_t size = preferences.getBytes(kKey, &stored, sizeof(stored));
    preferences.end();
    if (size != sizeof(stored) || !valid(stored)) {
        return false;
    }

    calibration.zDown = stored.zDown;
    calibration.x0 = stored.x0;
    calibration.y0 = stored.y0;
    calibration.x1 = stored.x1;
    calibration.y1 = stored.y1;
    calibration.zValid = (stored.validFlags & kZValid) != 0;
    calibration.paperValid = (stored.validFlags & kPaperValid) != 0;
    return true;
}

bool saveCalibration(const Calibration& calibration) {
    StoredCalibration stored = {};
    stored.version = kVersion;
    stored.validFlags = (calibration.zValid ? kZValid : 0) |
                        (calibration.paperValid ? kPaperValid : 0);
    stored.zDown = calibration.zDown;
    stored.x0 = calibration.x0;
    stored.y0 = calibration.y0;
    stored.x1 = calibration.x1;
    stored.y1 = calibration.y1;
    if (!valid(stored)) {
        return false;
    }

    Preferences preferences;
    if (!preferences.begin(kNamespace, false)) {
        return false;
    }
    const bool saved = preferences.putBytes(kKey, &stored, sizeof(stored)) ==
                       sizeof(stored);
    preferences.end();
    return saved;
}

}
