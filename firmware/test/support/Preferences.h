#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

class Preferences {
public:
    bool begin(const char*, bool readOnly = false) {
        readOnly_ = readOnly;
        return true;
    }

    size_t getBytes(const char*, void* destination, size_t size) const {
        if (data().size() != size) {
            return 0;
        }
        std::memcpy(destination, data().data(), size);
        return size;
    }

    size_t putBytes(const char*, const void* source, size_t size) {
        if (readOnly_) {
            return 0;
        }
        const uint8_t* bytes = static_cast<const uint8_t*>(source);
        data().assign(bytes, bytes + size);
        return size;
    }

    void end() {}

    static void reset() { data().clear(); }

private:
    static std::vector<uint8_t>& data() {
        static std::vector<uint8_t> value;
        return value;
    }

    bool readOnly_ = false;
};
