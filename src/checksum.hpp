#pragma once
#include <cstdint>

namespace robot_sim {
class Checksum {
public:
    void byte(uint8_t value) { value_ = (value_ ^ value) * 1099511628211ull; }
    void word(uint64_t value) {
        for (unsigned i = 0; i < 8; ++i) {
            byte(static_cast<uint8_t>(value));
            value >>= 8;
        }
    }
    uint64_t value() const { return value_; }
private:
    uint64_t value_ = 14695981039346656037ull;
};
}

