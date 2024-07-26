#include "clamp.hpp"
#include <cstdint>

float clamp(float value, float min, float max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    }
    return value;
}

int32_t clamp(int32_t value, int32_t min, int32_t max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    }
    return value;
}
