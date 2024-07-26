#include "simple_math.hpp"

float deg2rad(float degrees) {
    return degrees * (kPI / 180.0f);
}

float rad2deg(float radians) {
    return radians * (180.0f / kPI);
}
