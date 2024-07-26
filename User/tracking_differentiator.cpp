#include "tracking_differentiator.hpp"

float TD::Compute(float u) {
    u_ = u;
    x1_ += x2_ * h_;
    x2_ += (-2.0f * r_ * x2_ - r_ * r_ * (x1_ - u_)) * h_;
    return x1_;
}
