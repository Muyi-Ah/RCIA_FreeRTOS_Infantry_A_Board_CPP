#include "rotate_feedforward.hpp"

float RotateFeedForward::Compute() const {
    return vw * C_;
}