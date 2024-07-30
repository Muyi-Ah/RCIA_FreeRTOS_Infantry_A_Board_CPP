#include "LPF.hpp"

LPF::LPF(float alpha) {
    alpha_ = alpha;
}

float LPF::Compute(float input) {
    float output = alpha_ * input + (1 - alpha_) * output_prev_;
    output_prev_ = output;
    return output;
}

void LPF::ClearOutputPrev() {
    output_prev_ = 0;
}