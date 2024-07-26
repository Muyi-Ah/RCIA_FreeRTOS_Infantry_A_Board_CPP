#include "pid.hpp"

float PID::clamp(float value, float min, float max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    }
    return value;
}

float PID::Compute(float target, float measure) {
    error_ = target - measure;
    integral_ += error_;
    integral_ = clamp(integral_, -integral_limit_, integral_limit_);
    derivative_ = error_ - prev_error_;
    prev_error_ = error_;
    feed_forward_ = (target - prev_target_) * k_feed_forward_;
    prev_target_ = target;
    output_ = kp_ * error_ + ki_ * integral_ + kd_ * derivative_ + feed_forward_;
    output_ = clamp(output_, -output_limit_, output_limit_);
    return output_;
}

float PID::Compute(float error) {
    error_ = error;
    integral_ += error_;
    integral_ = clamp(integral_, -integral_limit_, integral_limit_);
    derivative_ = error_ - prev_error_;
    prev_error_ = error_;
    output_ = kp_ * error + ki_ * integral_ + kd_ * derivative_;
    output_ = clamp(output_, -output_limit_, output_limit_);
    return output_;
}
