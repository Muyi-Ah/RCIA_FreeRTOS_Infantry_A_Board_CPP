#include "ch110.hpp"
#include <math.h>

float CH110::IntegralErrorCompute(float value, float* value_prev) {
    float res1 = 0, res2 = 0;
    if (value - *value_prev > 0) {
        res1 = value - *value_prev - 360.0f;
        res2 = value - *value_prev;
        *value_prev = value;  //更新上一次值
        if (fabsf(res1) < fabsf(res2)) {
            return res1;
        } else {
            return res2;
        }
    } else {
        res1 = value - *value_prev + 360.0f;
        res2 = value - *value_prev;
        *value_prev = value;  //更新上一次值
        if (fabsf(res1) < fabsf(res2)) {
            return res1;
        } else {
            return res2;
        }
    }
}

void CH110::EulerUpdate(const uint8_t* buf) {
    roll_ = ((int16_t)(buf[1] << 8 | buf[0]) / 100.0f);
    pitch_ = ((int16_t)(buf[3] << 8 | buf[2]) / 100.0f);
    yaw_ = ((int16_t)(buf[5] << 8 | buf[4]) / 100.0f);

    if (is_first_recv_done_ == false) {
        roll_prev_ = roll_;
        pitch_prev_ = pitch_;
        yaw_prev_ = yaw_;

        is_first_recv_done_ = true;
    } else {
        roll_integral_ += IntegralErrorCompute(roll_, &roll_prev_);
        pitch_integral_ += IntegralErrorCompute(pitch_, &pitch_prev_);
        yaw_integral_ += IntegralErrorCompute(yaw_, &yaw_prev_);
        //@warning 务必在角度值更新之后再使用integral_error_update更新角度积分值
    }
}

void CH110::VelocityUpdate(const uint8_t* buf) {
    x_velocity_ = (int16_t)(buf[1] << 8 | buf[0]);
    y_velocity_ = (int16_t)(buf[3] << 8 | buf[2]);
    z_velocity_ = (int16_t)(buf[5] << 8 | buf[4]);
}
