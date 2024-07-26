#pragma once

class PID {
   public:
    PID(float kp, float ki, float kd, float integral_limit, float output_limit, float k_feed_forward)
        : kp_(kp),
          ki_(ki),
          kd_(kd),
          integral_limit_(integral_limit),
          output_limit_(output_limit),
          k_feed_forward_(k_feed_forward){};

    float kp_ = 0;
    float ki_ = 0;
    float kd_ = 0;
    float error_ = 0;
    float prev_error_ = 0;
    float integral_ = 0;
    float derivative_ = 0;
    float output_ = 0;
    float integral_limit_ = 0;
    float output_limit_ = 0;
    float k_feed_forward_ = 0;
    float feed_forward_ = 0;
    float prev_target_ = 0;

    float Compute(float target, float measure);
    float Compute(float error);

   private:
    float clamp(float value, float min, float max);
};
