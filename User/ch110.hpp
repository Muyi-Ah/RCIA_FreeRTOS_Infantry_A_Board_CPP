#pragma once
#include <cstdint>

class CH110 {
   public:

    float roll_ = 0;
    float pitch_ = 0;
    float yaw_ = 0;
    float roll_prev_ = 0;
    float pitch_prev_ = 0;
    float yaw_prev_ = 0;
    float roll_integral_ = 0;   //@notice 此处作为积分值若精度不够，考虑换成double
    float pitch_integral_ = 0;  //@notice 此处作为积分值若精度不够，考虑换成double
    float yaw_integral_ = 0;    //@notice 此处作为积分值若精度不够，考虑换成double
    int16_t x_velocity_ = 0;
    int16_t y_velocity_ = 0;
    int16_t z_velocity_ = 0;
    bool is_reply_ = false;  //IMU应答标志位
    bool is_first_recv_done_ = false;  //第一次接收数据标志位

    void EulerUpdate(const uint8_t* buf);
    void VelocityUpdate(const uint8_t* buf);

   private:
    float IntegralErrorCompute(float value, float* value_prev);
};