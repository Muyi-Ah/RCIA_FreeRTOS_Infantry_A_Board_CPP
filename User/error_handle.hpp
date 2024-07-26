#pragma once
#include "config.hpp"

class ErrorHandle {
   public:
    bool motor_is_offline[kMotorCount];  //电机掉线标志
    bool IMU_is_offline;                 //IMU掉线标志
    bool dr16_is_offline;                //遥控器接收机掉线标志
    bool communication_is_offline;       //板间通信掉线标志

    void Check();
    void Handle();
};