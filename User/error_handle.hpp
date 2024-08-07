#pragma once
#include "config.hpp"

class ErrorHandle {
   public:
    bool motor_is_offline[kMotorCount];  //������߱�־
    bool IMU_is_offline;                 //IMU���߱�־
    bool dr16_is_offline;                //ң�������ջ����߱�־
    bool communication_is_offline;       //���ͨ�ŵ��߱�־

    void Check();
    void Handle();
};