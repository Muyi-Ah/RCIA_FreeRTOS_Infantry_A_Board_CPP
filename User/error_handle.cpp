#include "error_handle.hpp"
#include <cstdint>
#include "variables.hpp"

/**
 * @brief ������
 * 
 */
void ErrorHandle::Check() {
    //������
    for (uint8_t i = 0; i < kMotorCount; i++) {
        if (dji_motor_list[i]->is_reply_) {
            dji_motor_list[i]->is_reply_ = false;
            motor_is_offline[i] = false;
        } else {
            motor_is_offline[i] = true;
        }
    }

    //IMUͨ�ż��
    if (ch110.is_reply_) {
        ch110.is_reply_ = false;
        IMU_is_offline = false;
    } else {
        IMU_is_offline = true;
    }

    //ң�������ջ�ͨ�ż��
    if (dr16.is_reply_) {
        dr16.is_reply_ = false;
        dr16_is_offline = false;
    } else {
        dr16_is_offline = true;
    }

    //���ͨ�ż��
    if (comm.is_reply_) {
        comm.is_reply_ = false;
        communication_is_offline = false;
    } else {
        communication_is_offline = true;
    }
}

/**
 * @brief ������
 * 
 */
void ErrorHandle::Handle() {
    for (uint8_t i = 0; i < kMotorCount; i++) {

        //�����������
        if (motor_is_offline[i]) {
            //@developing...
        }
    }

    //IMU��������
    if (IMU_is_offline) {
        //@developing...
    }

    //ң�������ջ���������
    if (dr16_is_offline) {
        dr16.remote_.ch0_ = 1024;
        dr16.remote_.ch1_ = 1024;
        dr16.remote_.ch2_ = 1024;
        dr16.remote_.ch3_ = 1024;
        dr16.remote_.s1_ = 0;
        dr16.remote_.s2_ = 0;
        dr16.remote_.wheel_ = 1024;
    }

    //���ͨ�Ŷ�������
    if (communication_is_offline) {
        RFF.vw = 0;
    }
}
