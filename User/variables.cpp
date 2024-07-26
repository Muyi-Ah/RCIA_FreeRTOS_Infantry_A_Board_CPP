#include "variables.hpp"

DjiMotor motor_201(0x201);  //Ħ����
DjiMotor motor_202(0x202);  //Ħ����
DjiMotor motor_204(0x204);  //����
DjiMotor motor_205(0x205);  //pitch
DjiMotor motor_206(0x206);  //yaw
DjiMotor* dji_motor_list[kMotorCount] = {
    &motor_201, &motor_202, &motor_204, &motor_205, &motor_206,
};  //�������

//΢�ָ����� ���ڵ���ٶ��˲�
TD td_201(200, 0.001);
TD td_202(200, 0.001);
TD td_204(200, 0.001);
TD td_205(200, 0.001);
TD td_206(200, 0.001);

TD testTD(200, 0.002);  //���Ӿ��˲�����ͼ��

PID pid_vel_201(11, 0.18, 0, 6000, 16384, 0);  //Ħ����
PID pid_vel_202(11, 0.18, 0, 6000, 16384, 0);  //Ħ����
PID pid_pos_204(0.35, 0, 0.5, 0, 30000, 0);    //�����⻷
PID pid_vel_204(2, 0, 9, 0, 30000, 0);         //�����ڻ�
PID pid_pos_205(100, 2, 0, 100, 3000, 50);     //yaw�⻷
PID pid_vel_205(15, 0, 0, 0, 30000, 0);        //yaw�ڻ�
PID pid_pos_206(150, 0, 0, 0, 5000, 25);       //pitch�⻷
PID pid_vel_206(20, 0, 0, 0, 30000, 0);        //pitch�ڻ�

DR16 dr16;                               //ң�������ջ�
CH110 ch110;                             //IMU
StateMachine state_machine;              //״̬��
Vision vision;                           //�Ӿ�
EmpiricalGravityCompensator EGC(-3000);  //��������
Communicator comm;                       //���ͨ��
RotateFeedForward RFF(1.7);              //С����ʱYAW��ǰ��
ErrorHandle error_handle;                //������