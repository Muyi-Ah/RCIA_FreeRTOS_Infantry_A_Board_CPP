#include "variables.hpp"

DjiMotor motor_201(0x201);  //摩擦轮
DjiMotor motor_202(0x202);  //摩擦轮
DjiMotor motor_204(0x204);  //拨盘
DjiMotor motor_205(0x205);  //pitch
DjiMotor motor_206(0x206);  //yaw
DjiMotor* dji_motor_list[kMotorCount] = {
    &motor_201, &motor_202, &motor_204, &motor_205, &motor_206,
};  //电机数组

//微分跟踪器 用于电机速度滤波
TD td_201(200, 0.001);
TD td_202(200, 0.001);
TD td_204(200, 0.001);
TD td_205(200, 0.001);
TD td_206(200, 0.001);

TD testTD(200, 0.002);  //看视觉滤波曲线图用

PID pid_vel_201(11, 0.18, 0, 6000, 16384, 0);  //摩擦轮
PID pid_vel_202(11, 0.18, 0, 6000, 16384, 0);  //摩擦轮
PID pid_pos_204(0.35, 0, 0.5, 0, 30000, 0);    //拨盘外环
PID pid_vel_204(2, 0, 9, 0, 30000, 0);         //拨盘内环
PID pid_pos_205(100, 2, 0, 100, 3000, 50);     //yaw外环
PID pid_vel_205(15, 0, 0, 0, 30000, 0);        //yaw内环
PID pid_pos_206(150, 0, 0, 0, 5000, 25);       //pitch外环
PID pid_vel_206(20, 0, 0, 0, 30000, 0);        //pitch内环

DR16 dr16;                               //遥控器接收机
CH110 ch110;                             //IMU
StateMachine state_machine;              //状态机
Vision vision;                           //视觉
EmpiricalGravityCompensator EGC(-3000);  //重力补偿
Communicator comm;                       //板间通信
RotateFeedForward RFF(1.7);              //小陀螺时YAW轴前馈
ErrorHandle error_handle;                //错误处理