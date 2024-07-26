#pragma once
#include "ch110.hpp"
#include "dji_motor.hpp"
#include "dr16.hpp"
#include "pid.hpp"
#include "state_machine.hpp"
#include "tracking_differentiator.hpp"
#include "vision.hpp"
#include "gravity_compensation.hpp"
#include "communication.hpp"
#include "rotate_feedforward.hpp"
#include "error_handle.hpp"
#include "config.hpp"

constexpr uint16_t kYawInitialEncoderValue = 2678;

extern DR16 dr16;
extern CH110 ch110;
extern DjiMotor motor_201;
extern DjiMotor motor_202;
extern DjiMotor motor_204;
extern DjiMotor motor_205;
extern DjiMotor motor_206;
extern DjiMotor* dji_motor_list[kMotorCount];

extern TD td_201;
extern TD td_202;
extern TD td_204;
extern TD td_205;
extern TD td_206;

extern TD testTD;

extern PID pid_vel_201;
extern PID pid_vel_202;
extern PID pid_pos_204;
extern PID pid_vel_204;
extern PID pid_pos_205;
extern PID pid_vel_205;
extern PID pid_pos_206;
extern PID pid_vel_206;

extern StateMachine state_machine;
extern Vision vision;
extern EmpiricalGravityCompensator EGC;
extern Communicator comm;
extern RotateFeedForward RFF;
extern ErrorHandle error_handle;