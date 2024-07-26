#pragma once
#include "can.h"

constexpr CAN_HandleTypeDef* kMotorCan = &hcan1;
constexpr CAN_HandleTypeDef* kImuCan = &hcan2;

void CanInit();
