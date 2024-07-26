#pragma once
#include "can.hpp"
#include "uart.hpp"

void CanCallBack(CAN_HandleTypeDef* _hcan);

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

void MainCallBack();

#ifdef __cplusplus
}
#endif  // __cplusplus
