#include "can.hpp"
#include "call_back.hpp"
#include "error_handle.hpp"

//constexpr CAN_HandleTypeDef* kMotorCan = &hcan1;
//constexpr CAN_HandleTypeDef* kImuCan = &hcan2;

void CanInit() {
    //注册中断回调函数 @warning 中断注册须在启动接收前
    HAL_CAN_RegisterCallback(kMotorCan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CanCallBack);
    HAL_CAN_RegisterCallback(kImuCan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CanCallBack);

    CAN_FilterTypeDef CAN_FilterInitStructure1{0};

    CAN_FilterInitStructure1.FilterIdHigh = 0x0000;
    CAN_FilterInitStructure1.FilterIdLow = 0x0000;
    CAN_FilterInitStructure1.FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure1.FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure1.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CAN_FilterInitStructure1.FilterBank = 0;
    CAN_FilterInitStructure1.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterInitStructure1.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterInitStructure1.FilterActivation = CAN_FILTER_ENABLE;
    CAN_FilterInitStructure1.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(kMotorCan, &CAN_FilterInitStructure1) != HAL_OK) {
        ErrorHandle(kHalLibError);
    }
    if (HAL_CAN_ActivateNotification(kMotorCan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        ErrorHandle(kHalLibError);
    }
    if (HAL_CAN_Start(kMotorCan) != HAL_OK) {
        ErrorHandle(kHalLibError);
    }

    CAN_FilterTypeDef CAN_FilterInitStructure2{0};

    CAN_FilterInitStructure2.FilterIdHigh = 0x0000;
    CAN_FilterInitStructure2.FilterIdLow = 0x0000;
    CAN_FilterInitStructure2.FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure2.FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure2.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CAN_FilterInitStructure2.FilterBank = 15;
    CAN_FilterInitStructure2.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterInitStructure2.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterInitStructure2.FilterActivation = CAN_FILTER_ENABLE;
    CAN_FilterInitStructure2.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(kImuCan, &CAN_FilterInitStructure2) != HAL_OK) {
        ErrorHandle(kHalLibError);
    }
    if (HAL_CAN_ActivateNotification(kImuCan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        ErrorHandle(kHalLibError);
    }
    if (HAL_CAN_Start(kImuCan) != HAL_OK) {
        ErrorHandle(kHalLibError);
    }
}