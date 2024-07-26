#pragma once
#include "usart.h"

constexpr UART_HandleTypeDef* kRemoteUart = &huart1;  //遥控器串口
constexpr UART_HandleTypeDef* kCommUart = &huart7;    //板间通信串口
constexpr UART_HandleTypeDef* kVisionUart = &huart8;  //视觉串口

constexpr uint16_t kRemoteSize = 18;   //遥控器数据字节数;
constexpr uint16_t kCommRecvSize = 8;  //板间通信数据字节数
constexpr uint16_t kVisionRecvSize = 23;  //视觉数据字节数

extern uint8_t remote_rx_buf[kRemoteSize];      //遥控器数据接收数组
extern uint8_t comm_rx_buf[kCommRecvSize];      //板间通信数据接收数组
extern uint8_t vision_rx_buf[kVisionRecvSize];  //视觉数据接收数组

void UartInit();
