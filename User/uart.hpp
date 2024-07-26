#pragma once
#include "usart.h"

constexpr UART_HandleTypeDef* kRemoteUart = &huart1;  //ң��������
constexpr UART_HandleTypeDef* kCommUart = &huart7;    //���ͨ�Ŵ���
constexpr UART_HandleTypeDef* kVisionUart = &huart8;  //�Ӿ�����

constexpr uint16_t kRemoteSize = 18;   //ң���������ֽ���;
constexpr uint16_t kCommRecvSize = 8;  //���ͨ�������ֽ���
constexpr uint16_t kVisionRecvSize = 23;  //�Ӿ������ֽ���

extern uint8_t remote_rx_buf[kRemoteSize];      //ң�������ݽ�������
extern uint8_t comm_rx_buf[kCommRecvSize];      //���ͨ�����ݽ�������
extern uint8_t vision_rx_buf[kVisionRecvSize];  //�Ӿ����ݽ�������

void UartInit();
