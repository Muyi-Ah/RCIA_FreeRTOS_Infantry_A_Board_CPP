#include "uart.hpp"
#include "call_back.hpp"
#include "error_handle.hpp"

uint8_t remote_rx_buf[kRemoteSize];  //遥控器数据接收数组
uint8_t comm_rx_buf[kCommRecvSize];      //板间通信数据接收数组
uint8_t vision_rx_buf[kVisionRecvSize];  //视觉数据接收数组

    /**
 * 串口初始化.
 */
void UartInit() {
    //注册中断回调函数 @warning 中断注册须在启动接收前
    //HAL_UART_RegisterRxEventCallback(kRemoteUart, UartCallBack);
    //HAL_UART_RegisterRxEventCallback(kCommUart, UartCallBack);
    //HAL_UART_RegisterRxEventCallback(kVisionUart, UartCallBack);

    if (HAL_UARTEx_ReceiveToIdle_DMA(kRemoteUart, remote_rx_buf, kRemoteSize) != HAL_OK) {
        /*ErrorHandle(kHalLibError);*/
    }
    if (HAL_UARTEx_ReceiveToIdle_DMA(kCommUart, comm_rx_buf, kCommRecvSize) != HAL_OK) {
        /*ErrorHandle(kHalLibError);*/
    }
    if (HAL_UARTEx_ReceiveToIdle_DMA(kVisionUart, vision_rx_buf, kVisionRecvSize) != HAL_OK) {
        /*ErrorHandle(kHalLibError);*/
    }
}
