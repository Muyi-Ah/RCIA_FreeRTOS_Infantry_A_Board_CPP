#include "uart.hpp"
#include "call_back.hpp"
#include "error_handle.hpp"

uint8_t remote_rx_buf[kRemoteSize];  //ң�������ݽ�������
uint8_t comm_rx_buf[kCommRecvSize];      //���ͨ�����ݽ�������
uint8_t vision_rx_buf[kVisionRecvSize];  //�Ӿ����ݽ�������

    /**
 * ���ڳ�ʼ��.
 */
void UartInit() {
    //ע���жϻص����� @warning �ж�ע��������������ǰ
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
