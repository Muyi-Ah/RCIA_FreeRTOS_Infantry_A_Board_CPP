#include "communication.hpp"
#include <cstdint>
#include "gimbal.hpp"
#include "string.h"
#include "uart.hpp"
#include "variables.hpp"

extern bool friction_is_enable;  //判断摩擦轮是否开启

static int32_t summation(const uint8_t* buf, uint16_t lenght) {
    int32_t result = 0;
    for (uint8_t index = 0; index < lenght - 4; index++) {
        result += buf[index];
    }
    return result;
}

static void AppendAccumulationCheckSum(uint8_t* buf, uint16_t length) {
    int32_t result = 0;

    result = summation(buf, length);

    buf[length - 4] = result;
    buf[length - 3] = result >> 8;
    buf[length - 2] = result >> 16;
    buf[length - 1] = result >> 24;
}

static uint8_t VerifyAccumulationCheckSum(const uint8_t* buf, uint16_t lenght) {
    int32_t buffer_result;
    int32_t accumulation_result;

    buffer_result =
        buf[lenght - 4] | buf[lenght - 3] << 8 | buf[lenght - 2] << 16 | buf[lenght - 1] << 24;

    accumulation_result = summation(buf, lenght);

    if (buffer_result == accumulation_result) {
        return true;
    } else {
        return false;
    }
}

void Communicator::RecvUpdate(const uint8_t* buf) {
    if (VerifyAccumulationCheckSum(buf, kCommRecvSize)) {
        memcpy(&RFF.vw, buf, sizeof(float));
    }
}

uint8_t len;
float theta;
uint8_t send_str2[(2 + 1) * 4] = {0};
uint8_t buf[sizeof(dr16) + sizeof(float) + sizeof(bool) + sizeof(enum AimType) + sizeof(int16_t) +
            sizeof(bool) + 4]{0};
void Communicator::Send() {

    theta = CalculateTheta(motor_206.encoder_value_, kYawInitialEncoderValue);

    // *((float*)&send_str2[0 * 4]) = (float)(vision.pitch_hub_increment_temp);
    // *((float*)&send_str2[1 * 4]) = (float)(vision.yaw_hub_increment_temp);
    // *((uint32_t*)&send_str2[sizeof(float) * (2)]) = 0x7f800000;
    // HAL_UART_Transmit_DMA(&huart6, send_str2, sizeof(float) * (2 + 1));

    memcpy(buf, &dr16, sizeof(dr16));                   //写入DR16数据
    memcpy(buf + sizeof(dr16), &theta, sizeof(theta));  //写入底盘夹角
    memcpy(buf + sizeof(dr16) + sizeof(theta), &vision.is_use_,
           sizeof(vision.is_use_));  //写入视觉使能位
    memcpy(buf + sizeof(dr16) + sizeof(theta) + sizeof(vision.is_use_), &vision.aim_type_,
           sizeof(vision.aim_type_));  //写入视觉瞄准类型
    memcpy(buf + sizeof(dr16) + sizeof(theta) + sizeof(vision.is_use_) + sizeof(vision.aim_type_),
           &friction_is_enable, sizeof(friction_is_enable));  //写入摩擦轮使能位
    memcpy(buf + sizeof(dr16) + sizeof(theta) + sizeof(vision.is_use_) + sizeof(vision.aim_type_) +
               sizeof(friction_is_enable),
           &vision.is_aimed_, sizeof(vision.is_aimed_));  //写入视觉瞄准状态位
    AppendAccumulationCheckSum(buf, sizeof(buf));         //写入校验值

    HAL_UART_Transmit_DMA(kCommUart, buf, sizeof(buf));
    len = sizeof(buf);
}
