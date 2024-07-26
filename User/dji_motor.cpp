#include "dji_motor.hpp"
#include <stdlib.h>
#include "can.h"
#include "can.hpp"
#include "error_handle.hpp"
#include "variables.hpp"

/**
 * @brief 编码器实际改变值计算
 * @return 
 */
int16_t DjiMotor::IntegralErrorCompute() {
    int16_t res1 = 0, res2 = 0;
    if (encoder_value_ - encoder_value_prev_ > 0) {
        res1 = encoder_value_ - encoder_value_prev_ - 8192;
        res2 = encoder_value_ - encoder_value_prev_;
        encoder_value_prev_ = encoder_value_;  //更新上一次值
        if (abs(res1) < abs(res2)) {
            return res1;
        } else {
            return res2;
        }

    } else {
        res1 = encoder_value_ - encoder_value_prev_ + 8192;
        res2 = encoder_value_ - encoder_value_prev_;
        encoder_value_prev_ = encoder_value_;  //更新上一次值
        if (abs(res1) < abs(res2)) {
            return res1;
        } else {
            return res2;
        }
    }
}

/**
 * @brief 电机数据更新
 * @param buf 数据地址
 */
void DjiMotor::DataUpdate(volatile const uint8_t* buf) {
    encoder_value_ = buf[0] << 8 | buf[1];
    actual_rpm_ = (int16_t)buf[2] << 8 | buf[3];
    actual_current_ = (int16_t)buf[4] << 8 | buf[5];
    temperatrue_ = buf[6];
    if (is_first_recv_done_ == false) {
        encoder_value_prev_ = encoder_value_;
        is_first_recv_done_ = true;
    } else {
        encoder_integral_ += IntegralErrorCompute();
        //@warning 务必在编码器值更新之后再使用integral_error_compute更新编码器积分值}
    }
}

/**
 * @brief 基于目标值的编码器误差计算
 * @param target 目标值
 * @param direction 旋转方向
 * @return 误差值
 */
int32_t DjiMotor::AbsoluteErrorCompute(uint16_t target, enum DirectionType direction) const {
    int32_t small_value = 0;
    int32_t big_value = 0;

    switch (direction) {
        case kBoth:  //两边都能转
            if (target < encoder_value_) {
                small_value = target;
                big_value = encoder_value_;
            } else {
                small_value = encoder_value_;
                big_value = target;
            }

            if (big_value - small_value < 8192 - abs(small_value - big_value)) {
                if (target < encoder_value_) {
                    return -(big_value - small_value);
                } else {
                    return big_value - small_value;
                }
            } else {
                if (target < encoder_value_) {
                    return 8192 - abs(small_value - big_value);
                } else {
                    return -(8192 - abs(small_value - big_value));
                }
            }
            break;

        case kCW:  //顺时针转
            if (target < encoder_value_) {
                small_value = target;
                big_value = encoder_value_;
            } else {
                small_value = encoder_value_;
                big_value = target;
            }

            if (target < encoder_value_) {
                return 8192 - abs(small_value - big_value);
            } else {
                return big_value - small_value;
            }
            break;

        case kCCW:  //逆时针转
            if (target < encoder_value_) {
                small_value = target;
                big_value = encoder_value_;
            } else {
                small_value = encoder_value_;
                big_value = target;
            }

            if (target < encoder_value_) {
                return -(big_value - small_value);
            } else {
                return -(8192 - abs(small_value - big_value));
            }
            break;

        default:
            //ErrorHandle(kSwitchError);  //错误处理
            break;
    }
}

uint8_t motor_tx_buf[2][8]{0};  //@notice 第一个是0x200用的，第二个是0x1FF用的
void DjiMotorSend() {

    //各电机输入值写入电机控制报文
    for (uint8_t i = 0; i < kMotorCount; i++) {
        switch (dji_motor_list[i]->recv_id_) {
            case 0x201:
                if (dji_motor_list[i]->is_enable_) {
                    motor_tx_buf[0][0] = dji_motor_list[i]->input_ >> 8;
                    motor_tx_buf[0][1] = dji_motor_list[i]->input_;
                } else {
                    motor_tx_buf[0][0] = 0;
                    motor_tx_buf[0][1] = 0;
                }
                break;
            case 0x202:
                if (dji_motor_list[i]->is_enable_) {
                    motor_tx_buf[0][2] = dji_motor_list[i]->input_ >> 8;
                    motor_tx_buf[0][3] = dji_motor_list[i]->input_;
                } else {
                    motor_tx_buf[0][2] = 0;
                    motor_tx_buf[0][3] = 0;
                }
                break;
            case 0x203:
                if (dji_motor_list[i]->is_enable_) {
                    motor_tx_buf[0][4] = dji_motor_list[i]->input_ >> 8;
                    motor_tx_buf[0][5] = dji_motor_list[i]->input_;
                } else {
                    motor_tx_buf[0][4] = 0;
                    motor_tx_buf[0][5] = 0;
                }
                break;
            case 0x204:
                if (dji_motor_list[i]->is_enable_) {
                    motor_tx_buf[0][6] = dji_motor_list[i]->input_ >> 8;
                    motor_tx_buf[0][7] = dji_motor_list[i]->input_;
                } else {
                    motor_tx_buf[0][6] = 0;
                    motor_tx_buf[0][7] = 0;
                }
                break;
            case 0x205:
                if (dji_motor_list[i]->is_enable_) {
                    motor_tx_buf[1][0] = dji_motor_list[i]->input_ >> 8;
                    motor_tx_buf[1][1] = dji_motor_list[i]->input_;
                } else {
                    motor_tx_buf[1][0] = 0;
                    motor_tx_buf[1][1] = 0;
                }
                break;
            case 0x206:
                if (dji_motor_list[i]->is_enable_) {
                    motor_tx_buf[1][2] = dji_motor_list[i]->input_ >> 8;
                    motor_tx_buf[1][3] = dji_motor_list[i]->input_;
                } else {
                    motor_tx_buf[1][2] = 0;
                    motor_tx_buf[1][3] = 0;
                }
                break;
            case 0x207:
                if (dji_motor_list[i]->is_enable_) {
                    motor_tx_buf[1][4] = dji_motor_list[i]->input_ >> 8;
                    motor_tx_buf[1][5] = dji_motor_list[i]->input_;
                } else {
                    motor_tx_buf[1][4] = 0;
                    motor_tx_buf[1][5] = 0;
                }
                break;
            case 0x208:
                if (dji_motor_list[i]->is_enable_) {
                    motor_tx_buf[1][6] = dji_motor_list[i]->input_ >> 8;
                    motor_tx_buf[1][7] = dji_motor_list[i]->input_;
                } else {
                    motor_tx_buf[1][6] = 0;
                    motor_tx_buf[1][7] = 0;
                }
                break;

            default:
                //ErrorHandle(kSwitchError);  //错误处理
                break;
        }
    }

    CAN_TxHeaderTypeDef tx_header[2]{0};

    tx_header[0].StdId = 0x200;
    tx_header[0].ExtId = 0;
    tx_header[0].IDE = CAN_ID_STD;
    tx_header[0].RTR = CAN_RTR_DATA;
    tx_header[0].DLC = 8;
    tx_header[0].TransmitGlobalTime = DISABLE;

    tx_header[1].StdId = 0x1FF;
    tx_header[1].ExtId = 0;
    tx_header[1].IDE = CAN_ID_STD;
    tx_header[1].RTR = CAN_RTR_DATA;
    tx_header[1].DLC = 8;
    tx_header[1].TransmitGlobalTime = DISABLE;

    //@warning 一个电机进行发送即可 不要多个电机对象发送 因为几个电机都使用同一个CAN
    HAL_CAN_AddTxMessage(kMotorCan, &tx_header[0], motor_tx_buf[0], (uint32_t*)CAN_TX_MAILBOX0);
    HAL_CAN_AddTxMessage(kMotorCan, &tx_header[1], motor_tx_buf[1], (uint32_t*)CAN_TX_MAILBOX1);
}

static float EncoderToAngle(uint16_t encoder_value) {
    return ((float)encoder_value / 8191) * 360.0f;
}

float CalculateTheta(uint16_t encoder_value, uint16_t initial_value) {
    float current_angle = EncoderToAngle(encoder_value);
    float initial_angle = EncoderToAngle(initial_value);

    float theta = current_angle - initial_angle;

    if (theta < 0) {
        theta += 360.0f;
    } else if (theta >= 360.0f) {
        theta -= 360.0f;
    }

    return theta;
}
