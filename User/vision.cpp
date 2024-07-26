#include "vision.hpp"
#include <stm32f4xx_hal.h>
#include "math.h"
#include "uart.hpp"
#include "variables.hpp"
#include "gimbal.hpp"

//  =============================== 原来的增量式 ================================
//bool Vision::AbsoluteFilte() const {
//    if (fabsf(yaw_increment_temp) < 10 && fabsf(pitch_increment_temp) < 10 &&
//        fabsf(yaw_hub_increment_temp) < 10 && fabsf(pitch_hub_increment_temp)) {
//        return true;
//    } else {
//        return false;
//    }
//}
//  ============================================================================

bool Vision::AbsoluteFilte() const {
    if (fabsf(yaw_increment_temp) < 25 && fabsf(pitch_increment_temp) < 25 &&
        fabsf(yaw_hub_increment_temp) < 25 && fabsf(pitch_hub_increment_temp) < 25) {
        return true;
    } else {
        return false;
    }
}

void Vision::RecvUpdate(const uint8_t* buf) {
    origin_pitch_hub =
        (int16_t)((buf[1] - 48) * 1000 + (buf[2] - 48) * 100 + (buf[3] - 48) * 10 + (buf[4] - 48));
    origin_yaw_hub_ =
        (int16_t)((buf[6] - 48) * 1000 + (buf[7] - 48) * 100 + (buf[8] - 48) * 10 + (buf[9] - 48));
    origin_pitch_ = (int16_t)((buf[11] - 48) * 1000 + (buf[12] - 48) * 100 + (buf[13] - 48) * 10 +
                              (buf[14] - 48));
    origin_yaw_ = (int16_t)((buf[16] - 48) * 1000 + (buf[17] - 48) * 100 + (buf[18] - 48) * 10 +
                            (buf[19] - 48));
    fire_flag = buf[21] - 48;

    is_aimed_ = false;  //先假设接收失败

    //  =============================== 原来的增量式 ================================
    //yaw_increment_temp = (float)((origin_yaw_ - 1000) * 0.04f);
    //pitch_increment_temp = (float)((origin_pitch_ - 1000) * 0.04f);
    //yaw_hub_increment_temp = (float)((origin_yaw_hub_ - 1000) * 0.04f);
    //pitch_hub_increment_temp = (float)((origin_pitch_hub - 1000) * 0.04f);
    //  ============================================================================

    yaw_increment_temp = (float)((origin_yaw_ - 1000) / 10.0f);
    pitch_increment_temp = (float)((origin_pitch_ - 1000) / 10.0f);
    yaw_hub_increment_temp = (float)((origin_yaw_hub_ - 1000) / 10.0f);
    pitch_hub_increment_temp = (float)((origin_pitch_hub - 1000) / 10.0f);

    if (AbsoluteFilte()) {
        is_aimed_ = true;
        if (vision.is_use_) {
            yaw_increament = yaw_increment_temp;
            pitch_increment = pitch_increment_temp;
            yaw_hub_increment = yaw_hub_increment_temp;
            pitch_hub_increment = pitch_hub_increment_temp;

            //  ============================== 绝对角度 add in 2024/7/22 ============================
            if (vision.aim_type_ == kArmor) {
               yaw_target_euler = -yaw_increament + ch110.yaw_integral_;
               pitch_target_euler = -pitch_increment + ch110.roll_;
            } else {
               yaw_target_euler = -yaw_hub_increment + ch110.yaw_integral_;
               pitch_target_euler = -pitch_hub_increment + ch110.roll_;
            }
            //  ====================================================================================
        }

        return;
    }

    if (origin_pitch_ == 0 || origin_pitch_hub == 0 || origin_yaw_ == 0 || origin_yaw_hub_ == 0 ||
        (HAL_GetTick() - recv_time) > 200) {
        yaw_increament = 0;
        pitch_increment = 0;
        yaw_hub_increment = 0;
        pitch_hub_increment = 0;
        fire_flag = false;
        is_aimed_ = false;
    }
}

int16_t yaw;
uint16_t yaw_angle;
uint8_t vision_buf[5]{0};
void Vision::Send() {
    yaw = (int16_t)(ch110.yaw_ * 10.0f);
    yaw += 1800;
    yaw_angle = (uint16_t)yaw;

    auto theta = CalculateTheta(motor_206.encoder_value_, kYawInitialEncoderValue);
    auto chassis_theta = (int16_t)(theta * 10.0f);
    auto chhassis_theta_angle = (uint16_t)chassis_theta;

    vision_buf[0] = 0xFF;
    vision_buf[1] = yaw_angle >> 8;
    vision_buf[2] = yaw_angle << 8 >> 8;
    vision_buf[3] = chhassis_theta_angle >> 8;
    vision_buf[4] = chhassis_theta_angle;

    HAL_UART_Transmit_DMA(kVisionUart, vision_buf, sizeof(vision_buf));
}
