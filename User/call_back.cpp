#include "call_back.hpp"
#include "clamp.hpp"
#include "error_handle.hpp"
#include "gimbal.hpp"
#include "variables.hpp"

constexpr auto vofa_cnt = 2;

void CanCallBack(CAN_HandleTypeDef* hcan) {
    if (hcan->Instance == kMotorCan->Instance) {

        CAN_RxHeaderTypeDef rx_header{0};
        uint8_t buf[8]{0};

        HAL_CAN_GetRxMessage(kMotorCan, CAN_RX_FIFO0, &rx_header, buf);  //接收数据

        for (uint8_t i = 0; i < kMotorCount; i++) {
            if (rx_header.StdId == dji_motor_list[i]->recv_id_) {
                dji_motor_list[i]->DataUpdate(buf);   //更新电机数据
                dji_motor_list[i]->is_reply_ = true;  //应答
                break;
            }
        }

    } else if (hcan->Instance == kImuCan->Instance) {

        CAN_RxHeaderTypeDef rx_header{0};
        uint8_t buf[8]{0};

        HAL_CAN_GetRxMessage(kImuCan, CAN_RX_FIFO0, &rx_header, buf);  //接收数据

        if (rx_header.StdId == 0x388) {
            ch110.EulerUpdate(buf);  //欧拉角更新
        } else if (rx_header.StdId == 0x288) {
            ch110.VelocityUpdate(buf);  //角速度更新
        }
        ch110.is_reply_ = true;  //应答
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart->Instance == kRemoteUart->Instance) {
        if (Size == kRemoteSize) {
            dr16.DataUpdate(remote_rx_buf);  //遥控器数据更新
            dr16.is_reply_ = true;           //应答
        }
        //重新启动接收
        if (HAL_UARTEx_ReceiveToIdle_DMA(kRemoteUart, remote_rx_buf, kRemoteSize) != HAL_OK) {
            /*ErrorHandle(kHalLibError);*/
        }

    } else if (huart->Instance == kCommUart->Instance && Size == kCommRecvSize) {
        comm.RecvUpdate(comm_rx_buf);  //通信数据更新
        comm.is_reply_ = true;         //应答
        //重新启动接收
        HAL_UARTEx_ReceiveToIdle_DMA(kCommUart, comm_rx_buf, kCommRecvSize);
    } else if (huart->Instance == kVisionUart->Instance) {
        if (Size == kVisionRecvSize) {
            vision.RecvUpdate(vision_rx_buf);  //视觉数据更新
            vision.recv_time = HAL_GetTick();
            vision.is_reply = true;  //应答
        }
        HAL_UARTEx_ReceiveToIdle_DMA(kVisionUart, vision_rx_buf, kVisionRecvSize);
        //重新启动接收
        //if (HAL_UARTEx_ReceiveToIdle_DMA(kVisionUart, vision_rx_buf, kVisionRecvSize) != HAL_OK) {
        //    ErrorHandle(kHalLibError);
        //}
    }
}

extern uint8_t blocking_flag;
void MainCallBack() {
    /*  =========================== 摩擦轮、拨盘电机控制 ===========================  */

    //摩擦轮转速PID
    auto measure_rpm_201 = td_201.Compute(motor_201.actual_rpm_);  //获取微分跟踪器滤波后转速
    auto dji_motor_201_input = pid_vel_201.Compute(friction_target_rpm, measure_rpm_201);
    motor_201.input_ = dji_motor_201_input;  //设置电机输出

    //摩擦轮转速PID
    auto measure_rpm_202 = td_202.Compute(motor_202.actual_rpm_);  //获取微分跟踪器滤波后转速
    auto dji_motor_202_input = pid_vel_202.Compute(-friction_target_rpm, measure_rpm_202);
    motor_202.input_ = dji_motor_202_input;  //设置电机输出

    if (blocking_flag == 0) {
        //拨盘外环PID
        auto temp_204 = pid_pos_204.Compute(trigger_target_pos, motor_204.encoder_integral_);

        //拨盘内环PID
        auto measure_rpm_204 = td_204.Compute(motor_204.actual_rpm_);  //获取微分跟踪器滤波后转速
        auto dji_motor_204_input = pid_vel_204.Compute(temp_204, measure_rpm_204);
        motor_204.input_ = dji_motor_204_input;  //设置电机输出
    }

    /*  =========================== YAW、PITCH电机控制 ===========================  */

    //视觉作用于目标值时需要关闭前馈，不然会有高频振动（不太影响控制但会响声，会影响部件寿命）
    if (vision.is_use_ && vision.is_aimed_ && vision.is_reply) {
        pid_pos_205.k_feed_forward_ = 0;
        pid_pos_206.k_feed_forward_ = 0;
    } else {
        pid_pos_205.k_feed_forward_ = kPitchFeedForward;
        pid_pos_206.k_feed_forward_ = kYawFeedForward;
    }

    //pitch外环PID
    /*auto euler_error_205 = clamp(pitch_target_euler, kLowestEuler, kHighestEuler) - ch110.roll_;*/
    auto temp_205 =
        pid_pos_205.Compute(clamp(pitch_target_euler, kLowestEuler, kHighestEuler), ch110.roll_);

    //pitch内环PID
    auto dji_motor_205_input = pid_vel_205.Compute(temp_205, ch110.y_velocity_);
    motor_205.input_ = dji_motor_205_input;  //设置电机输出
    motor_205.input_ =
        clamp(motor_205.input_ + EGC.Compute(ch110.roll_), -30000.0f, 30000.0f);  //重力补偿

    //yaw外环PID
    yaw_target_euler =
        clamp(yaw_target_euler, ch110.yaw_integral_ - 180.0f, ch110.yaw_integral_ + 180.0f);
    auto temp_206 = pid_pos_206.Compute(yaw_target_euler, ch110.yaw_integral_);

    //yaw内环PID
    auto dji_motor_206_input = pid_vel_206.Compute(temp_206, ch110.z_velocity_);
    motor_206.input_ = clamp(dji_motor_206_input + RFF.Compute(), -30000.0f, 30000.0f);

    DjiMotorSend();  //电机数据发送

    //uint8_t send_str2[(vofa_cnt + 1) * 4] = {0};
    //*((float*)&send_str2[0 * 4]) = (float)ch110.z_velocity_;
    //*((float*)&send_str2[1 * 4]) = (float)0;
    //*((uint32_t*)&send_str2[sizeof(float) * (vofa_cnt)]) = 0x7f800000;
    ////开始发送数据
    //HAL_UART_Transmit_DMA(&huart6, send_str2, sizeof(float) * (vofa_cnt + 1));
}
