#include "gimbal.hpp"
#include "clamp.hpp"
#include "cmsis_os2.h"
#include "stdint.h"
#include "stdlib.h"
#include "uart.hpp"
#include "variables.hpp"

/*  =========================== �������� ===========================  */

/*constexpr */ auto kRemoteDeadBand = 10;               //ң��������
/*constexpr */ auto kRemoteYawCoefficient = 0.0004f;    //ң����YAW��Ӧϵ��
/*constexpr */ auto kRemotePitchCoefficient = 0.0001f;  //ң����PITCH��Ӧϵ��
/*constexpr */ auto kMouseYawCoefficient = 0.0003f;     //���YAW��Ӧϵ��
/*constexpr */ auto kMousePitchCoefficient = 0.0002f;   //���PITCH��Ӧϵ��
/*constexpr */ auto kShotsPerFire = 1;                  //ÿ�����������
/*constexpr */ auto kFrictionRpm = 7000;                //Ħ����ת��

float vision_yaw_coefficient = 0.035f;
float vision_pitch_coefficient = 0.015f;

/*  =========================== �������� ===========================  */

static void ORE_Solve();
void HaltFunction();
void SubMode11Function();
void SubMode12Function();
void SubMode13Function();
void SubMode21Function();
void SubMode22Function();
void SubMode23Function();
void SubMode31Function();
void SubMode32Function();
void SubMode33Function();
void TimeStampClear();
void SubStateUpdate();
static void blocking_check();

/*  =========================== �������� ===========================  */

//������ģʽʱ���
uint32_t enter_mode_11_timestamp;
uint32_t enter_mode_12_timestamp;
uint32_t enter_mode_13_timestamp;
uint32_t enter_mode_21_timestamp;
uint32_t enter_mode_22_timestamp;
uint32_t enter_mode_23_timestamp;
uint32_t enter_mode_31_timestamp;
uint32_t enter_mode_32_timestamp;
uint32_t enter_mode_33_timestamp;

//��ģʽʱ���
uint32_t mode_11_timestamp;
uint32_t mode_12_timestamp;
uint32_t mode_13_timestamp;
uint32_t mode_21_timestamp;
uint32_t mode_22_timestamp;
uint32_t mode_23_timestamp;
uint32_t mode_31_timestamp;
uint32_t mode_32_timestamp;
uint32_t mode_33_timestamp;

float yaw_target_euler = 0;       //YAW��Ŀ��ŷ����
float pitch_target_euler = 0;     //PITCH��Ŀ��ŷ����
int16_t friction_target_rpm = 0;  //Ħ����Ŀ��ת��
int32_t trigger_target_pos = 0;   //����Ŀ��λ��

bool F_latch;

void GimbalTask(void* argument) {
    for (;;) {

        ORE_Solve();

        SubStateUpdate();

        //״̬��
        switch (state_machine.main_state_) {
            case kMainStateNone:
                motor_201.is_enable_ = false;
                motor_202.is_enable_ = false;
                motor_204.is_enable_ = false;
                motor_205.is_enable_ = false;
                motor_206.is_enable_ = false;
                state_machine.HandleEvent(kEventEnterOperate);
                break;

            case kOperate:  //����ģʽ

                switch (state_machine.sub_state_) {
                    case kSubStateNone:
                        motor_201.is_enable_ = false;
                        motor_202.is_enable_ = false;
                        motor_204.is_enable_ = false;
                        motor_205.is_enable_ = false;
                        motor_206.is_enable_ = false;
                        state_machine.HandleEvent(kEventEnterHalt);
                        break;

                    case kSubMode11:  //�е�ά��ģʽ
                        state_machine.HandleEvent(kEventEnterHalt);
                        SubMode11Function();
                        break;

                    case kSubMode12:  //ң����С����˳ʱ��ģʽ
                        SubMode12Function();
                        motor_201.is_enable_ = true;
                        motor_202.is_enable_ = true;
                        motor_204.is_enable_ = true;
                        motor_205.is_enable_ = true;
                        motor_206.is_enable_ = true;
                        break;

                    case kSubMode13:  //ң����С������ʱ��ģʽ
                        SubMode13Function();
                        motor_201.is_enable_ = true;
                        motor_202.is_enable_ = true;
                        motor_204.is_enable_ = true;
                        motor_205.is_enable_ = true;
                        motor_206.is_enable_ = true;
                        break;

                    case kSubMode21:
                    case kSubMode22:
                    case kSubMode23:  //ת��ģʽ
                        SubMode23Function();
                        motor_201.is_enable_ = true;
                        motor_202.is_enable_ = true;
                        motor_204.is_enable_ = true;
                        motor_205.is_enable_ = true;
                        motor_206.is_enable_ = true;
                        break;

                    case kSubMode31:
                        SubMode31Function();
                        motor_201.is_enable_ = true;
                        motor_202.is_enable_ = true;
                        motor_204.is_enable_ = true;
                        motor_205.is_enable_ = true;
                        motor_206.is_enable_ = true;
                        break;

                    case kSubMode32:  //ң�����ģʽ
                        SubMode32Function();
                        motor_201.is_enable_ = true;
                        motor_202.is_enable_ = true;
                        motor_204.is_enable_ = true;
                        motor_205.is_enable_ = true;
                        motor_206.is_enable_ = true;
                        break;

                    case kSubMode33:  //����ģʽ
                        SubMode33Function();
                        motor_201.is_enable_ = true;
                        motor_202.is_enable_ = true;
                        motor_204.is_enable_ = true;
                        motor_205.is_enable_ = true;
                        motor_206.is_enable_ = true;
                        break;

                    default:
                        /* code */
                        break;
                }

                break;  // kOperate

            case kHalt:  //ά��ģʽ
                motor_201.is_enable_ = false;
                motor_202.is_enable_ = false;
                motor_204.is_enable_ = false;
                motor_205.is_enable_ = false;
                motor_206.is_enable_ = false;
                HaltFunction();

                switch (state_machine.sub_state_) {
                    case kSubMode12:  //ң����С����˳ʱ��ģʽ
                        state_machine.HandleEvent(kEventEnterOperate);
                        break;

                    case kSubMode13:  //ң����С������ʱ��ģʽ
                        state_machine.HandleEvent(kEventEnterOperate);
                        break;

                    case kSubMode21:
                    case kSubMode22:
                    case kSubMode23:  //ת��ģʽ
                        state_machine.HandleEvent(kEventEnterOperate);
                        break;

                    case kSubMode31:
                    case kSubMode32:  //ң�����ģʽ
                        state_machine.HandleEvent(kEventEnterOperate);
                        break;

                    case kSubMode33:  //����ģʽ
                        state_machine.HandleEvent(kEventEnterOperate);
                        break;

                    default:
                        break;
                }

                break;

            default:
                /* code */
                break;

        }  // state machine

        blocking_check();

        //pitch��ֵ
        pitch_target_euler = clamp(pitch_target_euler, kLowestEuler, kHighestEuler);
        //yaw��ֵ
        yaw_target_euler =
            clamp(yaw_target_euler, (ch110.yaw_integral_ - 180.0f), (ch110.yaw_integral_ + 180.0f));

        TimeStampClear();  //ʱ������

        osDelay(1);  //��ʱ
    }
}

static void RemoteTargetHandle() {
    if (dr16.remote_.ch0_ < 1024 - kRemoteDeadBand || dr16.remote_.ch0_ > 1024 + kRemoteDeadBand) {
        yaw_target_euler -= (float)((dr16.remote_.ch0_ - 1024) * kRemoteYawCoefficient);
    }
    if (dr16.remote_.ch1_ < 1024 - kRemoteDeadBand || dr16.remote_.ch1_ > 1024 + kRemoteDeadBand) {
        pitch_target_euler -= (float)((dr16.remote_.ch1_ - 1024) * kRemotePitchCoefficient);
    }
}

void HaltFunction() {

    //�ý׶���YAW��Ŀ��ֵ����ʵ��ֵһ��
    yaw_target_euler = ch110.yaw_integral_;
    /*pitch_target_euler = clamp(ch110.roll_, -5.0f, 30.0f);*/

    //�ý׶��²��̵�Ŀ��ֵ����ʵ��ֵһ��
    trigger_target_pos = motor_204.encoder_integral_;
}

void SubMode11Function() {}
void SubMode12Function() {
    vision.is_use_ = false;  //�Ӿ�ʹ�ñ�־λ��0
    friction_target_rpm = 0;
    RemoteTargetHandle();  //Ŀ��ֵ����
}
void SubMode13Function() {
    vision.is_use_ = false;  //�Ӿ�ʹ�ñ�־λ��0
    friction_target_rpm = 0;
    RemoteTargetHandle();  //Ŀ��ֵ����
}
void SubMode21Function() {}
void SubMode22Function() {}
void SubMode23Function() {
    vision.is_use_ = false;  //�Ӿ�ʹ�ñ�־λ��0
    friction_target_rpm = 0;
    RemoteTargetHandle();  //Ŀ��ֵ����
}
void SubMode31Function() {
    RemoteTargetHandle();
    friction_target_rpm = 0;
    vision.is_use_ = true;  //�Ӿ�ʹ�ñ�־λ��1

    //if (vision.aim_type_ == kArmor) {
    //    yaw_target_euler -= vision.yaw_increament * vision_yaw_coefficient;
    //    pitch_target_euler -= vision.pitch_increment * vision_pitch_coefficient;
    //} else {
    //    yaw_target_euler -= vision.yaw_hub_increment * vision_yaw_coefficient;
    //    pitch_target_euler -= vision.pitch_hub_increment * vision_pitch_coefficient;
    //}
}
void SubMode32Function() {
    vision.is_use_ = true;  //�Ӿ�ʹ�ñ�־λ��0
    RemoteTargetHandle();   //Ŀ��ֵ����

    //��ȡϵͳʱ�������λΪms
    auto current_timestamp = HAL_GetTick();

    if (enter_mode_32_timestamp == 0) {  //�ս����ģʽ
        enter_mode_32_timestamp = current_timestamp;
        friction_target_rpm = 0;  //Ħ����ͣת
    }
    //1���
    else if ((current_timestamp - enter_mode_32_timestamp) >= 1000) {
        friction_target_rpm = kFrictionRpm;  //Ħ��������

        if (vision.aim_type_ == kArmor) {
            //ÿ500ms
            if (dr16.remote_.ch3_ > 1684 - kRemoteDeadBand &&
                current_timestamp - mode_32_timestamp >= 200) {
                trigger_target_pos -=
                    49152 * kShotsPerFire;  //����ʹ��M2006 P36�Ҳ���ÿȦ6�ŵ��� :8192*36/6=49152
                mode_32_timestamp = current_timestamp;
            }
        } else {
            if (dr16.remote_.ch3_ > 1684 - kRemoteDeadBand && vision.fire_flag == true &&
                vision.fire_latch == false) {
                trigger_target_pos -=
                    49152 * kShotsPerFire;  //����ʹ��M2006 P36�Ҳ���ÿȦ6�ŵ��� :8192*36/6=49152
                vision.fire_latch = true;
            } else if (vision.fire_flag == false) {
                vision.fire_latch = false;
            }
        }
    }
}
void SubMode33Function() {

    //��ȡϵͳʱ�������λΪms
    auto current_timestamp = HAL_GetTick();

    if (enter_mode_33_timestamp == 0) {  //�ս����ģʽ
        enter_mode_33_timestamp = current_timestamp;
        friction_target_rpm = 0;  //Ħ����ͣת
    }
    //һ���
    else if ((current_timestamp - enter_mode_33_timestamp) >= 1000) {

        //G������
        if (dr16.KeyBoard_.key_.G_key) {
            friction_target_rpm = kFrictionRpm;  //Ħ��������
        }

        //B������
        if (dr16.KeyBoard_.key_.B_key) {
            friction_target_rpm = 0;  //Ħ���ֹر�
        }

        //F������ ��׼ģʽ�л�
        if (dr16.KeyBoard_.key_.F_key && F_latch == false) {
            F_latch = true;
            if (vision.aim_type_ == kArmor) {
                vision.aim_type_ = kRobotHub;
            } else {
                vision.aim_type_ = kArmor;
            }
        } else if (dr16.KeyBoard_.key_.F_key == 0) {
            F_latch = false;
        }

        //������������Ħ���ֿ���
        if (dr16.mouse_.press_left_ && friction_target_rpm != 0) {
            //ÿ100ms
            if (vision.aim_type_ == kArmor) {
                if (current_timestamp - mode_33_timestamp >= 50) {
                    //����ʹ��M2006 P36�Ҳ���ÿȦ6�ŵ��� :8192*36/6=49152
                    trigger_target_pos -= 49152 * kShotsPerFire;
                    mode_33_timestamp = current_timestamp;
                }
            } else {
                if (vision.fire_flag == true && vision.fire_latch == false) {
                    trigger_target_pos -=
                        49152 *
                        kShotsPerFire;  //����ʹ��M2006 P36�Ҳ���ÿȦ6�ŵ��� :8192*36/6=49152
                    vision.fire_latch = true;
                } else if (vision.fire_flag == false) {
                    vision.fire_latch = false;
                }
            }
        }
        //����Ҽ�����������׼Ŀ��
        if (dr16.mouse_.press_right_) {
            vision.is_use_ = true;  //�Ӿ�ʹ�ñ�־λ��1
        }
        //δ�����Ӿ�
        else {
            vision.is_use_ = false;  //�Ӿ�ʹ�ñ�־λ��0
        }

        //������
        if (dr16.mouse_.x_axis_) {
            yaw_target_euler -= (float)(dr16.mouse_.x_axis_ * kMouseYawCoefficient);
        }
        //�������
        if (dr16.mouse_.y_axis_) {
            pitch_target_euler += (float)(dr16.mouse_.y_axis_ * kMousePitchCoefficient);
        }
    }
}

static void ORE_Solve() {
    //�������ORE����
    if (__HAL_UART_GET_FLAG(kRemoteUart, UART_FLAG_ORE) != RESET) {

        __HAL_UART_CLEAR_OREFLAG(kRemoteUart);  //���OREλ

        //������������
        HAL_UARTEx_ReceiveToIdle_DMA(kRemoteUart, remote_rx_buf, kRemoteSize);
    }

    //�������ORE����
    if (__HAL_UART_GET_FLAG(kCommUart, UART_FLAG_ORE) != RESET) {

        __HAL_UART_CLEAR_OREFLAG(kCommUart);  //���OREλ

        //������������
        HAL_UARTEx_ReceiveToIdle_DMA(kCommUart, comm_rx_buf, kCommRecvSize);
    }

    //�������ORE����
    if (__HAL_UART_GET_FLAG(kVisionUart, UART_FLAG_ORE) != RESET) {

        __HAL_UART_CLEAR_OREFLAG(kVisionUart);  //���OREλ

        //������������
        HAL_UARTEx_ReceiveToIdle_DMA(kVisionUart, vision_rx_buf, kVisionRecvSize);
    }
}

void TimeStampClear() {
    if (state_machine.sub_state_ != kSubMode11) {
        enter_mode_11_timestamp = 0;
        mode_11_timestamp = 0;
    }
    if (state_machine.sub_state_ != kSubMode12) {
        enter_mode_12_timestamp = 0;
        mode_12_timestamp = 0;
    }
    if (state_machine.sub_state_ != kSubMode13) {
        enter_mode_13_timestamp = 0;
        mode_13_timestamp = 0;
    }
    if (state_machine.sub_state_ != kSubMode21) {
        enter_mode_21_timestamp = 0;
        mode_21_timestamp = 0;
    }
    if (state_machine.sub_state_ != kSubMode22) {
        enter_mode_22_timestamp = 0;
        mode_22_timestamp = 0;
    }
    if (state_machine.sub_state_ != kSubMode23) {
        enter_mode_23_timestamp = 0;
        mode_23_timestamp = 0;
    }
    if (state_machine.sub_state_ != kSubMode31) {
        enter_mode_31_timestamp = 0;
        mode_31_timestamp = 0;
    }
    if (state_machine.sub_state_ != kSubMode32) {
        enter_mode_32_timestamp = 0;
        mode_32_timestamp = 0;
    }
    if (state_machine.sub_state_ != kSubMode33) {
        enter_mode_33_timestamp = 0;
        mode_33_timestamp = 0;
    }
}

void SubStateUpdate() {
    switch (dr16.remote_.s1_) {
        case 1:
            switch (dr16.remote_.s2_) {
                case 1:
                    state_machine.HandleEvent(kEventSwitchSubMode11);
                    break;
                case 3:
                    state_machine.HandleEvent(kEventSwitchSubMode12);
                    break;
                case 2:
                    state_machine.HandleEvent(kEventSwitchSubMode13);
                    break;

                default:
                    break;
            }
            break;
        case 3:
            switch (dr16.remote_.s2_) {
                case 1:
                    state_machine.HandleEvent(kEventSwitchSubMode21);
                    break;
                case 3:
                    state_machine.HandleEvent(kEventSwitchSubMode22);
                    break;
                case 2:
                    state_machine.HandleEvent(kEventSwitchSubMode23);
                    break;

                default:
                    break;
            }
            break;
        case 2:
            switch (dr16.remote_.s2_) {
                case 1:
                    state_machine.HandleEvent(kEventSwitchSubMode31);
                    break;
                case 3:
                    state_machine.HandleEvent(kEventSwitchSubMode32);
                    break;
                case 2:
                    state_machine.HandleEvent(kEventSwitchSubMode33);
                    break;

                default:
                    break;
            }
            break;

        default:
            break;
    }
}

uint8_t blocking_flag;
uint32_t blocking_time;
uint32_t time;
int64_t angle_sum_prev;
/**
 * @brief �������
 * 
 */
static void blocking_check() {
    //�˴�ת��λ�ô����ϴ�ת��λ����Ȧ
    if (llabs(trigger_target_pos - motor_204.encoder_integral_) > 16384) {
        //δ���˵������У�ÿ200ms���һ���Ƿ񿨵�
        if ((HAL_GetTick() - time) > 200 && blocking_flag == 0) {
            //
            if (llabs(motor_204.encoder_integral_ - angle_sum_prev) < 16384) {
                blocking_flag = 1;
                blocking_time = HAL_GetTick();
            }
            angle_sum_prev = motor_204.encoder_integral_;
            time = HAL_GetTick();
        }
    }
    //�����˵�
    if (blocking_flag) {
        //�˵��ѳ���50ms
        if (HAL_GetTick() - blocking_time > 100) {
            blocking_flag = 0;  //���������־
        }
        //�˵�������
        else {
            //ʵ��ֵ����Ŀ��ֵ ��ֹ�ָ�˲��ת��Ƕ�
            motor_204.encoder_integral_ = trigger_target_pos;
            motor_204.input_ = 5000;  //�������
        }
    }
}