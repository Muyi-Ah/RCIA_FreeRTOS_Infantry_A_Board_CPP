#include "gimbal.hpp"
#include "clamp.hpp"
#include "cmsis_os2.h"
#include "stdint.h"
#include "stdlib.h"
#include "uart.hpp"
#include "variables.hpp"

/*  =========================== 常量定义 ===========================  */

/*constexpr */ auto kRemoteDeadBand = 10;               //遥控器死区
/*constexpr */ auto kRemoteYawCoefficient = 0.0004f;    //遥控器YAW响应系数
/*constexpr */ auto kRemotePitchCoefficient = 0.0001f;  //遥控器PITCH响应系数
/*constexpr */ auto kMouseYawCoefficient = 0.0003f;     //鼠标YAW响应系数
/*constexpr */ auto kMousePitchCoefficient = 0.0002f;   //鼠标PITCH响应系数
/*constexpr */ auto kShotsPerFire = 1;                  //每次射击弹丸数
/*constexpr */ auto kFrictionRpm = 7000;                //摩擦轮转速

float vision_yaw_coefficient = 0.035f;
float vision_pitch_coefficient = 0.015f;

/*  =========================== 函数声明 ===========================  */

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

/*  =========================== 变量定义 ===========================  */

//进入子模式时间戳
uint32_t enter_mode_11_timestamp;
uint32_t enter_mode_12_timestamp;
uint32_t enter_mode_13_timestamp;
uint32_t enter_mode_21_timestamp;
uint32_t enter_mode_22_timestamp;
uint32_t enter_mode_23_timestamp;
uint32_t enter_mode_31_timestamp;
uint32_t enter_mode_32_timestamp;
uint32_t enter_mode_33_timestamp;

//子模式时间戳
uint32_t mode_11_timestamp;
uint32_t mode_12_timestamp;
uint32_t mode_13_timestamp;
uint32_t mode_21_timestamp;
uint32_t mode_22_timestamp;
uint32_t mode_23_timestamp;
uint32_t mode_31_timestamp;
uint32_t mode_32_timestamp;
uint32_t mode_33_timestamp;

float yaw_target_euler = 0;       //YAW轴目标欧拉角
float pitch_target_euler = 0;     //PITCH轴目标欧拉角
int16_t friction_target_rpm = 0;  //摩擦轮目标转速
int32_t trigger_target_pos = 0;   //拨盘目标位置

bool F_latch;

void GimbalTask(void* argument) {
    for (;;) {

        ORE_Solve();

        SubStateUpdate();

        //状态机
        switch (state_machine.main_state_) {
            case kMainStateNone:
                motor_201.is_enable_ = false;
                motor_202.is_enable_ = false;
                motor_204.is_enable_ = false;
                motor_205.is_enable_ = false;
                motor_206.is_enable_ = false;
                state_machine.HandleEvent(kEventEnterOperate);
                break;

            case kOperate:  //运行模式

                switch (state_machine.sub_state_) {
                    case kSubStateNone:
                        motor_201.is_enable_ = false;
                        motor_202.is_enable_ = false;
                        motor_204.is_enable_ = false;
                        motor_205.is_enable_ = false;
                        motor_206.is_enable_ = false;
                        state_machine.HandleEvent(kEventEnterHalt);
                        break;

                    case kSubMode11:  //切到维护模式
                        state_machine.HandleEvent(kEventEnterHalt);
                        SubMode11Function();
                        break;

                    case kSubMode12:  //遥控器小陀螺顺时针模式
                        SubMode12Function();
                        motor_201.is_enable_ = true;
                        motor_202.is_enable_ = true;
                        motor_204.is_enable_ = true;
                        motor_205.is_enable_ = true;
                        motor_206.is_enable_ = true;
                        break;

                    case kSubMode13:  //遥控器小陀螺逆时针模式
                        SubMode13Function();
                        motor_201.is_enable_ = true;
                        motor_202.is_enable_ = true;
                        motor_204.is_enable_ = true;
                        motor_205.is_enable_ = true;
                        motor_206.is_enable_ = true;
                        break;

                    case kSubMode21:
                    case kSubMode22:
                    case kSubMode23:  //转场模式
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

                    case kSubMode32:  //遥控射击模式
                        SubMode32Function();
                        motor_201.is_enable_ = true;
                        motor_202.is_enable_ = true;
                        motor_204.is_enable_ = true;
                        motor_205.is_enable_ = true;
                        motor_206.is_enable_ = true;
                        break;

                    case kSubMode33:  //键鼠模式
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

            case kHalt:  //维护模式
                motor_201.is_enable_ = false;
                motor_202.is_enable_ = false;
                motor_204.is_enable_ = false;
                motor_205.is_enable_ = false;
                motor_206.is_enable_ = false;
                HaltFunction();

                switch (state_machine.sub_state_) {
                    case kSubMode12:  //遥控器小陀螺顺时针模式
                        state_machine.HandleEvent(kEventEnterOperate);
                        break;

                    case kSubMode13:  //遥控器小陀螺逆时针模式
                        state_machine.HandleEvent(kEventEnterOperate);
                        break;

                    case kSubMode21:
                    case kSubMode22:
                    case kSubMode23:  //转场模式
                        state_machine.HandleEvent(kEventEnterOperate);
                        break;

                    case kSubMode31:
                    case kSubMode32:  //遥控射击模式
                        state_machine.HandleEvent(kEventEnterOperate);
                        break;

                    case kSubMode33:  //键鼠模式
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

        //pitch限值
        pitch_target_euler = clamp(pitch_target_euler, kLowestEuler, kHighestEuler);
        //yaw限值
        yaw_target_euler =
            clamp(yaw_target_euler, (ch110.yaw_integral_ - 180.0f), (ch110.yaw_integral_ + 180.0f));

        TimeStampClear();  //时间戳清除

        osDelay(1);  //延时
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

    //该阶段下YAW的目标值需与实际值一致
    yaw_target_euler = ch110.yaw_integral_;
    /*pitch_target_euler = clamp(ch110.roll_, -5.0f, 30.0f);*/

    //该阶段下拨盘的目标值需与实际值一致
    trigger_target_pos = motor_204.encoder_integral_;
}

void SubMode11Function() {}
void SubMode12Function() {
    vision.is_use_ = false;  //视觉使用标志位置0
    friction_target_rpm = 0;
    RemoteTargetHandle();  //目标值处理
}
void SubMode13Function() {
    vision.is_use_ = false;  //视觉使用标志位置0
    friction_target_rpm = 0;
    RemoteTargetHandle();  //目标值处理
}
void SubMode21Function() {}
void SubMode22Function() {}
void SubMode23Function() {
    vision.is_use_ = false;  //视觉使用标志位置0
    friction_target_rpm = 0;
    RemoteTargetHandle();  //目标值处理
}
void SubMode31Function() {
    RemoteTargetHandle();
    friction_target_rpm = 0;
    vision.is_use_ = true;  //视觉使用标志位置1

    //if (vision.aim_type_ == kArmor) {
    //    yaw_target_euler -= vision.yaw_increament * vision_yaw_coefficient;
    //    pitch_target_euler -= vision.pitch_increment * vision_pitch_coefficient;
    //} else {
    //    yaw_target_euler -= vision.yaw_hub_increment * vision_yaw_coefficient;
    //    pitch_target_euler -= vision.pitch_hub_increment * vision_pitch_coefficient;
    //}
}
void SubMode32Function() {
    vision.is_use_ = true;  //视觉使用标志位置0
    RemoteTargetHandle();   //目标值处理

    //获取系统时间戳，单位为ms
    auto current_timestamp = HAL_GetTick();

    if (enter_mode_32_timestamp == 0) {  //刚进入该模式
        enter_mode_32_timestamp = current_timestamp;
        friction_target_rpm = 0;  //摩擦轮停转
    }
    //1秒后
    else if ((current_timestamp - enter_mode_32_timestamp) >= 1000) {
        friction_target_rpm = kFrictionRpm;  //摩擦轮启动

        if (vision.aim_type_ == kArmor) {
            //每500ms
            if (dr16.remote_.ch3_ > 1684 - kRemoteDeadBand &&
                current_timestamp - mode_32_timestamp >= 200) {
                trigger_target_pos -=
                    49152 * kShotsPerFire;  //假设使用M2006 P36且拨盘每圈6颗弹丸 :8192*36/6=49152
                mode_32_timestamp = current_timestamp;
            }
        } else {
            if (dr16.remote_.ch3_ > 1684 - kRemoteDeadBand && vision.fire_flag == true &&
                vision.fire_latch == false) {
                trigger_target_pos -=
                    49152 * kShotsPerFire;  //假设使用M2006 P36且拨盘每圈6颗弹丸 :8192*36/6=49152
                vision.fire_latch = true;
            } else if (vision.fire_flag == false) {
                vision.fire_latch = false;
            }
        }
    }
}
void SubMode33Function() {

    //获取系统时间戳，单位为ms
    auto current_timestamp = HAL_GetTick();

    if (enter_mode_33_timestamp == 0) {  //刚进入该模式
        enter_mode_33_timestamp = current_timestamp;
        friction_target_rpm = 0;  //摩擦轮停转
    }
    //一秒后
    else if ((current_timestamp - enter_mode_33_timestamp) >= 1000) {

        //G键按下
        if (dr16.KeyBoard_.key_.G_key) {
            friction_target_rpm = kFrictionRpm;  //摩擦轮启动
        }

        //B键按下
        if (dr16.KeyBoard_.key_.B_key) {
            friction_target_rpm = 0;  //摩擦轮关闭
        }

        //F键按下 瞄准模式切换
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

        //鼠标左键按下且摩擦轮开启
        if (dr16.mouse_.press_left_ && friction_target_rpm != 0) {
            //每100ms
            if (vision.aim_type_ == kArmor) {
                if (current_timestamp - mode_33_timestamp >= 50) {
                    //假设使用M2006 P36且拨盘每圈6颗弹丸 :8192*36/6=49152
                    trigger_target_pos -= 49152 * kShotsPerFire;
                    mode_33_timestamp = current_timestamp;
                }
            } else {
                if (vision.fire_flag == true && vision.fire_latch == false) {
                    trigger_target_pos -=
                        49152 *
                        kShotsPerFire;  //假设使用M2006 P36且拨盘每圈6颗弹丸 :8192*36/6=49152
                    vision.fire_latch = true;
                } else if (vision.fire_flag == false) {
                    vision.fire_latch = false;
                }
            }
        }
        //鼠标右键按下且已瞄准目标
        if (dr16.mouse_.press_right_) {
            vision.is_use_ = true;  //视觉使用标志位置1
        }
        //未启动视觉
        else {
            vision.is_use_ = false;  //视觉使用标志位置0
        }

        //鼠标横移
        if (dr16.mouse_.x_axis_) {
            yaw_target_euler -= (float)(dr16.mouse_.x_axis_ * kMouseYawCoefficient);
        }
        //鼠标纵移
        if (dr16.mouse_.y_axis_) {
            pitch_target_euler += (float)(dr16.mouse_.y_axis_ * kMousePitchCoefficient);
        }
    }
}

static void ORE_Solve() {
    //解决串口ORE问题
    if (__HAL_UART_GET_FLAG(kRemoteUart, UART_FLAG_ORE) != RESET) {

        __HAL_UART_CLEAR_OREFLAG(kRemoteUart);  //清除ORE位

        //重新启动接收
        HAL_UARTEx_ReceiveToIdle_DMA(kRemoteUart, remote_rx_buf, kRemoteSize);
    }

    //解决串口ORE问题
    if (__HAL_UART_GET_FLAG(kCommUart, UART_FLAG_ORE) != RESET) {

        __HAL_UART_CLEAR_OREFLAG(kCommUart);  //清除ORE位

        //重新启动接收
        HAL_UARTEx_ReceiveToIdle_DMA(kCommUart, comm_rx_buf, kCommRecvSize);
    }

    //解决串口ORE问题
    if (__HAL_UART_GET_FLAG(kVisionUart, UART_FLAG_ORE) != RESET) {

        __HAL_UART_CLEAR_OREFLAG(kVisionUart);  //清除ORE位

        //重新启动接收
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
 * @brief 卡弹检测
 * 
 */
static void blocking_check() {
    //此次转子位置大于上次转子位置两圈
    if (llabs(trigger_target_pos - motor_204.encoder_integral_) > 16384) {
        //未在退弹过程中，每200ms检测一次是否卡弹
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
    //进行退弹
    if (blocking_flag) {
        //退弹已超过50ms
        if (HAL_GetTick() - blocking_time > 100) {
            blocking_flag = 0;  //清除卡弹标志
        }
        //退弹过程中
        else {
            //实际值跟随目标值 防止恢复瞬间转大角度
            motor_204.encoder_integral_ = trigger_target_pos;
            motor_204.input_ = 5000;  //反向输出
        }
    }
}