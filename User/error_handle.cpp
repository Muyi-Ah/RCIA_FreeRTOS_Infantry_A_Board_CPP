#include "error_handle.hpp"
#include "variables.hpp"
#include "cmsis_armclang.h"

void ErrorHandle(enum ErrorType error_type) {

    switch (error_type) {
        case kPointerError:  //指针错误
            __disable_irq();
            while (1) {
                /* code */
            }
            break;

        case kHalLibError:  //HAL库错误
            __disable_irq();
            while (1) {
                /* code */
            }
            break;

        case kSwitchError:  //Switch分支错误
            __disable_irq();
            while (1) {
                /* code */
            }
            break;

        case kMotorError:  //电机错误
            //@develop 晚点再写
            break;

        case kDr16Error:  //DR16接收错误
            dr16.KeyBoard_.key_code_ = 0;
            dr16.remote_.ch0_ = 1024;
            dr16.remote_.ch1_ = 1024;
            dr16.remote_.ch2_ = 1024;
            dr16.remote_.ch3_ = 1024;
            dr16.remote_.s1_ = 3;
            dr16.remote_.s2_ = 1;
            dr16.mouse_.x_axis_ = 0;
            dr16.mouse_.y_axis_ = 0;
            dr16.mouse_.press_left_ = 0;
            dr16.mouse_.press_right_ = 0;
            break;

        case kImuError:  //IMU错误
            //@develop 好似开香槟
            break;

        case kComuError:  //板间通信错误
                          //@develop 晚点写
            break;

        case kVisionError:               //视觉通信错误
            //vision.set_use(false);       //清空使用标志位
            //vision.set_is_aimed(false);  //清空接收成功标志位
            break;

        default:
            __disable_irq();
            while (1) {
                /* code */
            }
            break;
    }
}
