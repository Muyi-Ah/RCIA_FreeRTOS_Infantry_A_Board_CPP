#include "error_handle.hpp"
#include "variables.hpp"
#include "cmsis_armclang.h"

void ErrorHandle(enum ErrorType error_type) {

    switch (error_type) {
        case kPointerError:  //ָ�����
            __disable_irq();
            while (1) {
                /* code */
            }
            break;

        case kHalLibError:  //HAL�����
            __disable_irq();
            while (1) {
                /* code */
            }
            break;

        case kSwitchError:  //Switch��֧����
            __disable_irq();
            while (1) {
                /* code */
            }
            break;

        case kMotorError:  //�������
            //@develop �����д
            break;

        case kDr16Error:  //DR16���մ���
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

        case kImuError:  //IMU����
            //@develop ���ƿ�����
            break;

        case kComuError:  //���ͨ�Ŵ���
                          //@develop ���д
            break;

        case kVisionError:               //�Ӿ�ͨ�Ŵ���
            //vision.set_use(false);       //���ʹ�ñ�־λ
            //vision.set_is_aimed(false);  //��ս��ճɹ���־λ
            break;

        default:
            __disable_irq();
            while (1) {
                /* code */
            }
            break;
    }
}
