#include "state_machine.hpp"
#include "error_handle.hpp"

void StateMachine::HandleEvent(enum Event event) {
    switch (main_state_) {

        case kMainStateNone:  //��ʼ״̬

            switch (event) {
                case kEventEnterOperate:  //�л�������
                    main_state_ = kOperate;
                    break;
								
                case kEventEnterHalt:  //�л���ֹͣ
                    main_state_ = kHalt;
								
								case kEventSwitchSubMode11:  //�л�����״̬11
                    sub_state_ = kSubMode11;
                    break;

                case kEventSwitchSubMode12:  //�л�����״̬12
                    sub_state_ = kSubMode12;
                    break;

                case kEventSwitchSubMode13:  //�л�����״̬13
                    sub_state_ = kSubMode13;
                    break;

                case kEventSwitchSubMode21:  //�л�����״̬21
                    sub_state_ = kSubMode21;
                    break;

                case kEventSwitchSubMode22:  //�л�����״̬22
                    sub_state_ = kSubMode22;
                    break;

                case kEventSwitchSubMode23:  //�л�����״̬23
                    sub_state_ = kSubMode23;
                    break;

                case kEventSwitchSubMode31:  //�л�����״̬31
                    sub_state_ = kSubMode31;
                    break;

                case kEventSwitchSubMode32:  //�л�����״̬32
                    sub_state_ = kSubMode32;
                    break;

                case kEventSwitchSubMode33:  //�л�����״̬33
                    sub_state_ = kSubMode33;
                    break;

                default:
                    /*ErrorHandle(kSwitchError);*/
                    break;
            }

            break;

        case kHalt:  //ͣ��״̬

            switch (event) {
                case kEventEnterOperate:  //�л�������
                    main_state_ = kOperate;
                    break;

                case kEventEnterHalt:  //�л���ֹͣ
                    main_state_ = kHalt;
                    break;
								
								case kEventSwitchSubMode11:  //�л�����״̬11
                    sub_state_ = kSubMode11;
                    break;

                case kEventSwitchSubMode12:  //�л�����״̬12
                    sub_state_ = kSubMode12;
                    break;

                case kEventSwitchSubMode13:  //�л�����״̬13
                    sub_state_ = kSubMode13;
                    break;

                case kEventSwitchSubMode21:  //�л�����״̬21
                    sub_state_ = kSubMode21;
                    break;

                case kEventSwitchSubMode22:  //�л�����״̬22
                    sub_state_ = kSubMode22;
                    break;

                case kEventSwitchSubMode23:  //�л�����״̬23
                    sub_state_ = kSubMode23;
                    break;

                case kEventSwitchSubMode31:  //�л�����״̬31
                    sub_state_ = kSubMode31;
                    break;

                case kEventSwitchSubMode32:  //�л�����״̬32
                    sub_state_ = kSubMode32;
                    break;

                case kEventSwitchSubMode33:  //�л�����״̬33
                    sub_state_ = kSubMode33;
                    break;

                default:
                    /*ErrorHandle(kSwitchError);*/
                    break;
            }

            break;

        case kOperate:  //����״̬

            switch (event) {
                case kEventEnterOperate:  //�л�������
                    main_state_ = kOperate;
                    break;

                case kEventEnterHalt:  //�л���ֹͣ
                    main_state_ = kHalt;
                    sub_state_ = kSubStateNone;  //��״̬Ҳ������
                    break;

                case kEventSwitchSubMode11:  //�л�����״̬11
                    sub_state_ = kSubMode11;
                    break;

                case kEventSwitchSubMode12:  //�л�����״̬12
                    sub_state_ = kSubMode12;
                    break;

                case kEventSwitchSubMode13:  //�л�����״̬13
                    sub_state_ = kSubMode13;
                    break;

                case kEventSwitchSubMode21:  //�л�����״̬21
                    sub_state_ = kSubMode21;
                    break;

                case kEventSwitchSubMode22:  //�л�����״̬22
                    sub_state_ = kSubMode22;
                    break;

                case kEventSwitchSubMode23:  //�л�����״̬23
                    sub_state_ = kSubMode23;
                    break;

                case kEventSwitchSubMode31:  //�л�����״̬31
                    sub_state_ = kSubMode31;
                    break;

                case kEventSwitchSubMode32:  //�л�����״̬32
                    sub_state_ = kSubMode32;
                    break;

                case kEventSwitchSubMode33:  //�л�����״̬33
                    sub_state_ = kSubMode33;
                    break;

                default:
                    /*ErrorHandle(kSwitchError);*/
                    break;
            }

            break;

        default:
            /*ErrorHandle(kSwitchError);*/
            break;
    }
}
