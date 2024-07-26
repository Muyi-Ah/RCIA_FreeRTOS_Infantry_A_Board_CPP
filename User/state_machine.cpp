#include "state_machine.hpp"
#include "error_handle.hpp"

void StateMachine::HandleEvent(enum Event event) {
    switch (main_state_) {

        case kMainStateNone:  //³õÊ¼×´Ì¬

            switch (event) {
                case kEventEnterOperate:  //ÇÐ»»µ½ÔËÐÐ
                    main_state_ = kOperate;
                    break;
								
                case kEventEnterHalt:  //ÇÐ»»µ½Í£Ö¹
                    main_state_ = kHalt;
								
								case kEventSwitchSubMode11:  //ÇÐ»»µ½×Ó×´Ì¬11
                    sub_state_ = kSubMode11;
                    break;

                case kEventSwitchSubMode12:  //ÇÐ»»µ½×Ó×´Ì¬12
                    sub_state_ = kSubMode12;
                    break;

                case kEventSwitchSubMode13:  //ÇÐ»»µ½×Ó×´Ì¬13
                    sub_state_ = kSubMode13;
                    break;

                case kEventSwitchSubMode21:  //ÇÐ»»µ½×Ó×´Ì¬21
                    sub_state_ = kSubMode21;
                    break;

                case kEventSwitchSubMode22:  //ÇÐ»»µ½×Ó×´Ì¬22
                    sub_state_ = kSubMode22;
                    break;

                case kEventSwitchSubMode23:  //ÇÐ»»µ½×Ó×´Ì¬23
                    sub_state_ = kSubMode23;
                    break;

                case kEventSwitchSubMode31:  //ÇÐ»»µ½×Ó×´Ì¬31
                    sub_state_ = kSubMode31;
                    break;

                case kEventSwitchSubMode32:  //ÇÐ»»µ½×Ó×´Ì¬32
                    sub_state_ = kSubMode32;
                    break;

                case kEventSwitchSubMode33:  //ÇÐ»»µ½×Ó×´Ì¬33
                    sub_state_ = kSubMode33;
                    break;

                default:
                    /*ErrorHandle(kSwitchError);*/
                    break;
            }

            break;

        case kHalt:  //Í£ÖÍ×´Ì¬

            switch (event) {
                case kEventEnterOperate:  //ÇÐ»»µ½ÔËÐÐ
                    main_state_ = kOperate;
                    break;

                case kEventEnterHalt:  //ÇÐ»»µ½Í£Ö¹
                    main_state_ = kHalt;
                    break;
								
								case kEventSwitchSubMode11:  //ÇÐ»»µ½×Ó×´Ì¬11
                    sub_state_ = kSubMode11;
                    break;

                case kEventSwitchSubMode12:  //ÇÐ»»µ½×Ó×´Ì¬12
                    sub_state_ = kSubMode12;
                    break;

                case kEventSwitchSubMode13:  //ÇÐ»»µ½×Ó×´Ì¬13
                    sub_state_ = kSubMode13;
                    break;

                case kEventSwitchSubMode21:  //ÇÐ»»µ½×Ó×´Ì¬21
                    sub_state_ = kSubMode21;
                    break;

                case kEventSwitchSubMode22:  //ÇÐ»»µ½×Ó×´Ì¬22
                    sub_state_ = kSubMode22;
                    break;

                case kEventSwitchSubMode23:  //ÇÐ»»µ½×Ó×´Ì¬23
                    sub_state_ = kSubMode23;
                    break;

                case kEventSwitchSubMode31:  //ÇÐ»»µ½×Ó×´Ì¬31
                    sub_state_ = kSubMode31;
                    break;

                case kEventSwitchSubMode32:  //ÇÐ»»µ½×Ó×´Ì¬32
                    sub_state_ = kSubMode32;
                    break;

                case kEventSwitchSubMode33:  //ÇÐ»»µ½×Ó×´Ì¬33
                    sub_state_ = kSubMode33;
                    break;

                default:
                    /*ErrorHandle(kSwitchError);*/
                    break;
            }

            break;

        case kOperate:  //ÔËÐÐ×´Ì¬

            switch (event) {
                case kEventEnterOperate:  //ÇÐ»»µ½ÔËÐÐ
                    main_state_ = kOperate;
                    break;

                case kEventEnterHalt:  //ÇÐ»»µ½Í£Ö¹
                    main_state_ = kHalt;
                    sub_state_ = kSubStateNone;  //×Ó×´Ì¬Ò²±»ÖØÖÃ
                    break;

                case kEventSwitchSubMode11:  //ÇÐ»»µ½×Ó×´Ì¬11
                    sub_state_ = kSubMode11;
                    break;

                case kEventSwitchSubMode12:  //ÇÐ»»µ½×Ó×´Ì¬12
                    sub_state_ = kSubMode12;
                    break;

                case kEventSwitchSubMode13:  //ÇÐ»»µ½×Ó×´Ì¬13
                    sub_state_ = kSubMode13;
                    break;

                case kEventSwitchSubMode21:  //ÇÐ»»µ½×Ó×´Ì¬21
                    sub_state_ = kSubMode21;
                    break;

                case kEventSwitchSubMode22:  //ÇÐ»»µ½×Ó×´Ì¬22
                    sub_state_ = kSubMode22;
                    break;

                case kEventSwitchSubMode23:  //ÇÐ»»µ½×Ó×´Ì¬23
                    sub_state_ = kSubMode23;
                    break;

                case kEventSwitchSubMode31:  //ÇÐ»»µ½×Ó×´Ì¬31
                    sub_state_ = kSubMode31;
                    break;

                case kEventSwitchSubMode32:  //ÇÐ»»µ½×Ó×´Ì¬32
                    sub_state_ = kSubMode32;
                    break;

                case kEventSwitchSubMode33:  //ÇÐ»»µ½×Ó×´Ì¬33
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
