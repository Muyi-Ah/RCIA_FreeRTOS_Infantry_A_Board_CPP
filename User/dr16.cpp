#include "dr16.hpp"

void DR16::DataUpdate(const uint8_t* buf) {
    remote_.ch0_ = (buf[0] | buf[1] << 8) & 0x07FF;
    remote_.ch1_ = (buf[1] >> 3 | buf[2] << 5) & 0x07FF;
    remote_.ch2_ = (buf[2] >> 6 | buf[3] << 2 | buf[4] << 10) & 0x07FF;
    remote_.ch3_ = (buf[4] >> 1 | buf[5] << 7) & 0x07FF;

    remote_.s1_ = ((buf[5] >> 4) & 0x000C) >> 2;
    remote_.s2_ = ((buf[5] >> 4) & 0x0003);

    mouse_.x_axis_ = (buf[6] | buf[7] << 8);
    mouse_.y_axis_ = (buf[8] | buf[9] << 8);
    mouse_.z_axis_ = (buf[10] | buf[11] << 8);

    mouse_.press_left_ = buf[12];
    mouse_.press_right_ = buf[13];

    KeyBoard_.key_code_ = (buf[14] | buf[15] << 8);

    remote_.wheel_ = (buf[16] | buf[17] << 8);
}
