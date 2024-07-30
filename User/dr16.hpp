#pragma once
#include <cstdint>

class DR16 {
   public:

    struct {
        uint16_t ch0_ = 1024;
        uint16_t ch1_ = 1024;
        uint16_t ch2_ = 1024;
        uint16_t ch3_ = 1024;
        uint16_t wheel_ = 1024;
        uint8_t s1_;
        uint8_t s2_;
    } remote_;

    struct {
        int16_t x_axis_;
        int16_t y_axis_;
        int16_t z_axis_;
        uint8_t press_left_;
        uint8_t press_right_;
    } mouse_;

    union {
        uint16_t key_code_;
        struct {
            uint16_t W_key : 1;
            uint16_t S_key : 1;
            uint16_t A_key : 1;
            uint16_t D_key : 1;
            uint16_t SHIFT_key : 1;
            uint16_t CTRL_key : 1;
            uint16_t Q_key : 1;
            uint16_t E_key : 1;
            uint16_t R_key : 1;
            uint16_t F_key : 1;
            uint16_t G_key : 1;
            uint16_t Z_key : 1;
            uint16_t X_key : 1;
            uint16_t C_key : 1;
            uint16_t V_key : 1;
            uint16_t B_key : 1;
        } key_;
    } KeyBoard_;

    bool is_reply_;

    void DataUpdate(const uint8_t* buf);
};