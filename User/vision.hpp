#pragma once
#include <cstdint>

enum AimType {
    kArmor,
    kRobotHub,
};

class Vision {
   public:
    bool is_use_ = false;
    bool is_aimed_ = false;
    bool is_reply = false;
    enum AimType aim_type_;

    uint16_t origin_yaw_ = 0;
    uint16_t origin_pitch_ = 0;
    uint16_t origin_yaw_hub_ = 0;
    uint16_t origin_pitch_hub = 0;

    float yaw_increment_temp = 0;
    float pitch_increment_temp = 0;
    float yaw_hub_increment_temp = 0;
    float pitch_hub_increment_temp = 0;

    float yaw_increament = 0;
    float pitch_increment = 0;
    float yaw_hub_increment = 0;
    float pitch_hub_increment = 0;

    uint16_t fire_flag_temp = 0;
    bool fire_flag = false;
    bool fire_latch = false;

    uint32_t recv_time = 0;

    void RecvUpdate(const uint8_t* buf);
    void Send();
   private:
    bool AbsoluteFilte() const;
};
