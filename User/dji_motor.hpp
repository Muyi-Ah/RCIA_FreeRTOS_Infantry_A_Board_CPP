#pragma once
#include <cstdint>

enum DirectionType {
    kBoth,
    kCW,
    kCCW,
};

class DjiMotor {
   public:
    DjiMotor(uint32_t recv_id) { recv_id_ = recv_id; };

    uint32_t recv_id_ = 0;                 //接收ID
    volatile uint16_t encoder_value_ = 0;  //编码器值
    uint16_t encoder_value_prev_ = 0;      //上一次的编码器值
    int64_t encoder_integral_ = 0;         //编码器值积分
    volatile int16_t actual_rpm_ = 0;      //实际转速值
    volatile int16_t actual_current_ = 0;  //实际电流值
    volatile uint8_t temperatrue_ = 0;     //温度值
    int16_t input_ = 0;                    //电机输入值
    bool is_enable_ = false;               //电机使能标志位
    bool is_reply_ = false;                //电机应答标志位
    bool is_first_recv_done_ = false;      //第一次接收数据标志位

    void DataUpdate(volatile const uint8_t* buf);
    int32_t AbsoluteErrorCompute(uint16_t target, enum DirectionType direction) const;
    void clear_encoder_integral() { encoder_integral_ = 0; };

   private:
    int16_t IntegralErrorCompute();
};

void DjiMotorSend();
float CalculateTheta(uint16_t encoder_value, uint16_t initial_value);
