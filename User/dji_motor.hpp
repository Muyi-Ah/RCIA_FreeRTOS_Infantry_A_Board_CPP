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

    uint32_t recv_id_ = 0;                 //����ID
    volatile uint16_t encoder_value_ = 0;  //������ֵ
    uint16_t encoder_value_prev_ = 0;      //��һ�εı�����ֵ
    int64_t encoder_integral_ = 0;         //������ֵ����
    volatile int16_t actual_rpm_ = 0;      //ʵ��ת��ֵ
    volatile int16_t actual_current_ = 0;  //ʵ�ʵ���ֵ
    volatile uint8_t temperatrue_ = 0;     //�¶�ֵ
    int16_t input_ = 0;                    //�������ֵ
    bool is_enable_ = false;               //���ʹ�ܱ�־λ
    bool is_reply_ = false;                //���Ӧ���־λ
    bool is_first_recv_done_ = false;      //��һ�ν������ݱ�־λ

    void DataUpdate(volatile const uint8_t* buf);
    int32_t AbsoluteErrorCompute(uint16_t target, enum DirectionType direction) const;
    void clear_encoder_integral() { encoder_integral_ = 0; };

   private:
    int16_t IntegralErrorCompute();
};

void DjiMotorSend();
float CalculateTheta(uint16_t encoder_value, uint16_t initial_value);
