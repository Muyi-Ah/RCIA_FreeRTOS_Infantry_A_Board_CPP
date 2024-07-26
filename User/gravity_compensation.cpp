#include "gravity_compensation.hpp"
#include "variables.hpp"
#include "simple_math.hpp"
#include "arm_math.h"

float EmpiricalGravityCompensator::Compute(float theta) const {
    float theta_degree = theta;
    float theta_radian = deg2rad(theta_degree);
    //�������ص��þ���ϵ��C ����ʱ�Լ��Գ����ʵ�C�Ĵ�С
    float tau_c = C_ * arm_cos_f32(theta_radian);
    return tau_c;
}
