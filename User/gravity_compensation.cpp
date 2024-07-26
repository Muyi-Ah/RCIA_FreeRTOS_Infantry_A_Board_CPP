#include "gravity_compensation.hpp"
#include "variables.hpp"
#include "simple_math.hpp"
#include "arm_math.h"

float EmpiricalGravityCompensator::Compute(float theta) const {
    float theta_degree = theta;
    float theta_radian = deg2rad(theta_degree);
    //计算力矩但用经验系数C 调试时自己试出合适的C的大小
    float tau_c = C_ * arm_cos_f32(theta_radian);
    return tau_c;
}
