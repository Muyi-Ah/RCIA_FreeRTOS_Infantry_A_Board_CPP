#pragma once
#include <cstdint>

constexpr float kLowestEuler = -20.0f;
constexpr float kHighestEuler = 11.0f;

extern float yaw_target_euler;
extern float pitch_target_euler;
extern int16_t friction_target_rpm;
extern int32_t trigger_target_pos;

#ifdef __cplusplus
extern "C" {
#endif

void GimbalTask(void* argument);

#ifdef __cplusplus
}
#endif
