#pragma once
#include <math.h>
#include <stdint.h>
// Platform-independent control helpers used by the ESP32 controller AND unit tests.
namespace ControllerMath {
// Two concurrent tasks may timestamp the callback just after control-task time
// snapshot. A small future timestamp means the command is NEW, not stale.
inline uint64_t command_age_us(uint64_t now,uint64_t received){
    return now>=received?now-received:0ULL;
}

inline float clamp(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
inline float signed_speed_error(float desired_rpm,float measured_rpm){
    if(desired_rpm==0)return 0;
    const float direction=desired_rpm>0?1.0f:-1.0f;
    return fabsf(desired_rpm)-direction*measured_rpm;
}
// Returns correction to the magnitude of each wheel's requested speed.
// lead_meters>0 means LEFT wheel has travelled farther in commanded direction.
inline float straight_trim(float left_distance_m,float right_distance_m,
        int direction,float rpm_per_meter,float trim_cap_rpm){
    if(direction==0)return 0;
    const float lead=direction*(left_distance_m-right_distance_m);
    return clamp(lead*rpm_per_meter,-trim_cap_rpm,trim_cap_rpm);
}
inline void differential_rpms(float v,float wz,float track_m,
        float left_radius_m,float right_radius_m,float &l,float &r){
    constexpr float k=60.0f/(2.0f*3.14159265358979323846f);
    l=(v-wz*track_m*0.5f)*k/left_radius_m;
    r=(v+wz*track_m*0.5f)*k/right_radius_m;
}
}
