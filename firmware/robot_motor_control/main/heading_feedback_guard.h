#pragma once
// Diagnostic sanity guard for the outer yaw-loop, never modifies wheel GPIO.
// Compare two independent physical estimates only during deliberate spin tests;
// wheel-ground slip can otherwise make encoder/gyro rate temporarily disagree.
#include <math.h>
#include "robot_config.h"
namespace HeadingGuard {
class Monitor {
  int reason_=0,opposite_ticks_=0,extreme_ticks_=0;
public:
  void reset(){reason_=0;opposite_ticks_=0;extreme_ticks_=0;}
  bool blocked() const{return reason_!=0;}
  int reason() const{return reason_;} // 4=mismatched spin signs, 5=large yaw error
  void update(float v,float w,float gyro,float encoder_w,float heading_error){
    if(blocked())return;
    if(!isfinite(gyro)||!isfinite(encoder_w)||!isfinite(heading_error)){
      reason_=6;return;
    }
    if(fabsf(heading_error)>RobotConfig::HEADING_GUARD_MAX_ERROR_RAD &&
       (fabsf(v)>0.015f||fabsf(w)>0.10f)){
      if(++extreme_ticks_>=RobotConfig::HEADING_GUARD_EXTREME_SAMPLES){reason_=5;return;}
    }else extreme_ticks_=0;
    // Only check measured sign during deliberate spin (not straight-driving).
    const bool deliberate_spin=fabsf(w)>0.18f && fabsf(v)<0.018f;
    const bool measurable=fabsf(gyro)>RobotConfig::HEADING_GUARD_MIN_RATE_RADPS&&
                          fabsf(encoder_w)>RobotConfig::HEADING_GUARD_MIN_RATE_RADPS;
    if(deliberate_spin&&measurable&&gyro*encoder_w<0){
      if(++opposite_ticks_>=RobotConfig::HEADING_GUARD_SIGN_SAMPLES)reason_=4;
    }else opposite_ticks_=0;
  }
};
} // namespace HeadingGuard
