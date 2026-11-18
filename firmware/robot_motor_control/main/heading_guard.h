#pragma once
#include <cmath>
#include "robot_config.h"
// Conservative commissioning interlock: prevent wrong-sign / wildly divergent
// IMU feedback from making steering WORSE than encoder-only motion.
// Gyro and encoder are not independent ground truth under slip: fault means
// "do not trust heading feedback", NOT "the IMU is necessarily wrong".
namespace HeadingGuard {
enum Reason { NONE=0, OPPOSITE_RATE=4, EXCESSIVE_YAW_ERROR=5,
              IMPLAUSIBLE_RATE=6 };
class Monitor {
 int sign_count_=0, extreme_count_=0;
 int reason_=NONE;
 float filtered_encoder_=0.0f;
public:
 void reset(){sign_count_=extreme_count_=0;reason_=NONE;filtered_encoder_=0;}
 bool blocked() const{return reason_!=NONE;}
 int reason() const{return reason_;}
 void update(float vcmd,float wcmd,float gyro,float wheel_omega,float error){
   if(blocked())return; // latched: reset only on deliberate disable/re-enable
   if(!std::isfinite(gyro)||!std::isfinite(wheel_omega)||!std::isfinite(error)){
     reason_=IMPLAUSIBLE_RATE;return;
   }
   filtered_encoder_+=0.2f*(wheel_omega-filtered_encoder_);
   // Only compare signs in nominally straight forward/backward driving.
   if(std::fabs(vcmd)>0.05f && std::fabs(wcmd)<0.001f){
     const float minrate=RobotConfig::HEADING_GUARD_MIN_RATE_RADPS;
     const bool opposite=(gyro*filtered_encoder_<0.0f) &&
          std::fabs(gyro)>minrate && std::fabs(filtered_encoder_)>minrate;
     sign_count_=opposite?sign_count_+1:0;
     // Abrupt physically inconsistent spin while wheel odometry nearly still.
     const bool extreme=std::fabs(gyro)>0.9f && std::fabs(filtered_encoder_)<0.25f;
     extreme_count_=extreme?extreme_count_+1:0;
     if(sign_count_>=RobotConfig::HEADING_GUARD_SIGN_SAMPLES)
       reason_=OPPOSITE_RATE;
     else if(extreme_count_>=RobotConfig::HEADING_GUARD_EXTREME_SAMPLES)
       reason_=IMPLAUSIBLE_RATE;
     else if(std::fabs(error)>RobotConfig::HEADING_GUARD_MAX_ERROR_RAD)
       reason_=EXCESSIVE_YAW_ERROR;
   }else {sign_count_=0;extreme_count_=0;}
 }
};
} // namespace HeadingGuard
