#pragma once
#include <cmath>
#include <cstdint>
namespace ImuCalibrationV10 {
constexpr int SAMPLES=150;
constexpr float ACCEL_MIN=8.0f, ACCEL_MAX=11.4f;
constexpr float GYRO_ABS_MAX_COUNTS=1300.0f; // 19.85 deg/s at +/-500 dps (65.5 LSB/deg/s)
constexpr float GYRO_Z_BOOT_MAX_COUNTS=150.0f; // tighter yaw-axis motion guard (~2.29 deg/s @65.5 counts/deg/s)
constexpr float GYRO_STD_MAX_COUNTS=25.0f;
constexpr float ACCEL_STD_MAX=0.15f;
// Reject reasons: 1=accel invalid, 2=gyro gross movement, 4=unstable window, 8=sudden gyro step
struct Window {
  int count=0;
  float sum[3]={},sumsq[3]={},previous[3]={};
  float accel_sum=0,accel_sumsq=0;
  uint8_t reason=0;
  float bias[3]={};
  bool done=false;
  void reset(){count=0;done=false;for(int i=0;i<3;++i){sum[i]=sumsq[i]=previous[i]=0;} accel_sum=accel_sumsq=0;}
  bool push(const float gyro[3],float accel){
    if(done)return true;
    reason=0;
    if(!std::isfinite(accel)||accel<ACCEL_MIN||accel>ACCEL_MAX)reason|=1;
    for(int i=0;i<3;++i){
      if(!std::isfinite(gyro[i])||std::fabs(gyro[i])>GYRO_ABS_MAX_COUNTS)reason|=2;
      if(count && std::isfinite(gyro[i]) && std::fabs(gyro[i]-previous[i])>100.0f)reason|=8;
    }
    // Accept X/Y factory offsets such as -250 counts, but remain conservative
    // about yaw spin during zero-rate Z bias calibration.
    if(std::isfinite(gyro[2])&&std::fabs(gyro[2])>GYRO_Z_BOOT_MAX_COUNTS)reason|=2;
    if(reason){reset();return false;}
    for(int i=0;i<3;++i){sum[i]+=gyro[i];sumsq[i]+=gyro[i]*gyro[i];previous[i]=gyro[i];}
    accel_sum+=accel;accel_sumsq+=accel*accel;
    ++count;
    if(count<SAMPLES)return false;
    for(int i=0;i<3;++i){
      float mean=sum[i]/SAMPLES;
      float var=fmaxf(0.0f,sumsq[i]/SAMPLES-mean*mean);
      if(!std::isfinite(var)||var>GYRO_STD_MAX_COUNTS*GYRO_STD_MAX_COUNTS)reason|=4;
      bias[i]=mean;
    }
    float a_mean=accel_sum/SAMPLES;
    float a_var=fmaxf(0.0f,accel_sumsq/SAMPLES-a_mean*a_mean);
    if(!std::isfinite(a_var)||a_var>ACCEL_STD_MAX*ACCEL_STD_MAX)reason|=4;
    // Z bias beyond ~7.6 deg/s is more likely a rotating robot than static bias.
    if(std::fabs(bias[2])>500.0f)reason|=2;
    if(reason){reset();return false;}
    done=true;return true;
  }
};
} // namespace ImuCalibrationV10
