#pragma once
// V10: FF calibration data and input validation; no ESP/ROS dependency.
#include <cmath>
#include <cstdint>
#include <cstring>
#include "robot_config.h"
namespace FeedforwardV10 {
constexpr int COUNT = RobotConfig::FF_COUNT;
constexpr int MAPS = 4; // 0=FWD_L, 1=FWD_R, 2=REV_L, 3=REV_R
constexpr float SCALE_MIN=0.50f, SCALE_MAX=1.50f;
struct Calibration { float pwm[MAPS][COUNT] = {}; float scale[MAPS] = {}; };
inline Calibration defaults(){
  Calibration a{};
  const float* tables[MAPS]={RobotConfig::FF_LEFT,RobotConfig::FF_RIGHT,
                             RobotConfig::FF_REVERSE_LEFT,RobotConfig::FF_REVERSE_RIGHT};
  const float s[MAPS]={RobotConfig::FF_SCALE_FWD_L,RobotConfig::FF_SCALE_FWD_R,
                       RobotConfig::FF_SCALE_REV_L,RobotConfig::FF_SCALE_REV_R};
  for(int i=0;i<MAPS;++i){std::memcpy(a.pwm[i],tables[i],COUNT*sizeof(float));a.scale[i]=s[i];}
  return a;
}
inline int mapIndex(int wheel,int direction){
  if((wheel!=0&&wheel!=1)||(direction!=1&&direction!=-1))return -1;
  return (direction==1?0:2)+wheel;
}
inline bool valid(const Calibration& v){
  for(int m=0;m<MAPS;++m){
    if(!std::isfinite(v.scale[m])||v.scale[m]<SCALE_MIN||v.scale[m]>SCALE_MAX)return false;
    for(int i=0;i<COUNT;++i){
      if(!std::isfinite(v.pwm[m][i])||v.pwm[m][i]<0.0f||v.pwm[m][i]>RobotConfig::PWM_MAX)return false;
      if(i>0 && v.pwm[m][i]<v.pwm[m][i-1])return false;
    }
  }
  return true;
}
inline bool updatePoint(Calibration& v,int wheel,int direction,float rpm,float pwm){
  const int m=mapIndex(wheel,direction);
  if(m<0||!std::isfinite(rpm)||!std::isfinite(pwm))return false;
  int k=-1;
  for(int i=0;i<COUNT;++i)if(std::fabs(rpm-RobotConfig::FF_RPM[i])<0.01f){k=i;break;}
  if(k<0)return false;
  auto candidate=v;
  candidate.pwm[m][k]=pwm;
  if(!valid(candidate))return false;
  v=candidate;return true;
}
inline bool updateScale(Calibration& v,int wheel,int direction,float scale){
  const int m=mapIndex(wheel,direction);
  if(m<0||!std::isfinite(scale))return false;
  auto candidate=v;candidate.scale[m]=scale;
  if(!valid(candidate))return false;
  v=candidate;return true;
}
inline float duty(const Calibration& v,float rpm,bool left){
  const float magnitude=std::fabs(rpm);
  if(magnitude<RobotConfig::STOP_RPM)return 0.0f;
  const int m=mapIndex(left?0:1,rpm<0?-1:1);
  float base=0.0f;
  if(magnitude<RobotConfig::FF_RPM[0])base=v.pwm[m][0]*(magnitude/RobotConfig::FF_RPM[0]);
  else if(magnitude>=RobotConfig::FF_RPM[COUNT-1])base=v.pwm[m][COUNT-1];
  else for(int i=0;i<COUNT-1;++i)if(magnitude<=RobotConfig::FF_RPM[i+1]){
    const float t=(magnitude-RobotConfig::FF_RPM[i])/(RobotConfig::FF_RPM[i+1]-RobotConfig::FF_RPM[i]);
    base=v.pwm[m][i]+t*(v.pwm[m][i+1]-v.pwm[m][i]);break;
  }
  const float result=base*v.scale[m];
  return result<0?0:(result>RobotConfig::PWM_MAX?RobotConfig::PWM_MAX:result);
}
constexpr uint32_t MAGIC=0x46463130u; // FF10
constexpr uint32_t VERSION=10;
struct Persist {
  uint32_t magic=0,version=0;
  Calibration calibration{};
  uint32_t checksum=0;
};
inline uint32_t checksum(const Persist& p){
  // Only deterministic fixed-size integers/floats from zero-initialized Persist.
  uint32_t h=2166136261u;
  const auto* b=reinterpret_cast<const uint8_t*>(&p);
  for(size_t i=0;i<sizeof(Persist)-sizeof(uint32_t);++i){h^=b[i];h*=16777619u;}
  return h;
}
inline Persist pack(const Calibration& a){
  Persist p{};p.magic=MAGIC;p.version=VERSION;p.calibration=a;p.checksum=checksum(p);return p;
}
inline bool unpack(const Persist& p,Calibration& a){
  if(p.magic!=MAGIC||p.version!=VERSION||p.checksum!=checksum(p)||!valid(p.calibration))return false;
  a=p.calibration;return true;
}
} // namespace FeedforwardV10
