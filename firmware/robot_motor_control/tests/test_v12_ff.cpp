#include "../main/ff_calibration_v10.h"
#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdio>
int main(){
  using namespace FeedforwardV10;
  Calibration a=defaults();assert(valid(a));
  assert(std::fabs(duty(a,50.0f,true)-RobotConfig::FF_LEFT[3]*RobotConfig::FF_SCALE_FWD_L)<0.01f);
  assert(std::fabs(duty(a,50.0f,false)-RobotConfig::FF_RIGHT[3]*RobotConfig::FF_SCALE_FWD_R)<0.01f);
  assert(std::fabs(duty(a,40.0f,true)-RobotConfig::FF_LEFT[2]*RobotConfig::FF_SCALE_FWD_L)<0.01f);
  assert(std::fabs(duty(a,50.0f,false)-duty(a,-50.0f,false))>0.01f);
  assert(!updatePoint(a,2,1,50,130));assert(!updatePoint(a,0,0,50,130));
  assert(!updatePoint(a,0,1,55,130));assert(!updatePoint(a,0,1,50,NAN));
  assert(!updatePoint(a,0,1,50,240)); // destroys monotonicity with 60 RPM
  assert(updatePoint(a,0,1,50,130));assert(std::fabs(duty(a,50,true)-136.5f)<0.01f);
  assert(!updateScale(a,0,1,1.6f));assert(updateScale(a,0,1,1.15f));
  assert(std::fabs(duty(a,50,true)-149.5f)<0.01f);
  assert(updatePoint(a,1,-1,50,110));assert(updateScale(a,1,-1,1.02f));
  auto rec=pack(a);Calibration restored=defaults();assert(unpack(rec,restored));
  assert(std::fabs(duty(restored,50,true)-149.5f)<0.01f);
  rec.calibration.pwm[0][3]=12;assert(!unpack(rec,restored));
  rec=pack(a);rec.magic=0;assert(!unpack(rec,restored));
  printf("V10 FF point/scale/sign, input guards, monotonic interpolation, NVS integrity PASS\n");
}
