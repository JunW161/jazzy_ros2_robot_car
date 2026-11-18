#include "../main/yaw_outer.h"
#include <cassert>
#include <cmath>
#include <iostream>
int main(){
  using namespace YawOuter;
  Controller c;
  assert(c.observe(0.0f,1000000,1,true));
  assert(c.observe(1.0f,1020000,2,true));
  const float first=c.yawRadians();
  assert(std::fabs(first-0.01f)<0.0001f);
  // Reusing one sample MUST NOT integrate it twice.
  for(int i=0;i<10;i++)assert(c.observe(1.0f,1020000,2,true));
  assert(std::fabs(c.yawRadians()-first)<1e-6f);
  assert(c.observe(1.0f,1120000,3,true));
  assert(c.observe(0.0f,1140000,4,true));
  const float after_turn=c.yawRadians();
  assert(after_turn>0.11f && after_turn<0.13f);
  c.inactive();
  assert(c.observe(0.0f,1240000,5,true));
  assert(std::fabs(c.yawRadians()-after_turn)<1e-6f);
  // After full stop yaw remains at the measured angle, NOT auto-zero.
  auto s=c.update(0.10f,0.0f,0.02f,true);
  assert(std::fabs(s.heading_err)<1e-6f); // new straight leg latches current yaw
  assert(s.mode==1);
  assert(c.observe(0.5f,1260000,6,true));
  auto e=c.update(0.10f,0.0f,0.02f,true);
  assert(e.correction == 0.0f); // under 0.5 degree, NEW requested deadband
  assert(c.observe(0.5f,1280000,7,true));
  e=c.update(0.10f,0.0f,0.02f,true);
  assert(e.correction < 0.0f); // deviation >= 0.5deg: turn right
  assert(e.omega_out < 0.0f);
  assert(c.observe(-0.5f,1300000,8,true));
  auto right=c.update(0.10f,0.0f,0.02f,true);
  assert(right.rate_corr>0.0f); // right turn rate => counterclockwise correction
  assert(std::fabs(right.correction)<=0.07501f);
  c.inactive();
  const float persisted=c.yawRadians();
  auto idle=c.update(0,0,0.02f,false);
  assert(idle.mode==3 && std::fabs(c.yawRadians()-persisted)<1e-6f);
  // A turn integrates a command-derived REFERENCE; command w is a rate,
  // never interpreted as an absolute target angle.
  auto turn=c.update(0.05f,0.40f,0.02f,true);
  assert(turn.mode==2 && std::fabs(turn.heading_ref_rad-c.yawRadians()-0.008f)<0.0001f);
  // A sensor dropout does not reset yaw nor fabricate rotation across gap.
  const float pre_gap=c.yawRadians();
  c.lostSample();
  assert(c.observe(0.0f,1680000,9,true));
  assert(std::fabs(c.yawRadians()-pre_gap)<1e-6f);
  std::cout<<"Heading observer/outer control tests PASS\n";
}
