#include "../main/yaw_outer.h"
#include <cassert>
#include <cmath>
#include <iostream>
static bool near(float a,float b,float tol=0.008f){return std::fabs(a-b)<tol;}
int main(){
  using namespace YawOuter;
  Controller c;
  assert(c.observe(0.f,1000000,1,true));
  auto start=c.update(0.1f,0.2f,0.02f,true);
  assert(start.mode==2);
  Result arc;
  for(int i=1;i<=50;++i){
    assert(c.observe(0.2f,1000000+20000ULL*i,1+i,true));
    c.observeTravel(0.1f*0.02f);
    arc=c.update(0.1f,0.2f,0.02f,true);
  }
  assert(near(arc.heading_ref_rad,0.204f,0.005f));
  assert(near(arc.curvature_inv_m,2.0f,0.0001f));
  assert(near(arc.radius_m,0.5f,0.0001f));
  assert(near(arc.x_ref_m,0.5f*std::sin(0.204f),0.005f));
  assert(near(arc.y_ref_m,0.5f*(1-std::cos(0.204f)),0.005f));
  assert(std::fabs(arc.cross_track_m)<0.010f);
  // Stop turning and go straight: retain tangent of the arc.
  float target=arc.heading_ref_rad;
  assert(c.observe(0.f,2020000,52,true));
  c.observeTravel(0.002f);
  auto straight=c.update(0.1f,0.f,0.02f,true);
  assert(straight.mode==1);
  assert(near(straight.heading_ref_rad,target,1e-5f));
  assert(straight.rate_corr<=0.f); // no positive rate request after turn
  assert(std::fabs(straight.correction)<=0.07501f); // bounded transient
  // Standing IDLE keeps yaw, no active motor output.
  c.inactive();float persisted=c.yawRadians();
  assert(c.observe(0.f,2040000,53,true));
  assert(near(c.yawRadians(),persisted,1e-6f));
  // A fresh reference in reverse still observes REP103 signs.
  auto reverse=c.update(-0.1f,-0.2f,0.02f,true);
  assert(reverse.radius_m>0); // signed R=v/w: negative / negative
  assert(near(reverse.curvature_inv_m,2.f,0.0001f));
  assert(reverse.heading_ref_rad<c.yawRadians());
  // Large gap: never fabricate rotation, reacquire heading on next active.
  float before=c.yawRadians();
  c.lostSample();
  assert(c.observe(0.f,2300000,54,true));
  assert(near(c.yawRadians(),before,1e-6f));
  auto fresh=c.update(0.1f,0.f,0.02f,true);
  assert(near(fresh.heading_ref_rad,c.yawRadians(),1e-6f));
  // Offset-left -> correction-right in forward motion; if enabled.
  Controller c2;
  assert(c2.observe(0.f,1000000,1,true));
  c2.update(0.1f,0.f,0.02f,true,true);
  for(int i=1;i<=50;i++){
     c2.observe(0.0f,1000000+20000ULL*i,1+i,true);
     c2.observeTravel(0.002f);
     c2.update(0.1f,0.f,0.02f,true,true);
  }
  auto onpath=c2.update(0.1f,0.f,0.02f,true,true);
  assert(near(onpath.path_corr,0.0f,1e-5f));
  // Default disabled even in a curve: global tracking belongs to Nav2.
  assert(near(arc.path_corr,0.0f,1e-7f));
  std::cout<<"REP103 local twist/arc/heading/reverse/IMU tests PASS\n";
}
