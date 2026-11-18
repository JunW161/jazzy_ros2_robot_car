#include "../main/yaw_outer.h"
#include "../main/controller_math.h"
#include <cassert>
#include <cmath>
#include <iostream>
static bool near(float a,float b,float tol=0.002f){return std::fabs(a-b)<tol;}
int main(){
  using namespace YawOuter;
  Controller c;
  // 50 Hz, sample acquired at precise timestamps.
  assert(c.observe(0.0f,1000000,1,true));
  auto r=c.update(.10f,0.0f,.02f,true);
  assert(!r.active && r.correction==0 && near(r.heading_err,0));
  // +0.4 deg: below exact 0.5deg threshold, no output (including D).
  assert(c.observe(.34906585f,1020000,2,true)); // trapezoid +0.2 deg
  assert(c.observe(.34906585f,1040000,3,true)); // now total +0.6deg
  r=c.update(.10f,0,.02f,true);
  assert(r.active && r.heading_err<0 && r.correction<0 && r.p_term<0);
  float l=0,rr=0;
  ControllerMath::differential_rpms(.10f,r.omega_out,.195f,.044f,.044f,l,rr);
  assert(l>rr); // robot deviated left => require turn right => left wheel faster
  // When deviation is under 0.5deg after re-latching: exact zero correction.
  c.inactive();
  r=c.update(.10f,0,.02f,true);
  assert(!r.active && r.correction==0);
  // Negative yaw error (turn right) => positive correction, R wheel faster.
  assert(c.observe(-.34906585f,1060000,4,true));
  assert(c.observe(-.34906585f,1080000,5,true));
  assert(c.observe(-.34906585f,1100000,6,true));
  r=c.update(.10f,0,.02f,true);
  assert(r.active && r.heading_err>0 && r.correction>0);
  ControllerMath::differential_rpms(.10f,r.omega_out,.195f,.044f,.044f,l,rr);
  assert(rr>l);
  // PID I is selectable; integral builds only when outside threshold.
  c.setGains({0.0f,0.20f,0.0f});
  c.inactive();c.update(.10f,0,.02f,true);
  assert(c.observe(.5f,1120000,7,true));
  assert(c.observe(.5f,1140000,8,true));
  auto out=c.update(.10f,0,.02f,true);
  assert(out.active && out.i_term<0 && out.p_term==0 && out.d_term==0);
  const float earlier=out.i_term;
  for(int i=0;i<400;i++)out=c.update(.10f,0,.02f,true);
  assert(out.i_term<=earlier && std::fabs(out.i_term)<=0.04001f);
  assert(std::fabs(out.correction)<=.07501f);
  // No new drive when stopped; require external state machine to act on cmd.
  c.inactive();assert(!c.update(0,0,.02f,true).active);
  // Straight, turn and reversed steering REP-103 kinematics.
  Controller t;
  assert(t.observe(0,1000000,1,true));
  t.update(.10f,.20f,.02f,true);
  auto arc=t.update(.10f,.20f,.02f,true);
  assert(arc.heading_ref_rad>0 && near(arc.curvature_inv_m,2,0.0001f));
  std::cout<<"HEADING PID threshold/sign/integral/limits/reverse tests PASS\n";
}
