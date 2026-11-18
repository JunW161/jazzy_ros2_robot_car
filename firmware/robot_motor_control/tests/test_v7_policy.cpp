#include "../main/robot_safety_math.h"
#include "../main/yaw_outer.h"
#include <cassert>
#include <cmath>
int main(){
  using namespace RobotSafety;
  constexpr uint64_t now=1000000;
  auto near=forward_limit({0.055f,now,true},now,0,0.06f);
  assert(near.ok && near.blocked && near.state==NearState::NEAR_STOP);
  auto far=forward_limit({0.10f,now,true},now,0,0.06f);
  assert(far.ok && !far.blocked && far.max_forward_mps>0);
  auto offset=forward_limit({0.10f,now,true},now,0.03f,0.06f);
  assert(offset.max_forward_mps<0.05f); // 30mm offset leaves too little usable braking distance
  auto stale=forward_limit({0.09f,1,true},now,0);
  assert(!stale.ok && stale.blocked);
  auto unknown=forward_limit({0.15f,now,false},now,0);
  assert(!unknown.ok && unknown.blocked);
  auto supervised=forward_limit({0.15f,now,false},now,0,0.06f,0.3f,0.1f,0.005f,180,true,0.08f);
  assert(!supervised.ok && !supervised.blocked && supervised.max_forward_mps==0.08f);
  YawOuter::Controller yaw;
  // Positive yaw correction if robot drifts clockwise (gyro negative).
  yaw.reset(); auto a=yaw.update(0.1f,0.f,-0.04f,0.02f,true);
  assert(a.omega_out>0 && a.correction>0);
  auto b=yaw.update(-0.1f,0.f,0.04f,0.02f,true);
  assert(b.omega_out<0);
  return 0;
}
