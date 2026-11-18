#include "../main/motor_interlocks.h"
#include "../main/robot_safety_math.h"
#include <cassert>
#include <cstring>
int main(){
  using namespace MotorInterlock;
  assert(std::strcmp(name(FRONT_UNKNOWN),"FRONT_UNKNOWN_NO_LIDAR")==0);
  assert(std::strcmp(name(START_ENCODER_TIMEOUT),"START_ENCODER_TIMEOUT")==0);
  constexpr uint64_t now=1000000;
  // A short-range ToF cannot declare an open corridor safe when out of range.
  auto unknown=RobotSafety::forward_limit({0,0,false},now,0.03f);
  assert(!unknown.ok && unknown.blocked);
  auto close=RobotSafety::forward_limit({0.06f,now,true},now,0.03f);
  assert(close.blocked);
  auto observed=RobotSafety::forward_limit({0.10f,now,true},now,0.03f);
  assert(observed.ok && observed.max_forward_mps<0.05f);
  return 0;
}
