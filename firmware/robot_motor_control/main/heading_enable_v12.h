#pragma once
// Robot_Delivery_V12: deterministic outer-yaw ON/OFF policy.
// Pure C++, independently tested; only motor task calls step().
// Safety: OFF disables correction regardless of robot motion. Guard latch
// can reset only after an OFF was explicitly seen AND robot is fully IDLE.
namespace HeadingEnableV12 {
struct State {
  bool effective=false;
  bool pendingOff=false;  // explicit OFF not yet acknowledged while stationary
};
struct Decision {
  bool resetController=false;
  bool resetGuard=false;
  bool transition=false;
  int status=0; // 1=ON, 2=OFF, -2=OFF waiting for IDLE, -3=guard, -4=OFF must be acknowledged
};
inline Decision step(State& s,bool requested,bool sawOff,bool safeIdle,bool guardBlocked) {
  Decision d;
  const bool previous=s.effective;
  if(sawOff) s.pendingOff=true;
  if(!requested) {
    s.effective=false;
    if(s.pendingOff && safeIdle) {
      s.pendingOff=false;
      d.resetGuard=true;
    }
    d.status=s.pendingOff?-2:2;
  } else if(s.pendingOff) {
    // Fast OFF->ON cannot silently clear a fault or skip an OFF ack.
    s.effective=false;
    d.status=-4;
  } else if(guardBlocked) {
    s.effective=true;
    d.status=-3;
  } else {
    s.effective=true;
    d.status=1;
  }
  d.transition=previous!=s.effective;
  d.resetController=d.transition||sawOff;
  return d;
}
} // namespace HeadingEnableV12
