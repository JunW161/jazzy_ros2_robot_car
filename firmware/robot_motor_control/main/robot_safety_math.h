#pragma once
#include <math.h>
#include <stdint.h>
namespace RobotSafety {
// Near-field sensor is AUXILIARY to a future LiDAR! No clear road inferred from a missing return.
enum class NearState : uint8_t { UNKNOWN=0, OBSERVED_CLEAR=1, NEAR_STOP=2 };
struct RangeSample { float meters=0; uint64_t sample_us=0; bool valid=false; };
struct Limit { bool ok=false, blocked=true; float max_forward_mps=0; float clearance_m=0; NearState state=NearState::UNKNOWN; };
inline Limit forward_limit(const RangeSample &s,uint64_t now_us,
                          float sensor_to_bumper_m,float stop_clearance_m=0.06f,
                          float brake_mps2=0.30f,float latency_s=0.10f,
                          float margin_m=0.005f,uint32_t max_age_ms=180,
                          bool supervised_unknown=false,float supervised_cap=0.08f){
  Limit out;
  const bool fresh=s.sample_us>0&&now_us>=s.sample_us&&
      now_us-s.sample_us<=uint64_t(max_age_ms)*1000ULL;
  if(!fresh || !s.valid || !isfinite(s.meters) ||
     s.meters < 0.001f || s.meters > 0.10f){
    // Explicit supervised configuration, not autonomous obstacle clearance.
    if(supervised_unknown && fresh){out.blocked=false;out.max_forward_mps=supervised_cap;}
    return out;
  }
  out.ok=true;
  out.clearance_m=s.meters-sensor_to_bumper_m;
  const float usable=out.clearance_m-stop_clearance_m-margin_m;
  if(usable<=0.0f){out.state=NearState::NEAR_STOP;return out;}
  out.state=NearState::OBSERVED_CLEAR;
  if(brake_mps2<=0 || latency_s<0)return out;
  const float t=brake_mps2*latency_s;
  out.max_forward_mps=sqrtf(t*t+2.0f*brake_mps2*usable)-t;
  if(out.max_forward_mps>0)out.blocked=false;
  return out;
}
} // namespace RobotSafety
