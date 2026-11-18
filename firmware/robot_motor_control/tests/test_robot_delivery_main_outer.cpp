#include "../main/yaw_outer.h"
#include "../main/controller_math.h"
#include "../main/heading_feedback_guard.h"
#include <cassert>
#include <cmath>
#include <cstdio>
static bool close(float x,float y,float eps=0.0001f){return std::fabs(x-y)<eps;}
int main(){
 using namespace YawOuter;
 static constexpr float DEG=3.14159265358979323846f/180.0f;
 // 0.5 deg precisely is NOT enough; 0.51 deg must start correction now.
 Controller c;
 uint64_t t=1000000;uint32_t seq=1;
 assert(c.observe(0,t,seq,true));
 c.update(.10f,0,.02f,true);
 auto advance=[&](float g){t+=20000;++seq;assert(c.observe(g,t,seq,true));};
 advance(0.4f);advance(0.4f);  // +0.69deg integrated
 auto r=c.update(.10f,0,.02f,true);
 assert(r.active&&r.heading_err< -0.5f*DEG&&r.correction<0);
 float l=0,right=0;
 ControllerMath::differential_rpms(.1f,r.omega_out,.195f,.044f,.044f,l,right);
 assert(l>right); // actual yaw left => steer right => left wheel faster
 // Duplicate snapshot must NOT integrate a second time.
 const float stored=c.yawRadians();
 assert(c.observe(.4f,t,seq,true));assert(close(c.yawRadians(),stored));
 // Request exactly zero gyro rate and let heading recover with the signed PID.
 float command=r.omega_out;
 bool recovered=false;
 for(int i=0;i<1600;++i){
   advance(command);
   r=c.update(.10f,0,.02f,true);
   assert(std::fabs(r.correction)<Controller::CORRECTION_LIMIT_RADPS+1e-6f);
   if(!r.active){recovered=true;break;}
   command=r.omega_out;
 }
 assert(recovered); // with ideal wheel tracking, robot eventually steers straight
 assert(std::fabs(r.heading_err)<=Controller::HEADING_RELEASE_RAD+1e-4f);
 // Hysteresis: 0.3 to 0.5 deg won't flap while recovery is active.
 Controller h;uint64_t u=1000000;uint32_t n=1;
 assert(h.observe(0,u,n,true));h.update(.1f,0,.02f,true);
 for(int i=0;i<5;++i){u+=20000;++n;assert(h.observe(.2f,u,n,true));}
 auto a=h.update(.1f,0,.02f,true);assert(a.active);
 // Input gyro negative approaches target yet |error| can remain in dead zone.
 for(int i=0;i<4;++i){u+=20000;++n;assert(h.observe(-.12f,u,n,true));}
 a=h.update(.1f,0,.02f,true);
 // This overshot rate may cross release; either active with >release or inactive <release.
 assert(a.active || std::fabs(a.heading_err)<=Controller::HEADING_RELEASE_RAD+1e-5f);
 // After losing an IMU sample, steering is zero. On reacquire we latch current yaw.
 c.lostSample();auto z=c.update(.1f,0,.02f,false);assert(!z.active&&z.correction==0);
 t+=200000; ++seq; assert(c.observe(0,t,seq,true));
 z=c.update(.1f,0,.02f,true);assert(!z.active&&z.correction==0);
 // New motion yaw error right => positive command, right wheel target higher.
 Controller reverse;
 assert(reverse.observe(0,1000000,1,true));reverse.update(-.10f,0,.02f,true);
 assert(reverse.observe(-.4f,1020000,2,true));
 assert(reverse.observe(-.4f,1040000,3,true));
 auto rr=reverse.update(-.10f,0,.02f,true);
 assert(rr.active && rr.correction>0);
 ControllerMath::differential_rpms(-.10f,rr.omega_out,.195f,.044f,.044f,l,right);
 assert(right>l); // signed wheel RPM for reverse also uses REP-103 angular sign
 // STARTUP LAUNCH steering signs follow the *signed* differential RPM,
 // including reverse and spin-in-place (both wheel magnitudes should rise
 // if we ask for a faster positive CCW spin).
 auto adjustments=YawOuter::wheelRpmDelta(0.14f,.195f,.044f,.044f);
 assert(adjustments.left<0 && adjustments.right>0);
 assert(close(adjustments.right,-adjustments.left));
 assert(adjustments.right-adjustments.left>5.0f);
 const float forwardLeft=1.0f*adjustments.left;
 const float forwardRight=1.0f*adjustments.right;
 assert(forwardRight>forwardLeft); // positive yaw -> more right duty, less left
 const float reverseLeft=-1.0f*adjustments.left;
 const float reverseRight=-1.0f*adjustments.right;
 assert(reverseLeft>reverseRight); // reverse positive yaw -> more LEFT reverse duty
 const float spinLeft=-1.0f*adjustments.left;
 const float spinRight=1.0f*adjustments.right;
 assert(spinLeft>0&&spinRight>0); // spin in place: increase BOTH magnitudes
 // Conditional I anti-windup and hard output/slew bounds.
 Controller integral;
 integral.setGains({3.0f,1.5f,0.0f});
 assert(integral.observe(0,1000000,1,true));integral.update(.1f,0,.02f,true);
 assert(integral.observe(5.0f,1020000,2,true));
 assert(integral.observe(5.0f,1040000,3,true));
 for(int i=0;i<300;++i){
   auto v=integral.update(.1f,0,.02f,true);
   assert(std::fabs(v.correction)<=Controller::CORRECTION_LIMIT_RADPS+1e-6f);
   assert(std::fabs(v.i_term)<=Controller::INTEGRAL_TERM_LIMIT_RADPS+1e-6f);
 }
 // Sign guard only in deliberate spin; avoid spurious block in straight motion.
 HeadingGuard::Monitor guard;
 for(int i=0;i<30;++i)guard.update(.1f,0,.3f,-.4f,.02f);
 assert(!guard.blocked());
 for(int i=0;i<RobotConfig::HEADING_GUARD_SIGN_SAMPLES;++i)
   guard.update(0,.3f,.3f,-.4f,.02f);
 assert(guard.blocked()&&guard.reason()==4);
 guard.reset();assert(!guard.blocked());
 for(int i=0;i<RobotConfig::HEADING_GUARD_EXTREME_SAMPLES;++i)
   guard.update(.1f,0,0,0,.4f);
 assert(guard.blocked()&&guard.reason()==5);
 printf("Robot_Delivery_Main: turn signs, reverse, 0.5 trigger, hysteresis, recovery, IMU loss, guard, limits PASS\n");
}
