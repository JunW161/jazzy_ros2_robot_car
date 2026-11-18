#include "../main/yaw_outer.h"
#include "../main/heading_enable_v12.h"
#include "../main/heading_feedback_guard.h"
#include <cassert>
#include <cmath>
#include <cstdio>

static bool eq(float a, float b,float e=1e-5f){return std::fabs(a-b)<e;}
int main(){
 using namespace HeadingEnableV12;
 State s;
 // 1: initial ON. 2: explicit OFF while driving (always stops steering).
 auto d=step(s,true,false,true,false);
 assert(s.effective&&d.status==1&&d.resetController);
 d=step(s,false,true,false,false);
 assert(!s.effective&&s.pendingOff&&d.resetController&&d.status==-2);
 // ON before an OFF is acknowledged MUST NOT bypass a guard.
 d=step(s,true,false,false,true);
 assert(!s.effective&&d.status==-4&&s.pendingOff);
 // OFF while IDLE acknowledges reset, then ON works again.
 d=step(s,false,true,true,true);
 assert(!s.effective&&!s.pendingOff&&d.resetGuard&&d.status==2);
 d=step(s,true,false,true,false);
 assert(s.effective&&d.status==1&&d.resetController);
 // Repeated cycles do not require reboot.
 for(int k=0;k<20;k++){
   d=step(s,false,true,true,true);
   assert(!s.effective&&d.resetGuard&&d.status==2);
   d=step(s,true,false,true,false);
   assert(s.effective&&d.status==1);
 }
 // Guard cannot reset from ON alone.
 d=step(s,true,false,true,true);
 assert(!s.effective&&d.status==-3&&!d.resetGuard);
 // Tricky OFF->ON in a single 20 ms interval is reported, never silently lost.
 d=step(s,true,true,true,true);
 assert(!s.effective&&s.pendingOff&&d.status==-4&&!d.resetGuard);
 d=step(s,false,true,true,true);
 assert(d.resetGuard&&d.status==2);
 // PID threshold and direction both signs, and idle never drives motors.
 using namespace YawOuter;
 Controller c;
 assert(eq(c.gains().kp,0.50f)&&eq(c.gains().ki,0.06f)&&eq(c.gains().kd,0.10f));
 constexpr float deg=3.14159265358979323846f/180.0f;
 assert(eq(Controller::HEADING_TRIGGER_RAD,1.2f*deg));
 assert(eq(Controller::HEADING_RELEASE_RAD,1.f*deg));
 assert(c.observe(0,1000000,1,true));
 auto r=c.update(0,0,0.02f,true);
 assert(!r.active&&r.omega_out==0);
 assert(c.observe(0,1020000,2,true));
 c.update(.1f,0,.02f,true);
 for(int i=1;i<=6;i++)assert(c.observe(.3f,1020000+20000*i,2+i,true));
 r=c.update(.1f,0,.02f,true);
 assert(r.active&&r.heading_err<0&&r.correction<0);
 auto del=wheelRpmDelta(r.correction,.195f,.044f,.044f);
 assert(del.left>del.right);
 auto rIdle=c.update(0,0,.02f,true);
 assert(!rIdle.active&&eq(rIdle.correction,0));
 Controller neg;
 assert(neg.observe(0,1000000,1,true));
 neg.update(.1f,0,.02f,true);
 for(int i=1;i<=6;i++)assert(neg.observe(-.3f,1000000+20000*i,1+i,true));
 auto nr=neg.update(.1f,0,.02f,true);
 assert(nr.active&&nr.heading_err>0&&nr.correction>0);
 auto dl=wheelRpmDelta(nr.correction,.195f,.044f,.044f);
 assert(dl.right>dl.left);
 puts("V12 C++ tests passed: repeated ON/OFF, fast toggle, guard, idle, signed yaw");
}
