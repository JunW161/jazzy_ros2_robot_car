#include "kinematics.h"
#include <assert.h>
#include <math.h>
int main(){
  using namespace Kinematics;
  Pose p;
  integrate(p,1,1,.195);
  assert(fabs(p.x-1)<1e-10 && fabs(p.y)<1e-10 && fabs(p.yaw)<1e-10);
  p=Pose{}; integrate(p,-.1,.1,.2);
  assert(fabs(p.x)<1e-10 && fabs(p.y)<1e-10 && fabs(p.yaw-1)<1e-10);
  p=Pose{}; integrate(p,.1,.2,.2);
  assert(p.x>0 && p.y>0 && p.yaw>0);
  double l,r;wheel_setpoints(.15,0,.195,.044,l,r);
  assert(fabs(l-32.5544)<0.001 && fabs(r-32.5544)<.001);
  wheel_setpoints(0,1,.195,.044,l,r);
  assert(l<0 && r>0);
  return 0;
}
