#include "../main/robot_safety_math.h"
#include "../main/yaw_outer.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(){
 using namespace RobotSafety;
 auto bad=forward_limit({0,0,false},1000000,0.03f);assert(!bad.ok&&bad.blocked);
 auto stale=forward_limit({0.20f,1000000,true},1400000,0.03f);assert(!stale.ok);
 auto hard=forward_limit({0.09f,1000000,true},1010000,0.03f);assert(hard.ok&&hard.blocked);
 auto far=forward_limit({0.80f,1000000,true},1010000,0.03f);assert(far.ok&&!far.blocked&&far.max_forward_mps>0.1f);
 auto near=forward_limit({0.16f,1000000,true},1010000,0.03f);assert(near.ok&&near.max_forward_mps<far.max_forward_mps);
 YawOuter::Controller ctrl;
 // Robot heading drifts clockwise with zero yaw demand => positive corrective yaw.
 YawOuter::Result y;for(int i=0;i<60;i++)y=ctrl.update(0.10f,0.0f,-0.07f,0.02f,true);
 assert(y.mode==1 && y.correction>0 && y.omega_out>0);
 ctrl.reset();for(int i=0;i<60;i++)y=ctrl.update(-0.10f,0.0f,0.05f,0.02f,true);
 assert(y.mode==1 && y.correction<0);
 ctrl.reset();for(int i=0;i<50;i++)y=ctrl.update(0.10f,0.25f,0.10f,0.02f,true);
 assert(y.mode==2 && y.correction>0 && y.omega_out>0.25f);
 ctrl.reset();for(int i=0;i<50;i++)y=ctrl.update(0.0f,-0.25f,-0.10f,0.02f,true);
 assert(y.mode==2 && y.correction<0);
 ctrl.reset();y=ctrl.update(0.1f,0.0f,1.0f,0.2f,true);assert(y.mode==0 && y.correction==0);
 puts("V6 policy and outer yaw tests: PASS");
}
