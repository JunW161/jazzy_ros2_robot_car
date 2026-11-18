#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "controller_math.h"
#include "kinematics.h"
using namespace ControllerMath;
static bool near(float a,float b,float tolerance=0.0001f){return fabsf(a-b)<tolerance;}
int main(){
  assert(command_age_us(1000000ULL,1000001ULL)==0); // regression for random PWM reset
  assert(command_age_us(1000000ULL,999980ULL)==20);

  assert(near(signed_speed_error(+40,+36),4));
  assert(near(signed_speed_error(-40,-36),4));
  assert(near(signed_speed_error(-40,+10),50)); // wrong direction must NOT be ignored
  assert(near(straight_trim(-0.1f,-0.08f,-1,172,2),2)); // left reverse leads
  assert(near(straight_trim(+0.1f,+0.08f,+1,172,2),2)); // left forward leads
  assert(near(straight_trim(+0.08f,+0.1f,+1,172,2),-2));
  assert(near(straight_trim(0.1f,0.05f,0,172,2),0));
  float l=0,r=0;
  differential_rpms(0.10f,0,0.195f,0.044f,0.044f,l,r);
  assert(l>0&&near(l,r));
  differential_rpms(-0.10f,0,0.195f,0.044f,0.044f,l,r);
  assert(l<0&&near(l,r));
  differential_rpms(0,0.8f,0.195f,0.044f,0.044f,l,r);
  assert(l<0&&r>0&&near(l,-r));
  differential_rpms(0.12f,0.3f,0.195f,0.044f,0.044f,l,r);
  assert(l>0&&r>l);
  differential_rpms(0.12f,-0.3f,0.195f,0.044f,0.044f,l,r);
  assert(r>0&&l>r);
  differential_rpms(0.10f,0,0.195f,0.042f,0.046f,l,r);
  assert(l>r); // small left wheel needs more RPM for equal linear speed
  Kinematics::Pose p;
  Kinematics::integrate(p,1,1,0.195);
  assert(fabs(p.x-1)<1e-9&&fabs(p.y)<1e-9&&fabs(p.yaw)<1e-9);
  puts("OK: 13+ controller/kinematics assertions");
}
