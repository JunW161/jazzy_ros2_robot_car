#include "../main/imu_calibration_v10.h"
#include <cassert>
#include <cmath>
#include <cstdio>
int main(){
 using namespace ImuCalibrationV10;
 Window s;
 for(int i=0;i<SAMPLES;++i){
   const float g[3]={-250.0f+float(i%7-3),70.0f+float(i%5-2),27.0f+float(i%3-1)};
   const bool done=s.push(g,10.54f+float(i%3-1)*0.01f);
   assert(done==(i==SAMPLES-1));
 }
 assert(s.done && s.count==150 && std::fabs(s.bias[0]+250.0f)<1.0f);
 Window a;
 float g[3]={-250,70,27};
 assert(!a.push(g,10.54f));
 g[0]=1500;assert(!a.push(g,10.54f));assert((a.reason&2)!=0&&a.count==0);
 g[0]=-250;assert(!a.push(g,12.0f));assert(a.reason==1&&a.count==0);
 assert(!a.push(g,10.54f));g[0]=0;assert(!a.push(g,10.54f));assert(a.reason==8&&a.count==0);
 Window noisy;
 for(int i=0;i<SAMPLES;++i){const float gyro[3]={-250.0f+float((i%2)?30:-30),70,27};
   bool done=noisy.push(gyro,10.54f);assert(!done);}
 assert(noisy.reason==4&&noisy.count==0);
 Window spin;
 for(int i=0;i<SAMPLES;++i){const float gyro[3]={-250,70,600};assert(!spin.push(gyro,10.54f));}
 assert(spin.reason==2&&spin.count==0);
 printf("V10 IMU stable factory offset, sample reject, noisy, movement, slow spin gate PASS\n");
}
