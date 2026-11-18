#pragma once
#include <math.h>
#include <stdint.h>
namespace Kinematics {
constexpr double PI_D=3.1415926535897932384626433832795;
inline double clamp(double x,double a,double b){return x<a?a:(x>b?b:x);}
inline double wrap(double a){return atan2(sin(a),cos(a));}
inline double wheel_m(int32_t ticks,double radius_m,double cpr){return double(ticks)*2.0*PI_D*radius_m/cpr;}
inline double rpm_from_mps(double v,double radius_m){return v*60.0/(2.0*PI_D*radius_m);}
inline double mps_from_rpm(double rpm,double radius_m){return rpm*(2.0*PI_D*radius_m)/60.0;}
struct Pose { double x=0,y=0,yaw=0; };
inline void integrate(Pose& p,double dl,double dr,double track_m){
  double ds=(dl+dr)*0.5, dtheta=(dr-dl)/track_m;
  // Midpoint integration: accurate for small 20-ms encoder increments.
  p.x+=ds*cos(p.yaw+0.5*dtheta);
  p.y+=ds*sin(p.yaw+0.5*dtheta);
  p.yaw=wrap(p.yaw+dtheta);
}
inline void wheel_setpoints(double v,double w,double track_m,double radius_m,double &left_rpm,double &right_rpm){
  left_rpm=rpm_from_mps(v-0.5*w*track_m,radius_m);
  right_rpm=rpm_from_mps(v+0.5*w*track_m,radius_m);
}
}
