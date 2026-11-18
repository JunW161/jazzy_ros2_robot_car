#pragma once
#include <stdint.h>

// Snapshots produced by the 50 Hz motor/odometry task; ROS callbacks NEVER drive GPIO.
struct WheelReport {
  float targetL=0,targetR=0,rpmL=0,rpmR=0,pwmL=0,pwmR=0;
  float distanceL=0,distanceR=0,headingDeg=0,state=0;
  double x=0,y=0,yaw=0,linear=0,angular=0;
  double jointPosL=0,jointPosR=0,jointVelL=0,jointVelR=0;
  // control_debug: 14 floats. Setpoint after straight synchronizer, FF, PID terms, PWM command.
  float refL=0,refR=0,ffL=0,ffR=0;
  float pL=0,iL=0,dL=0,pR=0,iR=0,dR=0;
  float demandL=0,demandR=0;
  uint32_t watchdogStopCount=0,controlOverrunCount=0;
  float yawCorrection=0, gyroZ=0, safeSpeedLimit=-1, obstacleClearance=-1;
  // /robot/heading_status at 10 Hz. Relative yaw from integrated gyro, never
  // substitute this for an absolute magnetometer/AMCL yaw measurement.
  float imuYawDeg=0, headingTargetDeg=0, headingErrorDeg=0;
  float yawRateCorrection=0, yawHeadingCorrection=0;
  float yawRequested=0, yawEffective=0, yawMode=0;
  float imuFresh=0, imuAgeMs=-1;
  float headingPidEnabled=0, headingPidActive=0, headingKp=0, headingKi=0, headingKd=0;
  float headingP=0, headingI=0, headingD=0, headingSaturated=0;
  // Extended read-only /robot/heading_status diagnostics, indices 22..34.
  float imuInitialized=0,imuCalProgress=0,imuErrors=0,imuSeq=0;
  float imuRawZCounts=0,imuBiasZCounts=0,imuWho=0;
  float imuAccelNorm=0,imuCalReject=0,imuCalRestarts=0;
  float encoderYawRate=0,headingGuardReason=0,steeringAuthority=0;
  float headingDeltaRpm=0,headingSlewLimited=0;
  float headingSetStatus=0, headingSetSeq=0;
  // V12 confirmation fields; append only to /robot/heading_status.
  float headingEnableRxSeq=0,headingEnableAppliedSeq=0;
  float headingGuardResetCount=0,headingEnableApplyStatus=0;
  // /robot/trajectory_status: local body-relative SE(2) command/reference
  // and gyro+encoder dead-reckoning. Units SI. NOT map-localized.
  float pathXRef=0,pathYRef=0,pathXEst=0,pathYEst=0;
  float pathCrossTrack=0,pathAlongTrack=0,pathCurvature=0,pathRadius=0;
  float pathCorrection=0;

  float interlockReason=0;  // V8 /robot/safety_status[8], see docs
  float benchArmed=0, benchRemainingMs=0, cmdFresh=0;
  float sensorFlags=0; // diagnostic only: 0=both measured; 1=IMU stale; 2=ToF unavailable
  uint64_t sampleUs=0;
};
// state: 0 DISARMED, 1 IDLE, 2 LAUNCH, 3 ACTIVE, 4 FAULT, 5 DECEL
void robot_drive_start();
void robot_drive_cmd_vel(float v_mps,float wz_radps);
void robot_drive_get_report(WheelReport &r);
// PID RAM-only: kpL,kiL,kdL,kpR,kiR,kdR. Change only when stopped.
// Status: 0=NVS baseline; 10=queued; 1=applied; -1=invalid; -2=busy.
void robot_drive_pid_request(const float data[6],bool valid_length);
void robot_drive_pid_status(float out[8]);

// link_debug: latest cmd age(ms), max callback interval(ms), watchdog stop count,
// invalid command count, control-loop long cycles, current motor state.
void robot_drive_link_debug(float out[6]);

// BENCH-only arm, never used in autonomous floor driving; no NVS persistence.
void robot_drive_bench_request(bool arm);

// Heading PID: gain updates are accepted ONLY in IDLE with stopped wheels.
// ROS: /robot/heading_pid_set [Kp,Ki,Kd], /robot/heading_enable Bool.
// /robot/heading_status extends legacy indices 0..9, see README.
void robot_drive_heading_pid_request(const float data[3],bool valid_length);
void robot_drive_heading_enable_request(bool enabled);

// V10 FF calibration IDLE-only: RAM changes, optional explicit NVS save.
// /robot/ff_set: [wheel(0=L,1=R), direction(+1=forward,-1=reverse), rpm_breakpoint, pwm]
// /robot/ff_scale_set: [wheel, direction, scale(0.5..1.5)]
// /robot/ff_save and /robot/ff_reset: Bool true
// /robot/ff_status: 43 floats, [0]=result [1]=sequence [2]=stored,
// [3..6]=scales FL,FR,RL,RR, then 9 point tables FL,FR,RL,RR.
// result: 0 default, 1 point/scale applied, 2 saved, 3 reset, -1 invalid,
// -2 busy/not IDLE, -3 NVS error.
void robot_drive_ff_point_request(const float data[4],bool valid_length);
void robot_drive_ff_scale_request(const float data[3],bool valid_length);
void robot_drive_ff_save_request(bool save);
void robot_drive_ff_reset_request(bool reset);
void robot_drive_ff_status(float out[43]);
