#include "robot_drive.h"
#include "robot_config.h"
#include "kinematics.h"
#include "controller_math.h"
#include "yaw_outer.h"
#include "heading_feedback_guard.h"
#include "heading_enable_v12.h"
#include "robot_sensors.h"
#include "ff_calibration_v10.h"
#include "motor_interlocks.h"
#include <math.h>
#include <stdio.h>
#include <initializer_list>
#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
using namespace RobotConfig;
namespace {
constexpr ledc_mode_t MODE=LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t TIMER=LEDC_TIMER_0;
constexpr ledc_channel_t L_F=LEDC_CHANNEL_0,L_R=LEDC_CHANNEL_1;
constexpr ledc_channel_t R_F=LEDC_CHANNEL_2,R_R=LEDC_CHANNEL_3;
portMUX_TYPE encMux=portMUX_INITIALIZER_UNLOCKED;
volatile int32_t rawL=0,rawR=0;
portMUX_TYPE inputMux=portMUX_INITIALIZER_UNLOCKED;
float cmdV=0,cmdW=0;
uint64_t cmdLastUs=0;
bool cmdEver=false,cmdBad=false;
portMUX_TYPE reportMux=portMUX_INITIALIZER_UNLOCKED;
WheelReport published;
portMUX_TYPE pidMux=portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE ffMux=portMUX_INITIALIZER_UNLOCKED;
FeedforwardV10::Calibration ffConfig=FeedforwardV10::defaults();
float ffResult=0,ffSequence=0,ffStored=0;
constexpr const char* FF_NVS_NAMESPACE="robot_ff_v10";
constexpr const char* FF_NVS_KEY="config";

float activePid[6]={},requestedPid[6]={};
bool pidPending=false;float pidResult=0,pidSequence=0;
struct Gains{float kp,ki,kd;};
struct PID{float integral=0,prev=0,d=0,lastP=0,lastD=0;bool ready=false;};
Gains gl={FALLBACK_KP_L,FALLBACK_KI_L,FALLBACK_KD_L};
Gains gr={FALLBACK_KP_R,FALLBACK_KI_R,FALLBACK_KD_R};
PID pl,pr;
Kinematics::Pose pose;
float filteredL=0,filteredR=0,pwmL=0,pwmR=0,ditherL=0,ditherR=0;
float targetL=0,targetR=0,rampL=0,rampR=0;
float debugRefL=0,debugRefR=0,debugFfL=0,debugFfR=0;
float debugDemandL=0,debugDemandR=0;
int32_t oldL=0,oldR=0,originL=0,originR=0,syncOriginL=0,syncOriginR=0;
double accumL=0,accumR=0;
double syncDistL=0,syncDistR=0;
uint32_t watchdogStops=0, invalidCommands=0, controlOverruns=0;
uint32_t maxCmdGapMs=0;
uint64_t previousCmdUs=0;
uint64_t launchUs=0;
float launchDuty=0;
int movingBoth=0;
bool oldStraight=false;
int straightSign=0;
int launchSignL=1, launchSignR=1;
YawOuter::Controller yawCtl;
HeadingGuard::Monitor headingGuard;
float filteredWheelOmega=0.0f;
portMUX_TYPE headingMux=portMUX_INITIALIZER_UNLOCKED;
bool headingEnabled=RobotConfig::YAW_OUTER_ENABLED && RobotConfig::HEADING_DEFAULT_ENABLED;
bool headingPrevEnabled=RobotConfig::YAW_OUTER_ENABLED && RobotConfig::HEADING_DEFAULT_ENABLED;
// V12: ROS callback records desired state and every OFF. Motor task alone
// owns the controller and sticky guard. Never lose OFF due to rapid OFF->ON.
bool headingResetRequested=false;
uint32_t headingEnableRxSeq=0,headingEnableAppliedSeq=0,headingGuardResetCount=0;
uint32_t headingGuardIdleTicks = 0;
int headingEnableApplyStatus=0; // 1 ON,2 OFF,-2 wait stop,-3 guard,-4 OFF not acknowledged
HeadingEnableV12::State headingSwitch{};
float headingPidPending[3]={YawOuter::Controller::KP_HEADING,0.06f,YawOuter::Controller::KP_RATE};
float headingPidApplied[3]={YawOuter::Controller::KP_HEADING,0.06f,YawOuter::Controller::KP_RATE};
bool headingGainPending=false;
float headingSetResult=0,headingSetSeq=0;
float headingP=0,headingI=0,headingD=0,headingSaturated=0,headingActive=0;
float headingSlewLimited=0,steeringAuthority=0,headingDeltaRpm=0,debugWheelOmega=0;
float imuInitialized=0,imuCalProgress=0,imuErrors=0,imuSeq=0;
float imuRawZ=0,imuBiasZ=0,imuWho=0;
float imuAccelNorm=0,imuCalReject=0,imuCalRestarts=0;
float headingDebugEnabled=(RobotConfig::YAW_OUTER_ENABLED && RobotConfig::HEADING_DEFAULT_ENABLED)?1.0f:0.0f;
float yawCorrection=0,debugGyro=0,debugSafeSpeed=-1,debugClearance=-1,debugSensorFlags=0;
float yawRefDeg=0,headingErrorDeg=0,yawRateCorr=0,yawHeadingCorr=0;
float yawRequested=0,yawEffective=0,yawMode=0,imuSampleAgeMs=-1;
float pathXRef=0,pathYRef=0,pathXEst=0,pathYEst=0;
float pathCte=0,pathAlong=0,pathKappa=0,pathRadius=0,pathCorrection=0;

float debugInterlock=MotorInterlock::WAIT_CMD;
portMUX_TYPE benchMux=portMUX_INITIALIZER_UNLOCKED;
bool benchArmed=false;
uint64_t benchUntilUs=0;
uint32_t benchRefused=0;
bool benchArmedNow(uint64_t now, float &timeMs){
  portENTER_CRITICAL(&benchMux);
  if(benchArmed && now>=benchUntilUs) benchArmed=false;
  const bool active=benchArmed;
  timeMs=active?float((benchUntilUs-now)/1000ULL):0.0f;
  portEXIT_CRITICAL(&benchMux);
  return active;
}
void benchClear(){portENTER_CRITICAL(&benchMux);benchArmed=false;benchUntilUs=0;portEXIT_CRITICAL(&benchMux);}

bool calibrated=false;
float radiusM=RADIUS_M,trackM=TRACK_M;
enum class State{DISARMED=0,IDLE=1,LAUNCH=2,ACTIVE=3,FAULT=4,DECEL=5};
State state=State::DISARMED;
int stationaryTimeoutTicks=0;
float clampf(float v,float a,float b){return v<a?a:(v>b?b:v);}
float approach(float v,float to,float step){return v<to?fminf(v+step,to):fmaxf(v-step,to);}
void IRAM_ATTR leftA(void*){portENTER_CRITICAL_ISR(&encMux);rawL+=(gpio_get_level((gpio_num_t)L_ENC_A)==gpio_get_level((gpio_num_t)L_ENC_B))?1:-1;portEXIT_CRITICAL_ISR(&encMux);}
void IRAM_ATTR leftB(void*){portENTER_CRITICAL_ISR(&encMux);rawL+=(gpio_get_level((gpio_num_t)L_ENC_A)!=gpio_get_level((gpio_num_t)L_ENC_B))?1:-1;portEXIT_CRITICAL_ISR(&encMux);}
void IRAM_ATTR rightA(void*){portENTER_CRITICAL_ISR(&encMux);rawR+=(gpio_get_level((gpio_num_t)R_ENC_A)==gpio_get_level((gpio_num_t)R_ENC_B))?1:-1;portEXIT_CRITICAL_ISR(&encMux);}
void IRAM_ATTR rightB(void*){portENTER_CRITICAL_ISR(&encMux);rawR+=(gpio_get_level((gpio_num_t)R_ENC_A)!=gpio_get_level((gpio_num_t)R_ENC_B))?1:-1;portEXIT_CRITICAL_ISR(&encMux);}
void readEnc(int32_t &l,int32_t &r){portENTER_CRITICAL(&encMux);l=-rawL;r=rawR;portEXIT_CRITICAL(&encMux);}
float ffDuty(float rpm,bool left){
  FeedforwardV10::Calibration snapshot;
  portENTER_CRITICAL(&ffMux);snapshot=ffConfig;portEXIT_CRITICAL(&ffMux);
  return FeedforwardV10::duty(snapshot,rpm,left);
}
bool ffSafeToEdit(){
  float v,w;
  portENTER_CRITICAL(&inputMux);v=cmdV;w=cmdW;portEXIT_CRITICAL(&inputMux);
  return state==State::IDLE && fabsf(v)<0.001f && fabsf(w)<0.001f &&
         fabsf(filteredL)<2.0f && fabsf(filteredR)<2.0f &&
         pwmL==0.0f && pwmR==0.0f;
}
void ffSetResult(float result,bool dirty){
  portENTER_CRITICAL(&ffMux);
  ffSequence+=1.0f;ffResult=result;if(dirty)ffStored=0;
  portEXIT_CRITICAL(&ffMux);
}
void ffLoadNvs(){
  nvs_handle_t h;
  if(nvs_open(FF_NVS_NAMESPACE,NVS_READONLY,&h)!=ESP_OK)return;
  FeedforwardV10::Persist record{};size_t sz=sizeof(record);
  const esp_err_t e=nvs_get_blob(h,FF_NVS_KEY,&record,&sz);
  nvs_close(h);
  FeedforwardV10::Calibration loaded;
  if(e==ESP_OK && sz==sizeof(record) && FeedforwardV10::unpack(record,loaded)){
    portENTER_CRITICAL(&ffMux);ffConfig=loaded;ffStored=1;portEXIT_CRITICAL(&ffMux);
    printf("FF_V10 loaded validated table + scales from NVS\n");
  }else{
    printf("FF_V10 NVS invalid or outdated; using firmware defaults\n");
  }
}
bool validGain(Gains a){return isfinite(a.kp)&&isfinite(a.ki)&&isfinite(a.kd) && a.kp>=0&&a.kp<=5&&a.ki>=0&&a.ki<=10&&a.kd>=0&&a.kd<=1;}
bool readNvsFloat(nvs_handle_t h,const char*key,float&v){float val=0;size_t sz=sizeof(val);if(nvs_get_blob(h,key,&val,&sz)!=ESP_OK||sz!=sizeof(val)||!isfinite(val))return false;v=val;return true;}
void loadCalibration(){
  nvs_handle_t h;
  // Radius / track from verified ESP-IDF calibration as compile-time fallback.
  float wheelMm=RADIUS_M*1000,trackMm=TRACK_M*1000;
  if(nvs_open("robot_pid_v2",NVS_READONLY,&h)==ESP_OK){
    float a,b;
    // Previous Arduino code stored mm in float NVS blob via Preferences.
    if(USE_NVS_GEOMETRY && readNvsFloat(h,"wheel_mm",a)&&readNvsFloat(h,"track_mm",b)&&a>=25&&a<=150&&b>=50&&b<=1000){
      wheelMm=a;trackMm=b;printf("CAL geometry from NVS\n");
    }
    Gains l,r;
    if(USE_NVS_PID && readNvsFloat(h,"kpl",l.kp)&&readNvsFloat(h,"kil",l.ki)&&readNvsFloat(h,"kdl",l.kd)&&
       readNvsFloat(h,"kpr",r.kp)&&readNvsFloat(h,"kir",r.ki)&&readNvsFloat(h,"kdr",r.kd)&&validGain(l)&&validGain(r)){
      gl=l;gr=r;printf("CAL PID from NVS\n");
    }
    nvs_close(h);
  }
  calibrated=wheelMm>=25&&wheelMm<=150&&trackMm>=50&&trackMm<=1000&&validGain(gl)&&validGain(gr);
  if(calibrated){radiusM=wheelMm*0.001f;trackM=trackMm*0.001f;}
  // Geometry can be changed after measurement; never infer from previous .ino default radius 88.
  printf("CAL radius_mm=%.3f track_mm=%.3f\n",wheelMm,trackMm);
  printf("CAL PID left=%.4f %.4f %.4f right=%.4f %.4f %.4f | NVS_PID=%d\n",gl.kp,gl.ki,gl.kd,gr.kp,gr.ki,gr.kd,(int)USE_NVS_PID);
  printf("CAL %s | MOTOR=%s | REVERSE=%s | GOAL=ROS2_HOST\n",calibrated?"VALID":"INVALID",
         ENABLE_MOTOR_OUTPUT?"ENABLED":"LOCKED",ENABLE_REVERSE?"ON":"OFF");
  portENTER_CRITICAL(&pidMux);
  activePid[0]=gl.kp;activePid[1]=gl.ki;activePid[2]=gl.kd;
  activePid[3]=gr.kp;activePid[4]=gr.ki;activePid[5]=gr.kd;
  portEXIT_CRITICAL(&pidMux);
}
void pwmWrite(ledc_channel_t ch,int duty){ESP_ERROR_CHECK(ledc_set_duty(MODE,ch,(uint32_t)duty));ESP_ERROR_CHECK(ledc_update_duty(MODE,ch));}
void motors(float l,float r){
  l=clampf(l,-PWM_MAX,PWM_MAX);r=clampf(r,-PWM_MAX,PWM_MAX);
  if(!ENABLE_MOTOR_OUTPUT||!calibrated){l=r=0;}
  if(!ENABLE_REVERSE){l=fmaxf(l,0);r=fmaxf(r,0);}
  int ll=(int)l,rr=(int)r;
  // Change direction only through a zero-PWM transition; keep opposite
  // channel at zero. No repeated four-channel off/on pulses each 20-ms tick.
  if(pwmL<0&&ll>=0)pwmWrite(L_R,0);
  if(pwmL>0&&ll<=0)pwmWrite(L_F,0);
  if(pwmR<0&&rr>=0)pwmWrite(R_R,0);
  if(pwmR>0&&rr<=0)pwmWrite(R_F,0);
  pwmWrite(L_F,ll>0?ll:0);pwmWrite(L_R,ll<0?-ll:0);
  pwmWrite(R_F,rr>0?rr:0);pwmWrite(R_R,rr<0?-rr:0);
  gpio_set_level((gpio_num_t)L_EN,ll!=0);gpio_set_level((gpio_num_t)R_EN,rr!=0);
  pwmL=ll;pwmR=rr;
}
void resetPid(){pl=PID{};pr=PID{};ditherL=ditherR=0;}
void motorStop(){yawCtl.inactive();yawCorrection=0;motors(0,0);targetL=targetR=rampL=rampR=0;
 debugRefL=debugRefR=debugFfL=debugFfR=debugDemandL=debugDemandR=0;launchDuty=0;movingBoth=0;resetPid();oldStraight=false;straightSign=0;stationaryTimeoutTicks=0;
 if(state!=State::FAULT)state=(calibrated&&ENABLE_MOTOR_OUTPUT)?State::IDLE:State::DISARMED;}
void fault(const char* why){
  printf("MOTOR FAULT: %s; motor off; reset MCU after resolving cause\n",why);
  if(strcmp(why,"START_ENCODER_TIMEOUT")==0) debugInterlock=MotorInterlock::START_ENCODER_TIMEOUT;
  else if(strcmp(why,"ENCODER_STALL_BOTH")==0) debugInterlock=MotorInterlock::ENCODER_STALL;
  else debugInterlock=MotorInterlock::MOTOR_FAULT;
  state=State::FAULT;benchClear();motorStop();
}
float pidDemand(float set,float sensed,float dt,float ff,Gains g,PID &p,bool settled){
  if(fabsf(set)<STOP_RPM){p=PID{};return 0;}
  // Motor direction is controlled by setpoint sign, but ERROR must honor actual wheel sign.
  const float direction=set>0?1.0f:-1.0f;
  const float measuredAlong=direction*sensed;
  const float err=ControllerMath::signed_speed_error(set,sensed);
  if(!p.ready){p.ready=true;p.prev=measuredAlong;}
  const float deriv=(measuredAlong-p.prev)/fmaxf(dt,0.001f);
  p.prev=measuredAlong;
  p.d+=D_ALPHA*(deriv-p.d);
  const float pp=g.kp*err;
  const float dd=settled?-g.kd*p.d:0;
  p.lastP=pp;p.lastD=dd;
  const float candidate=clampf(p.integral+g.ki*err*dt,-I_LIMIT_PWM,I_LIMIT_PWM);
  const float requested=pp+candidate+dd;
  // Conditional integration: do not wind up when trim or full FF+PID duty saturates.
  const bool atHigh=requested>=PID_TRIM_LIMIT_PWM || ff+requested>=PWM_MAX;
  const bool atLow=requested<=-PID_TRIM_LIMIT_PWM || ff+requested<=0;
  if(settled && !((err>0&&atHigh)||(err<0&&atLow)))p.integral=candidate;
  return clampf(pp+p.integral+dd,-PID_TRIM_LIMIT_PWM,PID_TRIM_LIMIT_PWM);
}
int dutyQuantize(float amount,float &accum){
  float mag=fabsf(amount);int n=(int)floorf(mag);accum+=mag-n;
  if(accum>=1){n++;accum-=1;}
  return (amount<0?-n:n);
}
void pidApplyIfStopped(bool cmdZero){
  float values[6];bool pending=false;
  portENTER_CRITICAL(&pidMux);
  if(pidPending){memcpy(values,requestedPid,sizeof(values));pidPending=false;pending=true;}
  portEXIT_CRITICAL(&pidMux);
  if(!pending)return;
  if(!cmdZero||state!=State::IDLE||fabsf(filteredL)>2||fabsf(filteredR)>2){
    portENTER_CRITICAL(&pidMux);pidResult=-2;portEXIT_CRITICAL(&pidMux);printf("PID rejected: stop wheels first\n");return;
  }
  gl={values[0],values[1],values[2]};gr={values[3],values[4],values[5]};resetPid();
  portENTER_CRITICAL(&pidMux);memcpy(activePid,values,sizeof(values));pidResult=1;portEXIT_CRITICAL(&pidMux);
  printf("PID RAM updated (NVS unchanged)\n");
}
// Called from the motor task, never from an ROS callback.
void headingApplyIfStopped(bool stopped){
  float g[3]={};bool pending=false;
  portENTER_CRITICAL(&headingMux);
  if(headingGainPending){memcpy(g,headingPidPending,sizeof(g));
     headingGainPending=false;pending=true;}
  portEXIT_CRITICAL(&headingMux);
  if(!pending)return;
  if(!stopped || state!=State::IDLE || fabsf(filteredL)>2 || fabsf(filteredR)>2){
    portENTER_CRITICAL(&headingMux);headingSetResult=-2;portEXIT_CRITICAL(&headingMux);
    printf("HEADING_PID_REJECTED: stop wheels before gain update\n");return;
  }
  yawCtl.setGains({g[0],g[1],g[2]});
  portENTER_CRITICAL(&headingMux);
  memcpy(headingPidApplied,g,sizeof(g));headingSetResult=1;
  portEXIT_CRITICAL(&headingMux);
  printf("HEADING_PID_APPLIED kp=%.4f ki=%.4f kd=%.4f RAM-only\n",g[0],g[1],g[2]);
}
void controlTask(void*){
  readEnc(oldL,oldR);originL=oldL;originR=oldR;
  uint64_t last=esp_timer_get_time();TickType_t wake=xTaskGetTickCount();
  bool benchWasActive=false;
  while(true){
    vTaskDelayUntil(&wake,pdMS_TO_TICKS(CONTROL_MS));
    uint64_t now=esp_timer_get_time();float dt=(now-last)*0.000001f;last=now;
    if(dt<=0||dt>0.25f){
      // Never run open-loop after an unexpectedly delayed control task.
      ++controlOverruns;motors(0,0);state=State::FAULT;
      debugInterlock=MotorInterlock::CONTROL_OVERRUN;benchClear();
      readEnc(oldL,oldR);continue;
    }
    if(dt>0.045f)++controlOverruns;
    int32_t a,b;readEnc(a,b);int32_t da=a-oldL,db=b-oldR;oldL=a;oldR=b;
    float rpmRawL=da*60.0f/(ENCODER_CPR*dt),rpmRawR=db*60.0f/(ENCODER_CPR*dt);
    filteredL+=RPM_ALPHA*(rpmRawL-filteredL);
    filteredR+=RPM_ALPHA*(rpmRawR-filteredR);
    // Effective radii: include experimentally measured correction factors.
    const double dl=Kinematics::wheel_m(da,radiusM*LEFT_RADIUS_FACTOR,ENCODER_CPR);
    const double dr=Kinematics::wheel_m(db,radiusM*RIGHT_RADIUS_FACTOR,ENCODER_CPR);
    accumL+=dl;accumR+=dr;
    Kinematics::integrate(pose,dl,dr,trackM);
    double velocity=0.5*(dl+dr)/dt,omega=(dr-dl)/(trackM*dt);
    filteredWheelOmega+=0.2f*(float(omega)-filteredWheelOmega);
    debugWheelOmega=filteredWheelOmega;
    float v=0,w=0;uint64_t rx=0;bool ever=false,bad=false;
    portENTER_CRITICAL(&inputMux);
    v=cmdV;w=cmdW;rx=cmdLastUs;ever=cmdEver;bad=cmdBad;
    portEXIT_CRITICAL(&inputMux);
    // IMPORTANT: read clock AFTER copying callback data. If callback ran between
    // previous motor-loop clock and this snapshot, rx may be newer than 'now'.
    const uint64_t cmdNow=esp_timer_get_time();
    const uint64_t ageUs=ControllerMath::command_age_us(cmdNow,rx);
    // future timestamp means fresh, NOT timeout
    const bool cmdFresh=ever&&!bad&&ageUs<uint64_t(CMD_TIMEOUT_MS)*1000ULL;
    const bool zeroCmd=(fabsf(v)<0.001f&&fabsf(w)<0.001f);
    pidApplyIfStopped(!cmdFresh||zeroCmd);
    headingApplyIfStopped(!cmdFresh||zeroCmd);
    bool requestedHeading=false,offEdge=false;
    uint32_t enableRx=0;
    portENTER_CRITICAL(&headingMux);
    requestedHeading=headingEnabled;
    offEdge=headingResetRequested;
    headingResetRequested=false;
    enableRx=headingEnableRxSeq;
    portEXIT_CRITICAL(&headingMux);
    // A guard fault is sticky. Only an explicit OFF acknowledged at full IDLE
    // clears it; receiving ON while an OFF is pending cannot bypass this rule.
    const bool safeGuardReset=(state==State::IDLE && (!cmdFresh||zeroCmd) &&
                               fabsf(filteredL)<2.0f && fabsf(filteredR)<2.0f &&
                               pwmL==0.0f && pwmR==0.0f);
    const auto action=HeadingEnableV12::step(
        headingSwitch,requestedHeading,offEdge,safeGuardReset,headingGuard.blocked());
    if(action.resetGuard){
      headingGuard.reset();
      ++headingGuardResetCount;
    }
    if(action.resetController){
      yawCtl.inactive();
      yawCorrection=0.0f;
      syncDistL=accumL;syncDistR=accumR;
    }
    headingPrevEnabled=headingSwitch.effective;
    const bool enabledNow=headingSwitch.effective;
    // Acknowledgement pertains to the motor-task state, not just callback RX.
    // Refresh status on a later safe-IDLE transition even without new commands.
    portENTER_CRITICAL(&headingMux);
    const bool newRequest=enableRx!=headingEnableAppliedSeq;
    headingEnableAppliedSeq=enableRx;
    headingEnableApplyStatus=action.status;
    portEXIT_CRITICAL(&headingMux);
    if(newRequest || action.resetGuard || action.transition){
      printf("HEADING_V12_APPLY rx=%lu effective=%d status=%d guard=%d resets=%lu\n",
             (unsigned long)enableRx,int(enabledNow),action.status,
             headingGuard.reason(),(unsigned long)headingGuardResetCount);
    }
    headingDebugEnabled=enabledNow?1.0f:0.0f;
    headingP=headingI=headingD=headingSaturated=headingActive=0.0f;
    const float previousSteeringAuthority=steeringAuthority;
    headingSlewLimited=steeringAuthority=headingDeltaRpm=0.0f;
    bool imu_ok=false;
    float benchRemaining=0;
    const bool benchMode=benchArmedNow(now,benchRemaining);
    const bool benchJustEnded=benchWasActive && !benchMode;
    benchWasActive=benchMode;
    if(benchJustEnded){
        // Never continue a previously bench-authorized motion under a different safety mode.
        motorStop();robot_drive_cmd_vel(0.0f,0.0f);
        debugInterlock=MotorInterlock::BENCH_EXPIRED;
        goto report;
    }
    if(state!=State::FAULT)debugInterlock=MotorInterlock::NONE;
    // Sensor observation is PASSIVE and runs in all motion states, IDLE
    // included. The yaw estimate must not reset after a turn or zero Twist.
    // The VL6180X is a near-range telemetry instrument only: it has NO
    // authority to cap speed, stop motors, or alter /cmd_vel.
    {
      ImuSample imu;robot_imu_snapshot(imu);
      RangeReport tof;robot_range_snapshot(tof);
      // V1 read `now` before reading the IMU snapshot. When the sensor-owner
      // task sampled between those actions, valid new timestamps looked
      // "future" and were rejected, intermittently resetting yaw control.
      const uint64_t sensorNow=esp_timer_get_time();
      imu_ok=imu.valid && imu.calibrated && imu.time_us>0 &&
             imu.time_us<=sensorNow && sensorNow-imu.time_us<=80000ULL && isfinite(imu.gz);
      const bool sampleAccepted=yawCtl.observe(imu.gz,imu.time_us,imu.seq,imu_ok);
      imu_ok=imu_ok && sampleAccepted;
      imuInitialized=imu.initialized?1.0f:0.0f;
      imuCalProgress=float(imu.calibration_progress);
      imuErrors=float(imu.errors);imuSeq=float(imu.seq);
      imuRawZ=imu.raw_gz_counts;imuBiasZ=imu.bias_gz_counts;
      imuWho=float(imu.who);
      imuAccelNorm=imu.accel_norm;
      imuCalReject=float(imu.calibration_reject_reason);
      imuCalRestarts=float(imu.calibration_restarts);
      imuSampleAgeMs=(imu.time_us>0 && imu.time_us<=sensorNow)?
                     float(sensorNow-imu.time_us)*0.001f:-1.0f;
      debugGyro=imu_ok?imu.gz:0.0f;
      const bool tofFresh=tof.valid && tof.time_us>0 &&
            tof.time_us<=sensorNow && sensorNow-tof.time_us<=180000ULL &&
            isfinite(tof.meters) && tof.meters>=0.001f && tof.meters<=0.100f;
      debugClearance=tofFresh?tof.meters:-1.0f;
    }


    if (enabledNow &&
        headingGuard.reason() == 5 &&
        safeGuardReset &&
        imu_ok &&
        fabsf(debugGyro) < 0.03f) {


        ++headingGuardIdleTicks;

        if (headingGuardIdleTicks >= 25) {

            headingGuard.reset();
            yawCtl.inactive();

            yawCorrection = 0.0f;
            headingGuardIdleTicks = 0;

            ++headingGuardResetCount;

            printf(
                "HEADING_AUTO_RECOVER: "
                "reason=5 cleared at safe IDLE; "
                "heading remains enabled\n"
            );
        }

    } else {
        headingGuardIdleTicks = 0;
    }


    if(!calibrated||!ENABLE_MOTOR_OUTPUT){debugInterlock=MotorInterlock::OUTPUT_DISABLED;
      motors(0,0);targetL=targetR=0;goto report;}
    if(state==State::FAULT){motors(0,0);targetL=targetR=0;goto report;}
    debugSafeSpeed=-1.0f; // No ToF speed cap, ever.
    // Legacy diagnostic flags: 1=IMU unavailable; 2=ToF not observed.
    // Neither flag is a VL6180X movement interlock.
    debugSensorFlags=!imu_ok?1.0f:(debugClearance<0.0f?2.0f:0.0f);
    yawRefDeg=yawCtl.targetRadians()*180.0f/PI_F;
    headingErrorDeg=YawOuter::wrap(yawCtl.targetRadians()-yawCtl.yawRadians())*180.0f/PI_F;
    yawMode=float(yawCtl.mode());
    yawRateCorr=yawHeadingCorr=0.0f;
    yawRequested=w;
    yawEffective=w;
    if(!cmdFresh){
      debugInterlock=bad?MotorInterlock::CMD_INVALID:(ever?MotorInterlock::CMD_EXPIRED:MotorInterlock::WAIT_CMD);
      if(state==State::ACTIVE||state==State::LAUNCH||state==State::DECEL){
        if(ever&&!bad)++watchdogStops;
        printf("STOP_EVENT cmd_unfresh age=%llu ms bad=%d state=%d\n",
          (unsigned long long)(ageUs/1000ULL),(int)bad,(int)state);
        motorStop();
      }
      yawCtl.inactive();yawCorrection=0.0f;yawMode=0.0f;
      targetL=targetR=0;goto report;
    }
    {
      // BENCH mode is a separate, time-limited motor diagnostic.
      if(benchMode && (fabsf(v)>BENCH_MAX_LINEAR_MPS || fabsf(w)>BENCH_MAX_ANGULAR_RADPS)){
          debugInterlock=MotorInterlock::CMD_LIMIT;motorStop();goto report;
      }
      if((fabsf(v)>0.001f||fabsf(w)>0.001f) && !imu_ok && !benchMode && IMU_REQUIRED_FOR_MOTION){
          debugInterlock=MotorInterlock::IMU_UNAVAILABLE;
          if(state==State::ACTIVE||state==State::LAUNCH||state==State::DECEL)
              fault("IMU_LOST_DURING_MOTION");
          else motorStop();
          goto report;
      }
      const bool geometricStraight=fabsf(w)<0.001f && fabsf(v)>0.001f;
      // Heading controller changes angular command, never wheel PID/PWM.
      // Gyro observation above continues even while IDLE/LAUNCH/DECEL.
      // Independent of LAUNCH / ACTIVE: latch at first fresh non-zero Twist.
      // Still NEVER actuates motors in IDLE without a valid motion command.
      const bool headingOwnsSteering=!benchMode && enabledNow && imu_ok && !headingGuard.blocked();
      // Merely enabling heading does not suppress encoder straight sync.
      steeringAuthority=benchMode?2.0f:0.0f;
      if(headingOwnsSteering &&
         (fabsf(v)>0.001f||fabsf(w)>0.001f)){
          // Encoder translation + gyro heading provide a local dead-reckoned
          // trajectory diagnostic. Neither GPS nor map pose is claimed here.
          yawCtl.observeTravel(float(0.5*(dl+dr)));
          const auto y=yawCtl.update(v,w,dt,true,LOCAL_CROSSTRACK_ENABLED);
          headingGuard.update(v,w,debugGyro,filteredWheelOmega,y.heading_err);
          pathXRef=y.x_ref_m;pathYRef=y.y_ref_m;
          pathXEst=y.x_est_m;pathYEst=y.y_est_m;
          pathCte=y.cross_track_m;pathAlong=y.along_track_m;
          pathKappa=y.curvature_inv_m;pathRadius=y.radius_m;
          pathCorrection=y.path_corr;
          // When feedback is contradicted by wheel kinematics or grossly
          // divergent, fall back to the original cmd_vel WITHOUT heading steer.
          // Do not latch motor FAULT: odometry/encoder PID can still run.
          if(!headingGuard.blocked()) {
            steeringAuthority=y.active?1.0f:0.0f;
            w=y.omega_out;
            yawCorrection=y.correction;
            headingSlewLimited=y.slew_limited?1.0f:0.0f;
            const auto wheelDelta=YawOuter::wheelRpmDelta(yawCorrection,trackM,
                radiusM*LEFT_RADIUS_FACTOR,radiusM*RIGHT_RADIUS_FACTOR);
            headingDeltaRpm=wheelDelta.right-wheelDelta.left;
          } else {
            yawCtl.inactive();
            yawCorrection=0.0f;steeringAuthority=0.0f;
            printf("HEADING_GUARD_BLOCKED reason=%d; disable heading, stop, then re-enable\n",
                   headingGuard.reason());
          }
          headingErrorDeg=y.heading_err*180.0f/PI_F;
          yawRefDeg=y.heading_ref_rad*180.0f/PI_F;
          yawRateCorr=y.rate_corr;
          yawHeadingCorr=y.heading_corr;
          yawMode=headingGuard.blocked()?float(headingGuard.reason()):float(y.mode);
          yawEffective=w;
          headingP=headingGuard.blocked()?0.0f:y.p_term;
          headingI=headingGuard.blocked()?0.0f:y.i_term;
          headingD=headingGuard.blocked()?0.0f:y.d_term;
          headingActive=(!headingGuard.blocked() && y.active)?1.0f:0.0f;
          headingSaturated=(!headingGuard.blocked() && y.saturated)?1.0f:0.0f;
      }else{
          yawCtl.inactive();
          yawCorrection=0.0f;
          yawMode=headingGuard.blocked()?float(headingGuard.reason()):(imu_ok?0.0f:3.0f);
          pathCorrection=0.0f;
      }
      // Re-anchor encoder trim on transitions between yaw feedback and
      // wheel-distance synchronization; otherwise stale error causes RPM steps.
      if(!benchMode && previousSteeringAuthority!=steeringAuthority &&
         (previousSteeringAuthority==1.0f || steeringAuthority==1.0f)){
        syncDistL=accumL;syncDistR=accumR;
      }
      double wantL=0,wantR=0;
      float setL=0,setR=0;
      ControllerMath::differential_rpms(v,w,trackM,radiusM*LEFT_RADIUS_FACTOR,
                                       radiusM*RIGHT_RADIUS_FACTOR,setL,setR);
      wantL=setL;wantR=setR;
      bool allowed=isfinite(wantL)&&isfinite(wantR)&&fabs(wantL)<=MAX_WHEEL_RPM&&fabs(wantR)<=MAX_WHEEL_RPM&&
        (ENABLE_REVERSE||(wantL>=-0.01&&wantR>=-0.01));
      if(!allowed){debugInterlock=MotorInterlock::CMD_LIMIT;motorStop();targetL=targetR=0;goto report;}
      float newL=fabs(wantL)<0.01?0:(float)wantL;
      float newR=fabs(wantR)<0.01?0:(float)wantR;
      const bool requestedStop=(newL==0&&newR==0);
      targetL=newL;targetR=newR;
      if(!ENABLE_MOTOR_OUTPUT){state=State::DISARMED;goto report;}
      if(state==State::DISARMED)goto report;
      // Do not energize opposite direction until the previous wheel has actually
      // almost stopped (encoder-filtered signed speed, not just PWM sign).
      const bool reversal=
        (newL>STOP_RPM&&filteredL< -REVERSAL_STILL_RPM)||
        (newL< -STOP_RPM&&filteredL>REVERSAL_STILL_RPM)||
        (newR>STOP_RPM&&filteredR< -REVERSAL_STILL_RPM)||
        (newR< -STOP_RPM&&filteredR>REVERSAL_STILL_RPM);
      if(reversal){
        if(state==State::LAUNCH){motorStop();goto report;}
        if(state==State::ACTIVE)state=State::DECEL;
      }
      if(requestedStop){
        if(state==State::LAUNCH){motorStop();goto report;}
        if(state==State::ACTIVE)state=State::DECEL;
      }else if(state==State::DECEL&&!reversal){state=State::ACTIVE;}
      // Apply correction by physical distance instead of matching raw counts;
      // equal RPM alone is not enough if the effective radii differ.
      const bool straightCmd=(geometricStraight && !requestedStop && !reversal && newL*newR>0);
      const int desiredSign=straightCmd?(newL>0?1:-1):0;
      const bool launchEligible=!benchMode&&!requestedStop&&!reversal&&
          fabsf(newL)>=LAUNCH_ELIGIBLE_MIN_RPM&&
          fabsf(newR)>=LAUNCH_ELIGIBLE_MIN_RPM &&
          (straightCmd || fabsf(newL+newR)<0.1f); // spin in place
      if(state==State::IDLE&&!requestedStop){
        if(fabsf(filteredL)>REVERSAL_STILL_RPM||fabsf(filteredR)>REVERSAL_STILL_RPM){
          motors(0,0);goto report; // wheel still coasting; do not kick
        }
        state=launchEligible?State::LAUNCH:State::ACTIVE;
        launchSignL=newL>0?1:-1;launchSignR=newR>0?1:-1;
        launchUs=now;launchDuty=0;movingBoth=0;
        syncDistL=accumL;syncDistR=accumR;
        oldStraight=straightCmd;straightSign=desiredSign;
        rampL=rampR=0;resetPid();stationaryTimeoutTicks=0;
      }
      if(state!=State::DECEL&&(straightCmd!=oldStraight||desiredSign!=straightSign)){
        oldStraight=straightCmd;straightSign=desiredSign;
        syncDistL=accumL;syncDistR=accumR;
      }
      if(state==State::LAUNCH){
        launchDuty=approach(launchDuty,LAUNCH_PWM,LAUNCH_RISE_PER_S*dt);
        float trim=0;
        if(straightCmd && steeringAuthority==0.0f){
          trim=ControllerMath::straight_trim(
            float(accumL-syncDistL),float(accumR-syncDistR),straightSign,
            SYNC_RPM_PER_M*0.40f,LAUNCH_SYNC_MAX_PWM);
        }
        // LAUNCH steering uses the *change* in signed wheel RPM caused by yaw
        // feedback. Works for forward, reverse AND spin-in-place (the old
        // |newL|-|newR| calculation cancelled the correction during a spin).
        float launchSteerL=0,launchSteerR=0;
        if(steeringAuthority==1.0f && fabsf(yawCorrection)>0.0f){
          const float perRpm=2.0f; // conservative short startup duty / wheel-RPM trim
          const auto delta=YawOuter::wheelRpmDelta(yawCorrection,trackM,
              radiusM*LEFT_RADIUS_FACTOR,radiusM*RIGHT_RADIUS_FACTOR);
          launchSteerL=clampf(launchSignL*delta.left*perRpm,-6.0f,6.0f);
          launchSteerR=clampf(launchSignR*delta.right*perRpm,-6.0f,6.0f);
        }
        motors(launchSignL*roundf(clampf(launchDuty-trim+launchSteerL,0,PWM_MAX)),
               launchSignR*roundf(clampf(launchDuty+trim+launchSteerR,0,PWM_MAX)));
        if(rpmRawL*launchSignL>=LAUNCH_MOVING_RPM&&
           rpmRawR*launchSignR>=LAUNCH_MOVING_RPM)movingBoth++;
        else movingBoth=0;
        uint64_t elapsed=(now-launchUs)/1000ULL;
        if(elapsed>=LAUNCH_MAX_MS&&movingBoth<LAUNCH_MOVING_SAMPLES){
          printf("LAUNCH_DIAG targetL=%.1f targetR=%.1f rawL=%.1f rawR=%.1f "
                 "filteredL=%.1f filteredR=%.1f pwmL=%.0f pwmR=%.0f "
                 "elapsed=%llu ms: check 12V, EN and encoder A/B before resetting MCU\n",
                 newL,newR,rpmRawL,rpmRawR,filteredL,filteredR,pwmL,pwmR,
                 (unsigned long long)elapsed);
          fault("START_ENCODER_TIMEOUT");goto report;
        }
        if(elapsed>=LAUNCH_MIN_MS&&movingBoth>=LAUNCH_MOVING_SAMPLES){
          rampL=launchSignL*clampf(fmaxf(fabsf(rpmRawL),fabsf(filteredL)),0,fabsf(newL));
          rampR=launchSignR*clampf(fmaxf(fabsf(rpmRawR),fabsf(filteredR)),0,fabsf(newR));
          resetPid();state=State::ACTIVE;
        }
      }else if(state==State::ACTIVE||state==State::DECEL){
        // Reversal is treated as a commanded deceleration to zero, never a
        // direct transition from forward force to reverse force.
        const bool rampToZero=requestedStop||reversal;
        const float change=(rampToZero?RPM_DECEL_PER_S:RPM_ACCEL_PER_S)*dt;
        rampL=approach(rampL,rampToZero?0:newL,change);
        rampR=approach(rampR,rampToZero?0:newR,change);
        if(rampToZero&&fabsf(rampL)<STOP_RPM&&fabsf(rampR)<STOP_RPM){
          motorStop();goto report;
        }
        float refL=rampL,refR=rampR;
        const bool straightRunning=straightCmd&&state==State::ACTIVE&&
            fabsf(rampL)>15&&fabsf(rampR)>15&&
            fabsf(rampL-newL)<1.0f&&fabsf(rampR-newR)<1.0f;
        // IMU loop owns steering in ACTIVE. Keep launch-stage encoder synchronization,
        // but never run encoder straight_trim concurrently with gyro correction.
        if(straightRunning && steeringAuthority==0.0f){
          const float trim=ControllerMath::straight_trim(
            float(accumL-syncDistL),float(accumR-syncDistR),straightSign,
            SYNC_RPM_PER_M,SYNC_MAX_RPM);
          refL=straightSign*clampf(fabsf(rampL)-trim,0,MAX_WHEEL_RPM);
          refR=straightSign*clampf(fabsf(rampR)+trim,0,MAX_WHEEL_RPM);
        }
        // V1 disabled inner-wheel I and D whenever the outer loop updated
        // targets by even 0.1RPM. That prevented correcting persistent wheel
        // asymmetry (the dominant error in the supplied log).
        // Enable them separately for each wheel, after launch, only when the
        // ramp is nearly caught up and measured speed is in the valid region.
        const bool feedbackReady=!rampToZero&&state==State::ACTIVE&&
          now-launchUs>=350000ULL;
        const bool settledL=feedbackReady&&fabsf(newL-rampL)<3.0f&&
          fabsf(refL)>15.0f && fabsf(filteredL)>4.0f &&
          fabsf(refL-filteredL)<15.0f;
        const bool settledR=feedbackReady&&fabsf(newR-rampR)<3.0f&&
          fabsf(refR)>15.0f && fabsf(filteredR)>4.0f &&
          fabsf(refR-filteredR)<15.0f;
        const float ffL=ffDuty(refL,true),ffR=ffDuty(refR,false);
        debugRefL=refL;debugRefR=refR;debugFfL=ffL;debugFfR=ffR;
        float demandL=refL==0?0:copysignf(clampf(
           ffL+pidDemand(refL,filteredL,dt,ffL,gl,pl,settledL),0,PWM_MAX),refL);
        float demandR=refR==0?0:copysignf(clampf(
           ffR+pidDemand(refR,filteredR,dt,ffR,gr,pr,settledR),0,PWM_MAX),refR);
        if(refL==0)pl=PID{};
        if(refR==0)pr=PID{};
        // Static-friction startup assistance only for an initial brief interval,
        // never for normal running, DECEL or opposite-direction wheel motion.
        if(state==State::ACTIVE&&now-launchUs<(uint64_t)STARTUP_BOOST_MAX_MS*1000ULL){
          if(fabsf(refL)>STOP_RPM&&fabsf(filteredL)<STARTUP_MOVING_RPM)
            demandL=copysignf(fmaxf(fabsf(demandL),STARTUP_MIN_PWM),refL);
          if(fabsf(refR)>STOP_RPM&&fabsf(filteredR)<STARTUP_MOVING_RPM)
            demandR=copysignf(fmaxf(fabsf(demandR),STARTUP_MIN_PWM),refR);
        }
        debugDemandL=demandL;debugDemandR=demandR;
        float slew=PWM_SLEW_PER_S*dt;
        float nextL=approach(pwmL,demandL,slew),nextR=approach(pwmR,demandR,slew);
        if((pwmL>0&&nextL<0)||(pwmL<0&&nextL>0))nextL=0;
        if((pwmR>0&&nextR<0)||(pwmR<0&&nextR>0))nextR=0;
        motors(dutyQuantize(nextL,ditherL),dutyQuantize(nextR,ditherR));
        if(!rampToZero&&(fabsf(rampL)>=25||fabsf(rampR)>=25)&&
           fabsf(filteredL)<1&&fabsf(filteredR)<1)stationaryTimeoutTicks++;
        else stationaryTimeoutTicks=0;
        if(stationaryTimeoutTicks>60){fault("ENCODER_STALL_BOTH");goto report;}
      }
    }
report:
    {WheelReport r;
      r.targetL=targetL;r.targetR=targetR;r.rpmL=filteredL;r.rpmR=filteredR;
      r.pwmL=pwmL;r.pwmR=pwmR;r.distanceL=accumL;r.distanceR=accumR;
      r.headingDeg=pose.yaw*180.0/Kinematics::PI_D;r.state=(float)(int)state;
      r.x=pose.x;r.y=pose.y;r.yaw=pose.yaw;r.linear=velocity;r.angular=omega;
      r.jointPosL=accumL/(radiusM*LEFT_RADIUS_FACTOR);
      r.jointPosR=accumR/(radiusM*RIGHT_RADIUS_FACTOR);
      r.jointVelL=filteredL*2*Kinematics::PI_D/60;
      r.jointVelR=filteredR*2*Kinematics::PI_D/60;
      r.refL=debugRefL;r.refR=debugRefR;r.ffL=debugFfL;r.ffR=debugFfR;
      r.pL=pl.lastP;r.iL=pl.integral;r.dL=pl.lastD;
      r.pR=pr.lastP;r.iR=pr.integral;r.dR=pr.lastD;
      r.demandL=debugDemandL;r.demandR=debugDemandR;
      r.watchdogStopCount=watchdogStops;
      r.controlOverrunCount=controlOverruns;
      r.yawCorrection=yawCorrection;r.gyroZ=debugGyro;
      r.imuYawDeg=yawCtl.yawRadians()*180.0f/PI_F;
      r.headingTargetDeg=yawRefDeg;
      r.headingErrorDeg=headingErrorDeg;
      r.yawRateCorrection=yawRateCorr;
      r.yawHeadingCorrection=yawHeadingCorr;
      r.yawRequested=yawRequested;
      r.yawEffective=yawEffective;
      r.yawMode=yawMode;
      r.imuFresh=imu_ok?1.0f:0.0f;
      r.imuAgeMs=imuSampleAgeMs;
      r.headingPidEnabled=headingDebugEnabled;r.headingPidActive=headingActive;
      r.headingKp=yawCtl.gains().kp;r.headingKi=yawCtl.gains().ki;
      r.headingKd=yawCtl.gains().kd;
      r.headingP=headingP;r.headingI=headingI;r.headingD=headingD;
      r.headingSaturated=headingSaturated;
      r.headingSlewLimited=headingSlewLimited;
      r.steeringAuthority=steeringAuthority;
      r.headingDeltaRpm=headingDeltaRpm;
      r.encoderYawRate=debugWheelOmega;
      r.headingGuardReason=float(headingGuard.reason());
      r.imuInitialized=imuInitialized;r.imuCalProgress=imuCalProgress;
      r.imuErrors=imuErrors;r.imuSeq=imuSeq;
      r.imuRawZCounts=imuRawZ;r.imuBiasZCounts=imuBiasZ;r.imuWho=imuWho;
      r.imuAccelNorm=imuAccelNorm;r.imuCalReject=imuCalReject;
      r.imuCalRestarts=imuCalRestarts;
      portENTER_CRITICAL(&headingMux);
      r.headingSetStatus=headingSetResult;r.headingSetSeq=headingSetSeq;
      r.headingEnableRxSeq=float(headingEnableRxSeq);
      r.headingEnableAppliedSeq=float(headingEnableAppliedSeq);
      r.headingGuardResetCount=float(headingGuardResetCount);
      r.headingEnableApplyStatus=float(headingEnableApplyStatus);
      portEXIT_CRITICAL(&headingMux);
      r.pathXRef=pathXRef;r.pathYRef=pathYRef;
      r.pathXEst=pathXEst;r.pathYEst=pathYEst;
      r.pathCrossTrack=pathCte;r.pathAlongTrack=pathAlong;
      r.pathCurvature=pathKappa;r.pathRadius=pathRadius;
      r.pathCorrection=pathCorrection;

      r.safeSpeedLimit=debugSafeSpeed;r.obstacleClearance=debugClearance;
      r.sensorFlags=debugSensorFlags;
      r.interlockReason=debugInterlock;
      float remaining=0;
      r.benchArmed=benchArmedNow(now,remaining)?1.0f:0.0f;
      r.benchRemainingMs=remaining;
      r.cmdFresh=cmdFresh?1.0f:0.0f;
      r.sampleUs=now;
      portENTER_CRITICAL(&reportMux);published=r;portEXIT_CRITICAL(&reportMux);
    }
  }
}
void configureEnc(){
  gpio_config_t c={};c.pin_bit_mask=(1ULL<<L_ENC_A)|(1ULL<<L_ENC_B)|(1ULL<<R_ENC_A)|(1ULL<<R_ENC_B);
  c.mode=GPIO_MODE_INPUT;c.pull_up_en=GPIO_PULLUP_DISABLE;c.pull_down_en=GPIO_PULLDOWN_DISABLE;
  c.intr_type=GPIO_INTR_ANYEDGE;ESP_ERROR_CHECK(gpio_config(&c));
  esp_err_t e=gpio_install_isr_service(0);if(e!=ESP_OK&&e!=ESP_ERR_INVALID_STATE)ESP_ERROR_CHECK(e);
  ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)L_ENC_A,leftA,nullptr));
  ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)L_ENC_B,leftB,nullptr));
  ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)R_ENC_A,rightA,nullptr));
  ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)R_ENC_B,rightB,nullptr));
}
void configureMotors(){
  gpio_config_t c={};c.pin_bit_mask=(1ULL<<L_EN)|(1ULL<<R_EN);
  c.mode=GPIO_MODE_OUTPUT;ESP_ERROR_CHECK(gpio_config(&c));
  gpio_set_level((gpio_num_t)L_EN,0);gpio_set_level((gpio_num_t)R_EN,0);
  ledc_timer_config_t timer={};timer.speed_mode=MODE;timer.timer_num=TIMER;
  timer.duty_resolution=LEDC_TIMER_8_BIT;timer.freq_hz=PWM_HZ;timer.clk_cfg=LEDC_AUTO_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&timer));
  const struct {int pin;ledc_channel_t channel;} map[]={
   {L_FORWARD_PIN,L_F},{L_REVERSE_PIN,L_R},{R_FORWARD_PIN,R_F},{R_REVERSE_PIN,R_R}};
  for(auto& it:map){ledc_channel_config_t cfg={};cfg.gpio_num=it.pin;cfg.speed_mode=MODE;
    cfg.channel=it.channel;cfg.intr_type=LEDC_INTR_DISABLE;cfg.timer_sel=TIMER;
    cfg.duty=0;cfg.hpoint=0;ESP_ERROR_CHECK(ledc_channel_config(&cfg));}
  motors(0,0);
}
} // namespace
void robot_drive_start(){
  esp_err_t err=nvs_flash_init();
  if(err==ESP_ERR_NVS_NO_FREE_PAGES||err==ESP_ERR_NVS_NEW_VERSION_FOUND){
    printf("NVS cannot be initialized safely; refusing motor start (do not erase PID automatically)\n");abort();
  }
  ESP_ERROR_CHECK(err);loadCalibration();ffLoadNvs();configureMotors();configureEnc();
  state=calibrated&&ENABLE_MOTOR_OUTPUT?State::IDLE:State::DISARMED;
  if(xTaskCreatePinnedToCore(controlTask,"wheel_50hz",8192,nullptr,6,nullptr,1)!=pdPASS){motors(0,0);abort();}
}
void robot_drive_cmd_vel(float v,float w){
  bool ok=isfinite(v)&&isfinite(w)&&fabsf(v)<=0.48f&&fabsf(w)<=2.0f;
  portENTER_CRITICAL(&inputMux);
  uint64_t received=esp_timer_get_time();
  if(previousCmdUs>0&&received>=previousCmdUs){
    uint64_t gap=(received-previousCmdUs)/1000ULL;
    if(gap>maxCmdGapMs)maxCmdGapMs=(uint32_t)(gap>UINT32_MAX?UINT32_MAX:gap);
  }
  previousCmdUs=received;
  if(!ok)++invalidCommands;
  cmdV=ok?v:0;cmdW=ok?w:0;cmdBad=!ok;cmdEver=true;cmdLastUs=received;
  portEXIT_CRITICAL(&inputMux);
}
void robot_drive_get_report(WheelReport&r){portENTER_CRITICAL(&reportMux);r=published;portEXIT_CRITICAL(&reportMux);}
void robot_drive_pid_request(const float data[6],bool length){
  bool ok=length&&data;
  if(ok){for(int i=0;i<6;i++)if(!isfinite(data[i]))ok=false;}
  if(ok){Gains a{data[0],data[1],data[2]},b{data[3],data[4],data[5]};ok=validGain(a)&&validGain(b);}
  portENTER_CRITICAL(&pidMux);pidSequence++;
  if(ok){memcpy(requestedPid,data,6*sizeof(float));pidPending=true;pidResult=10;}
  else{pidPending=false;pidResult=-1;}
  portEXIT_CRITICAL(&pidMux);
}
void robot_drive_pid_status(float out[8]){
  portENTER_CRITICAL(&pidMux);for(int i=0;i<6;i++)out[i]=activePid[i];out[6]=pidResult;out[7]=pidSequence;portEXIT_CRITICAL(&pidMux);
}

void robot_drive_link_debug(float out[6]){
  uint64_t rx=0;uint32_t gap=0,invalid=0;
  portENTER_CRITICAL(&inputMux);
  rx=cmdLastUs;gap=maxCmdGapMs;invalid=invalidCommands;
  portEXIT_CRITICAL(&inputMux);
  uint64_t now=esp_timer_get_time();
  out[0]=(float)(rx&&now>=rx?(now-rx)/1000ULL:0ULL);
  out[1]=(float)gap;
  WheelReport snap;robot_drive_get_report(snap);
  out[2]=(float)snap.watchdogStopCount;
  out[3]=(float)invalid;
  out[4]=(float)snap.controlOverrunCount;
  out[5]=snap.state;
}

void robot_drive_bench_request(bool arm){
  // The ROS command does not prove that the wheels are raised. The host tool
  // MUST demand explicit human confirmation, and the firmware limits duration.
  if(!arm){
    benchClear();robot_drive_cmd_vel(0,0);
    printf("BENCH_DISARMED: forced zero /cmd_vel\n");return;
  }
  if(!RobotConfig::BENCH_DIAGNOSTICS_AVAILABLE){printf("BENCH_REFUSED: disabled in firmware\n");return;}
  float v,w;bool ever;
  portENTER_CRITICAL(&inputMux);v=cmdV;w=cmdW;ever=cmdEver;portEXIT_CRITICAL(&inputMux);
  if(state!=State::IDLE || fabsf(filteredL)>2.0f || fabsf(filteredR)>2.0f ||
     (ever&&(fabsf(v)>0.001f||fabsf(w)>0.001f))){
    ++benchRefused;printf("BENCH_REFUSED: IDLE and zero cmd required; state=%d\n",int(state));return;
  }
  const uint64_t until=esp_timer_get_time()+uint64_t(BENCH_ARM_TTL_MS)*1000ULL;
  portENTER_CRITICAL(&benchMux);benchArmed=true;benchUntilUs=until;portEXIT_CRITICAL(&benchMux);
  printf("BENCH_ARMED: raised wheels ONLY, cap v=%.2f w=%.2f TTL=%dms\n",
          BENCH_MAX_LINEAR_MPS,BENCH_MAX_ANGULAR_RADPS,BENCH_ARM_TTL_MS);
}

void robot_drive_heading_enable_request(bool enabled){
  portENTER_CRITICAL(&headingMux);
  headingEnabled=enabled;
  ++headingEnableRxSeq;
  if(!enabled)headingResetRequested=true;
  const uint32_t seq=headingEnableRxSeq;
  portEXIT_CRITICAL(&headingMux);
  printf("HEADING_V12_REQUEST seq=%lu requested=%d\n",
         (unsigned long)seq,int(enabled));
}
void robot_drive_heading_pid_request(const float data[3],bool length){
  bool ok=length&&data;
  if(ok) for(int i=0;i<3;i++)if(!isfinite(data[i]))ok=false;
  if(ok)ok=data[0]>=0&&data[0]<=3.0f&&data[1]>=0&&data[1]<=1.5f&&
          data[2]>=0&&data[2]<=1.5f;
  portENTER_CRITICAL(&headingMux);
  headingSetSeq+=1.0f;
  if(ok){memcpy(headingPidPending,data,3*sizeof(float));
         headingGainPending=true;headingSetResult=10;}
  else{headingGainPending=false;headingSetResult=-1;}
  portEXIT_CRITICAL(&headingMux);
}

// V10 feedforward tuners: ros callbacks only, after IDLE/still validation.
void robot_drive_ff_point_request(const float* data,bool length){
  if(!length||!data || !isfinite(data[0])||!isfinite(data[1])||!isfinite(data[2])||!isfinite(data[3])||
     (data[0]!=0.0f&&data[0]!=1.0f)||(data[1]!=1.0f&&data[1]!=-1.0f)){
    ffSetResult(-1,false);return;
  }
  if(!ffSafeToEdit()){ffSetResult(-2,false);return;}
  portENTER_CRITICAL(&ffMux);
  bool good=FeedforwardV10::updatePoint(ffConfig,(int)data[0],(int)data[1],data[2],data[3]);
  portEXIT_CRITICAL(&ffMux);
  ffSetResult(good?1:-1,good);
  if(good)printf("FF_V10 POINT wheel=%d direction=%d rpm=%.1f pwm=%.1f RAM\n",
                 (int)data[0],(int)data[1],data[2],data[3]);
}
void robot_drive_ff_scale_request(const float* data,bool length){
  if(!length||!data||!isfinite(data[0])||!isfinite(data[1])||!isfinite(data[2])||
     (data[0]!=0.0f&&data[0]!=1.0f)||(data[1]!=1.0f&&data[1]!=-1.0f)){
    ffSetResult(-1,false);return;
  }
  if(!ffSafeToEdit()){ffSetResult(-2,false);return;}
  portENTER_CRITICAL(&ffMux);
  bool good=FeedforwardV10::updateScale(ffConfig,(int)data[0],(int)data[1],data[2]);
  portEXIT_CRITICAL(&ffMux);
  ffSetResult(good?1:-1,good);
  if(good)printf("FF_V10 SCALE wheel=%d direction=%d scale=%.3f RAM\n",
                 (int)data[0],(int)data[1],data[2]);
}
void robot_drive_ff_reset_request(bool reset){
  if(!reset)return;
  if(!ffSafeToEdit()){ffSetResult(-2,false);return;}
  portENTER_CRITICAL(&ffMux);ffConfig=FeedforwardV10::defaults();portEXIT_CRITICAL(&ffMux);
  ffSetResult(3,true);
  printf("FF_V10 defaults restored to RAM; save explicitly to persist\n");
}
void robot_drive_ff_save_request(bool save){
  if(!save)return;
  if(!ffSafeToEdit()){ffSetResult(-2,false);return;}
  FeedforwardV10::Calibration current;
  portENTER_CRITICAL(&ffMux);current=ffConfig;portEXIT_CRITICAL(&ffMux);
  if(!FeedforwardV10::valid(current)){ffSetResult(-1,false);return;}
  const auto record=FeedforwardV10::pack(current);
  nvs_handle_t h;
  esp_err_t rc=nvs_open(FF_NVS_NAMESPACE,NVS_READWRITE,&h);
  if(rc==ESP_OK){
    rc=nvs_set_blob(h,FF_NVS_KEY,&record,sizeof(record));
    if(rc==ESP_OK)rc=nvs_commit(h);
    nvs_close(h);
  }
  ffSetResult(rc==ESP_OK?2:-3,false);
  if(rc==ESP_OK){portENTER_CRITICAL(&ffMux);ffStored=1;portEXIT_CRITICAL(&ffMux);}
  printf("FF_V10 NVS save rc=%s\n",esp_err_to_name(rc));
}
void robot_drive_ff_status(float out[43]){
  portENTER_CRITICAL(&ffMux);
  out[0]=ffResult;out[1]=ffSequence;out[2]=ffStored;
  for(int m=0;m<4;++m)out[3+m]=ffConfig.scale[m];
  for(int m=0;m<4;++m)for(int i=0;i<FeedforwardV10::COUNT;++i)
    out[7+m*FeedforwardV10::COUNT+i]=ffConfig.pwm[m][i];
  portEXIT_CRITICAL(&ffMux);
}
