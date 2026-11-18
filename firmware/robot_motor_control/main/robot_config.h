#pragma once
#include <stdint.h>
// Units: SI (m, rad, s) except explicit PWM/RPM and NVS _mm keys.
// Preserves the stepA PID-WiFi low-level motor configuration (source ZIP supplied by user).
namespace RobotConfig {
// Fixed wiring: shared I2C bus, no rewiring of approved project.
constexpr int I2C_SDA_GPIO=15, I2C_SCL_GPIO=16;
constexpr int IMU_INT_GPIO=17, TOF_XSHUT_GPIO=47, TOF_INT_GPIO=21;
// VL6180X is an OBSERVATION-ONLY sensor. It has NO direct motor control,
// no speed limiting, no 6-cm interlock; /front_range publishes valid data.
// Future LiDAR / Nav2 collision monitor will handle obstacle avoidance.
// Keep GPIO wiring, I2C address and range driver unchanged.
// IMU yaw-rate correction is an OPTIONAL outer loop to the unchanged FF/PID.
// On missing/stale gyro, stop applying correction and let encoder/PID run.
// This is for SUPERVISED development, not certified autonomous driving.
constexpr bool YAW_OUTER_ENABLED=true; // compile-time capability; runtime /robot/heading_enable
// FAIL-SAFE default: commissioning required; do not run the unverified outer loop on boot.
constexpr bool HEADING_DEFAULT_ENABLED=false;
// +1 when MPU +Z points UP and CCW/yaw-left increases gyro Z. Verify against encoder.
// For a physically inverted MPU, change to -1 ONLY after a stationary rotation test.
constexpr float IMU_GYRO_Z_SIGN=1.0f;
// Guard only the heading steering correction; low-level motor PID remains active.
constexpr float HEADING_GUARD_MAX_ERROR_RAD=0.349066f; // 20 deg
constexpr float HEADING_GUARD_MIN_RATE_RADPS=0.050f;
constexpr int HEADING_GUARD_SIGN_SAMPLES=8; // ~160ms at 50Hz
constexpr int HEADING_GUARD_EXTREME_SAMPLES=5;

// Experimental LOCAL XY correction based on wheel distance + relative gyro.
// Keep false until heading and encoder accuracy are characterized.
// Nav2 path tracking in map/odom is a separate, higher-level loop.
constexpr bool LOCAL_CROSSTRACK_ENABLED=false;
constexpr bool IMU_REQUIRED_FOR_MOTION=false;

// BENCH diagnostics: only when wheels are securely raised.
constexpr bool BENCH_DIAGNOSTICS_AVAILABLE=true;
constexpr int BENCH_ARM_TTL_MS=9000;
constexpr float BENCH_MAX_LINEAR_MPS=0.10f;
constexpr float BENCH_MAX_ANGULAR_RADPS=0.30f;
constexpr int L_RPWM=4, L_LPWM=5, L_EN=6, L_ENC_A=10, L_ENC_B=11;
constexpr int R_RPWM=40, R_LPWM=41, R_EN=42, R_ENC_A=1, R_ENC_B=2;
constexpr int L_FORWARD_PIN=L_LPWM, L_REVERSE_PIN=L_RPWM;
constexpr int R_FORWARD_PIN=R_RPWM, R_REVERSE_PIN=R_LPWM;
constexpr float PI_F=3.14159265358979323846f;
constexpr float ENCODER_CPR=3960.0f; // normalized quadrature counts / wheel revolution
constexpr float RADIUS_M=0.044f;     // latest VERIFIED value from the user's ESP-IDF log
constexpr float TRACK_M=0.195f;      // latest VERIFIED value from the user's ESP-IDF log
// These are initial effective dimensions. Calibrate against measured floor travel.

constexpr bool ENABLE_MOTOR_OUTPUT=true; // DANGER: bench test raised wheels before supplying motor power
constexpr bool ENABLE_REVERSE=true; // Verify encoder signs and motor directions on stands
// Goal control belongs to the ROS 2 host (host_ros2/go_to_pose.py), not firmware.
// odom->base_link TF is published by host_ros2/odom_tf_joint_bridge.py, not ESP32.
// Disable that bridge TF when robot_localization EKF publishes the same transform.

constexpr int PWM_HZ=10000, PWM_MAX=240, CONTROL_MS=20, DEBUG_MS=100, ODOM_MS=100;
// Publish telemetry without blocking command processing (best-effort for debug).
constexpr int ODOM_RELIABLE_ACK_MS=80;
// Bound diagnostic reliable-publish waits so they never monopolize the ROS callback loop.
// These two topics use RELIABLE so the user's ordinary `ros2 topic echo` works.
constexpr int DEBUG_RELIABLE_ACK_MS=25;
constexpr int LINK_DEBUG_MS=500;
constexpr int CMD_TIMEOUT_MS=800;
// No actual acceleration on losing commands: PWM must coast to 0 for safety.
constexpr float MAX_WHEEL_RPM=100.0f;
constexpr float RPM_ALPHA=0.35f, D_ALPHA=0.15f;
constexpr float PWM_SLEW_PER_S=600.0f, RPM_ACCEL_PER_S=120.0f, RPM_DECEL_PER_S=80.0f;
constexpr float I_LIMIT_PWM=20.0f, PID_TRIM_LIMIT_PWM=25.0f;
constexpr float LAUNCH_PWM=80.0f, LAUNCH_RISE_PER_S=500.0f;
// Give the real gearbox time to spin up before declaring an encoder fault.
// Still STOP and latch FAULT if measured motion is not confirmed.
constexpr int LAUNCH_MIN_MS=160, LAUNCH_MAX_MS=750, LAUNCH_MOVING_SAMPLES=3;
// The previous 12 RPM / 450 ms combination was unnecessarily strict at ~20 RPM commands.
constexpr float LAUNCH_MOVING_RPM=6.0f;
// Min speed for the encoder-verified LAUNCH state. Includes 0.5 rad/s spin (~10.6 RPM).
constexpr float LAUNCH_ELIGIBLE_MIN_RPM=8.0f;
// Startup count synchronization: capped asymmetric correction to common PWM.
constexpr float LAUNCH_SYNC_PWM_PER_COUNT=0.05f;
constexpr float LAUNCH_SYNC_MAX_PWM=6.0f;
// Running straight: encoder-count synchronizer, retained from user's sketch.
// Physical distance synchronizer: 0.012 RPM per encoder tick at R=44mm, CPR=3960
constexpr float SYNC_RPM_PER_M=172.0f, SYNC_MAX_RPM=2.0f;
// Effective rolling radius correction (calibrate from tape tests); 1.0 = geometry default.
constexpr float LEFT_RADIUS_FACTOR=1.0f, RIGHT_RADIUS_FACTOR=1.0f;
// Motor commands smaller than 20RPM are NOT covered by the measured FF table.
constexpr float STARTUP_MIN_PWM=54.0f;
constexpr float STARTUP_MOVING_RPM=3.0f;
constexpr int STARTUP_BOOST_MAX_MS=350;
constexpr float REVERSAL_STILL_RPM=3.0f;
// Keep user-requested PID+geometry as authoritative until deliberately switched on.
constexpr bool USE_NVS_PID=false;
constexpr bool USE_NVS_GEOMETRY=false;
constexpr float STOP_RPM=2.0f;
// Signed wheel directions: when reversing, FF is evaluated on |RPM|.
constexpr int FF_COUNT=9;
static const float FF_RPM[FF_COUNT]  ={20,30,40,50,60,70,80,90,100};
// FF V2 from the predecessor ESP-IDF robot_motor_control/robot_config.h.
// The uploaded stepA ZIP contains only 3 .cpp/.h source files, not robot_config.h.
static const float FF_LEFT[FF_COUNT]={55,73,98,119,144,164,187,218,237};
static const float FF_RIGHT[FF_COUNT]={54,70,94,112,132,156,180,200,223};
// Reference gains only, overwritten if full valid prior NVS PID exists.
constexpr float FALLBACK_KP_L=0.20f, FALLBACK_KI_L=0.075f, FALLBACK_KD_L=0.10f;
constexpr float FALLBACK_KP_R=0.30f, FALLBACK_KI_R=0.060f, FALLBACK_KD_R=0.10f;
// Reverse curves START as hypotheses (not measured!) — tune from reverse-wheel CSV plots.
static const float FF_REVERSE_LEFT[FF_COUNT]={55,73,98,119,144,164,187,218,237};
static const float FF_REVERSE_RIGHT[FF_COUNT]={54,70,94,112,132,156,180,200,223};
// Optional global per-direction FF scale, 1.0 = unmodified table.
constexpr float FF_SCALE_FWD_L=1.05f, FF_SCALE_FWD_R=1.06f;
constexpr float FF_SCALE_REV_L=1.06f, FF_SCALE_REV_R=1.08f;

}
