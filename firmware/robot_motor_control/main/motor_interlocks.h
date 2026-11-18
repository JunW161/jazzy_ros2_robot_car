#pragma once
// V8 reason codes. Retain in firmware and host tools; never reinterpret existing indices.
namespace MotorInterlock {
enum Code : int {
  NONE=0, WAIT_CMD=1, CMD_EXPIRED=2, CMD_INVALID=3, IMU_UNAVAILABLE=4,
  FRONT_UNKNOWN=5, FRONT_NEAR=6, MOTOR_FAULT=7, CMD_LIMIT=8,
  OUTPUT_DISABLED=9, BENCH_EXPIRED=10, START_ENCODER_TIMEOUT=11,
  ENCODER_STALL=12, CONTROL_OVERRUN=13
};
inline const char* name(int code) {
  switch(code){
    case NONE:return "NONE"; case WAIT_CMD:return "WAIT_CMD";
    case CMD_EXPIRED:return "CMD_EXPIRED"; case CMD_INVALID:return "CMD_INVALID";
    case IMU_UNAVAILABLE:return "IMU_UNAVAILABLE";
    case FRONT_UNKNOWN:return "FRONT_UNKNOWN_NO_LIDAR";
    case FRONT_NEAR:return "FRONT_NEAR_STOP";
    case MOTOR_FAULT:return "MOTOR_FAULT";case CMD_LIMIT:return "CMD_LIMIT";
    case OUTPUT_DISABLED:return "OUTPUT_DISABLED";case BENCH_EXPIRED:return "BENCH_EXPIRED";
    case START_ENCODER_TIMEOUT:return "START_ENCODER_TIMEOUT";
    case ENCODER_STALL:return "ENCODER_STALL";case CONTROL_OVERRUN:return "CONTROL_OVERRUN";
    default:return "UNKNOWN";
  }
}
} // namespace MotorInterlock
