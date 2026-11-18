#pragma once
#include <stdint.h>
struct ImuSample {
  float ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
  uint64_t time_us=0; uint32_t seq=0,errors=0; bool valid=false,calibrated=false;
  // Device status remains visible during initialization, calibration and I2C faults.
  bool initialized=false; uint16_t calibration_progress=0; uint8_t who=0;
  float raw_gz_counts=0,bias_gz_counts=0,accel_norm=0;
  uint8_t calibration_reject_reason=0; // bit0=gravity, bit1=gyro motion, bit2=noisy 150-sample window
  uint32_t calibration_restarts=0;
};
struct RangeReport {
  float meters=0; uint64_t time_us=0; uint32_t seq=0, errors=0;
  uint16_t model_id=0; uint8_t range_status=255; bool valid=false,initialized=false;
};
void robot_sensors_start();
void robot_imu_snapshot(ImuSample&);
void robot_range_snapshot(RangeReport&);
