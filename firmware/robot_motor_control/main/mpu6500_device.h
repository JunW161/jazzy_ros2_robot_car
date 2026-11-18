#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
namespace Mpu6500 {
struct Raw { float ax=0,ay=0,az=0; float gyro_counts[3]={}; };
esp_err_t init(); // WHO_AM_I must be 0x70
// One I2C transaction per initStep; allows sensor manager to interleave IMU and ToF safely.
size_t initStepCount();
esp_err_t initStep(size_t step, uint8_t& id, uint8_t& register_address);
esp_err_t sample(Raw &out); // SI acceleration + raw counts for calibration
float radPerCount();
}
