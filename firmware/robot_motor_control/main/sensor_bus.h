#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
// Sole owner of the shared I2C controller; the ESP-IDF master driver serializes transactions.
namespace SensorBus {
  esp_err_t start();
  i2c_master_dev_handle_t imu();
  i2c_master_dev_handle_t tof();
  esp_err_t write8(i2c_master_dev_handle_t device, uint16_t reg, uint8_t value);
  esp_err_t read8(i2c_master_dev_handle_t device, uint16_t reg, uint8_t &value);
  esp_err_t read(i2c_master_dev_handle_t device, uint16_t reg, uint8_t* value, size_t size);
  // MPU register space is one byte, unlike the two-byte VL6180X index.
  esp_err_t mpuWrite(uint8_t reg, uint8_t value);
  esp_err_t mpuRead(uint8_t reg, uint8_t *out, size_t length);
}
