// V7 I2C owner: one FreeRTOS task performs ALL sensor I2C traffic.
// Wheel control never waits for the bus; the bus manager exposes timestamped snapshots.
#include "robot_sensors.h"
#include "robot_config.h"
#include "sensor_bus.h"
#include "mpu6500_device.h"
#include "imu_calibration_v10.h"
#include "vl6180x_device.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {
portMUX_TYPE imu_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE range_mux = portMUX_INITIALIZER_UNLOCKED;
ImuSample last_imu;
RangeReport last_range;

uint64_t nowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }
void store(const ImuSample& s) {
  portENTER_CRITICAL(&imu_mux); last_imu = s; portEXIT_CRITICAL(&imu_mux);
}
void store(const RangeReport& s) {
  portENTER_CRITICAL(&range_mux); last_range = s; portEXIT_CRITICAL(&range_mux);
}

struct ImuManager {
  ImuSample data;
  bool initialized = false;
  size_t init_step = 0;
  uint8_t who = 0;
  uint64_t retry_after_us = 0;
  int errors_in_row = 0;
  int calibration_count = 0;
  ImuCalibrationV10::Window gyroWindow;
  float bias[3] = {};
  uint64_t lastRejectLogUs=0;

  // V10 accepts stable nonzero factory gyro offset, but rejects motion/noise.
  void resetWindow() {
    calibration_count=0;
    gyroWindow.reset();
    data.calibration_progress=0;
  }
  void invalidate() {
    data.valid = false;
    data.calibrated = false;
    store(data);
  }
  void retry(esp_err_t err, uint8_t reg) {
    ++data.errors;
    initialized = false;
    data.initialized = false;
    data.calibration_reject_reason = 0;
    data.raw_gz_counts = 0.0f;
    data.bias_gz_counts = 0.0f;
    init_step = 0;
    resetWindow();
    errors_in_row = 0;
    retry_after_us = nowUs() + 500000ULL;
    invalidate();  // Publish the reset state, rather than the stale initialized flag.
    printf("I2C MPU6500 init/read failed reg=0x%02X err=%s; retry in 500ms\n",
           reg, esp_err_to_name(err));
  }
  void tick(uint64_t now) {
    if (!initialized) {
      if (now < retry_after_us) return;
      uint8_t reg = 0;
      esp_err_t err = Mpu6500::initStep(init_step, who, reg);
      if (init_step == 0) data.who = who; // Show WHO_AM_I even on a bad ID.
      if (err != ESP_OK) { retry(err, reg); return; }
      if (init_step == 0) printf("I2C MPU6500 WHO=0x%02X\n", who);
      ++init_step;
      if (init_step == Mpu6500::initStepCount()) {
        initialized = true;
        data.initialized = true;
        data.calibration_reject_reason = 0;
        resetWindow();
        invalidate();
        printf("MPU6500 configured: hold still for 150 gyro samples\n");
      }
      return;
    }

    Mpu6500::Raw raw;
    esp_err_t err = Mpu6500::sample(raw);
    if (err != ESP_OK) {
      ++data.errors;
      data.valid = false;
      store(data);
      if (++errors_in_row >= 3) {
        // retry() accounts for the third failure once; avoid double counting.
        --data.errors;
        retry(err, 0x3B);
      }
      return;
    }
    errors_in_row = 0;
    data.raw_gz_counts = raw.gyro_counts[2];
    const float magnitude = sqrtf(raw.ax * raw.ax + raw.ay * raw.ay + raw.az * raw.az);
    data.accel_norm = magnitude;
    if (calibration_count < ImuCalibrationV10::SAMPLES) {
      const int before=gyroWindow.count;
      const bool complete=gyroWindow.push(raw.gyro_counts,magnitude);
      calibration_count=gyroWindow.count;
      data.calibration_progress=static_cast<uint16_t>(calibration_count);
      data.calibration_reject_reason=gyroWindow.reason;
      if(gyroWindow.reason!=0){
        if(before>0)++data.calibration_restarts;
        if(now-lastRejectLogUs>2000000ULL){
          lastRejectLogUs=now;
          printf("IMU_CAL_REJECT reason=%u rawX=%+.1f rawY=%+.1f rawZ=%+.1f accel=%.3f count=%d errors=%lu\n",
                 (unsigned)gyroWindow.reason,raw.gyro_counts[0],raw.gyro_counts[1],raw.gyro_counts[2],
                 magnitude,calibration_count,(unsigned long)data.errors);
        }
      }
      if(calibration_count==50||calibration_count==100)
        printf("MPU6500 gyro calibration %d/150; keep robot STILL\n",calibration_count);
      if(complete){
        for(int i=0;i<3;++i)bias[i]=gyroWindow.bias[i];
        data.bias_gz_counts=bias[2];
        data.calibration_reject_reason=0;
        printf("MPU6500 V10 calibrated stationary bias X=%.2f Y=%.2f Z=%.2f counts\n",
               bias[0],bias[1],bias[2]);
      }
      invalidate();
      return;
    }

    data.ax = raw.ax; data.ay = raw.ay; data.az = raw.az;
    data.gx = (raw.gyro_counts[0] - bias[0]) * Mpu6500::radPerCount();
    data.gy = (raw.gyro_counts[1] - bias[1]) * Mpu6500::radPerCount();
    data.gz = (raw.gyro_counts[2] - bias[2]) * Mpu6500::radPerCount() * RobotConfig::IMU_GYRO_Z_SIGN;
    data.time_us = nowUs();
    ++data.seq;
    data.calibration_progress = 150;
    data.initialized = true;
    data.calibration_reject_reason = 0;
    data.valid = data.calibrated = true;
    store(data);
  }
};

enum class NearPhase : uint8_t {
  RESET_LOW, WAIT_LOW, WAIT_HIGH, CONFIG_STEP, START_SINGLE, WAIT_RESULT, RETRY
};
struct NearManager {
  RangeReport data;
  NearPhase phase = NearPhase::RESET_LOW;
  size_t init_step = 0;
  uint8_t model_id = 0;
  uint64_t next_action_us = 0;
  uint64_t measurement_start_us = 0;
  int errors_in_row = 0;

  void invalidate() { data.valid = false; store(data); }
  void fail(esp_err_t e, uint16_t reg, bool already_counted=false) {
    if (!already_counted) ++data.errors;
    data.initialized=false;
    invalidate();
    printf("I2C VL6180X failed phase=%u step=%u reg=0x%04X err=%s; retry in 1s\n",
           static_cast<unsigned>(phase), static_cast<unsigned>(init_step), reg,
           esp_err_to_name(e));
    phase=NearPhase::RETRY;
    next_action_us=nowUs()+1000000ULL;
    errors_in_row=0;
    init_step=0;
  }
  void readError(esp_err_t e, uint16_t reg) {
    ++data.errors;
    invalidate();
    if (++errors_in_row >= 3) { fail(e, reg, true); return; }
    phase=NearPhase::START_SINGLE;
  }
  void tick(uint64_t now) {
    switch (phase) {
      case NearPhase::RESET_LOW:
        data.initialized=false;
        invalidate();
        // Open drain low, then release. The breakout must provide a VALID pull-up.
        gpio_set_level((gpio_num_t)RobotConfig::TOF_XSHUT_GPIO, 0);
        next_action_us=now+10000ULL;
        phase=NearPhase::WAIT_LOW;
        return;
      case NearPhase::WAIT_LOW:
        if (now < next_action_us) return;
        gpio_set_level((gpio_num_t)RobotConfig::TOF_XSHUT_GPIO, 1);
        next_action_us=now+20000ULL;
        phase=NearPhase::WAIT_HIGH;
        return;
      case NearPhase::WAIT_HIGH:
        if (now < next_action_us) return;
        init_step=0; model_id=0;
        phase=NearPhase::CONFIG_STEP;
        return;
      case NearPhase::CONFIG_STEP: {
        // Precisely ONE register transaction per 20ms cycle: IMU is always serviced first.
        uint16_t reg=0;
        const esp_err_t e=Vl6180x::initStep(init_step, model_id, reg);
        data.model_id=model_id;
        if (e!=ESP_OK) { fail(e,reg); return; }
        ++init_step;
        if (init_step==Vl6180x::initStepCount()) {
          data.initialized=true;
          errors_in_row=0;
          store(data);
          phase=NearPhase::START_SINGLE;
          printf("VL6180X ready ID=0x%02X after %u incremental I2C init steps\n",
                 model_id, static_cast<unsigned>(init_step));
        }
        return;
      }
      case NearPhase::START_SINGLE: {
        const esp_err_t e=Vl6180x::startSingle();
        if (e!=ESP_OK) { readError(e,0x0018); return; }
        measurement_start_us=nowUs();
        phase=NearPhase::WAIT_RESULT;
        return;
      }
      case NearPhase::WAIT_RESULT: {
        bool ready=false;
        Vl6180x::Range result;
        const esp_err_t e=Vl6180x::readIfReady(ready,result);
        if (e!=ESP_OK) { readError(e,0x004F); return; }
        if (nowUs()-measurement_start_us > 150000ULL) {
          readError(ESP_ERR_TIMEOUT,0x004F); return;
        }
        if (!ready) return;
        errors_in_row=0;
        data.meters=result.distance_mm * 0.001f;
        data.range_status=result.error_code;
        data.valid=result.valid;
        data.time_us=nowUs(); ++data.seq;
        store(data);
        phase=NearPhase::START_SINGLE;
        return;
      }
      case NearPhase::RETRY:
        if (now >= next_action_us) phase=NearPhase::RESET_LOW;
        return;
    }
  }
};

void sensorOwnerTask(void*) {
  ImuManager imu;
  NearManager near;
  TickType_t last=xTaskGetTickCount();
  const TickType_t period=pdMS_TO_TICKS(20);
  while (true) {
    vTaskDelayUntil(&last, period);
    // One task is the only owner of I2C operations on 0x68 / 0x29.
    imu.tick(nowUs());
    near.tick(nowUs());
    // Avoid a burst of catch-up cycles if one hardware transaction times out.
    if (xTaskGetTickCount()-last > period) last=xTaskGetTickCount();
  }
}
} // namespace

void robot_imu_snapshot(ImuSample& s) {
  portENTER_CRITICAL(&imu_mux); s=last_imu; portEXIT_CRITICAL(&imu_mux);
}
void robot_range_snapshot(RangeReport& s) {
  portENTER_CRITICAL(&range_mux); s=last_range; portEXIT_CRITICAL(&range_mux);
}
void robot_sensors_start() {
  ESP_ERROR_CHECK(SensorBus::start());
  gpio_config_t pin={};
  pin.pin_bit_mask=1ULL<<RobotConfig::TOF_XSHUT_GPIO;
  pin.mode=GPIO_MODE_OUTPUT_OD;
  pin.pull_up_en=GPIO_PULLUP_DISABLE;
  pin.pull_down_en=GPIO_PULLDOWN_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&pin));
  gpio_set_level((gpio_num_t)RobotConfig::TOF_XSHUT_GPIO,1);
  // GPIO17/21 reserved: polling (no ISR). NEVER create another sensor I2C task.
  if (xTaskCreatePinnedToCore(sensorOwnerTask,"i2c_sensor_owner",8192,
                              nullptr,5,nullptr,1)!=pdPASS) abort();
}
