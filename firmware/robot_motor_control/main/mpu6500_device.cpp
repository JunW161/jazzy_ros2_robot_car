#include "mpu6500_device.h"
#include "sensor_bus.h"
#include <cmath>
namespace Mpu6500 {
namespace { int16_t decode(uint8_t hi,uint8_t lo){return int16_t((uint16_t(hi)<<8)|lo);} }
namespace {
constexpr uint8_t settings[][2] = {
  {0x6B,0x01}, {0x1B,0x08}, {0x1C,0x08},
  {0x1A,0x04}, {0x1D,0x04}, {0x19,0x04}
};
}
size_t initStepCount() { return 1 + sizeof(settings)/sizeof(settings[0]); }
esp_err_t initStep(size_t step, uint8_t& id, uint8_t& register_address) {
  if (step == 0) {
    register_address = 0x75;
    esp_err_t result = SensorBus::mpuRead(register_address, &id, 1);
    if (result != ESP_OK) return result;
    return id == 0x70 ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
  }
  if (step >= initStepCount()) return ESP_ERR_INVALID_ARG;
  register_address = settings[step-1][0];
  return SensorBus::mpuWrite(register_address, settings[step-1][1]);
}
esp_err_t init() {
  uint8_t id = 0, reg = 0;
  for (size_t step = 0; step < initStepCount(); ++step) {
    esp_err_t e = initStep(step, id, reg);
    if (e != ESP_OK) return e;
  }
  return ESP_OK;
}
esp_err_t sample(Raw &out){
  uint8_t data[14]={};
  esp_err_t e=SensorBus::mpuRead(0x3B,data,sizeof(data));
  if(e!=ESP_OK)return e;
  const float accel=9.80665f/8192.0f;
  out.ax=decode(data[0],data[1])*accel;
  out.ay=decode(data[2],data[3])*accel;
  out.az=decode(data[4],data[5])*accel;
  out.gyro_counts[0]=float(decode(data[8],data[9]));
  out.gyro_counts[1]=float(decode(data[10],data[11]));
  out.gyro_counts[2]=float(decode(data[12],data[13]));
  return ESP_OK;
}
float radPerCount(){return 3.14159265358979323846f/(180.0f*65.5f);}
} // namespace
