#include "vl6180x_device.h"
#include "sensor_bus.h"
namespace Vl6180x {
namespace {
struct Reg { uint16_t reg; uint8_t value; };
// ST AN4545 Rev2, SR03 mandatory sequence. Order matters; applied after reset.
constexpr Reg sr03[] = {
 {0x0207,0x01},{0x0208,0x01},{0x0096,0x00},{0x0097,0xFD},
 {0x00E3,0x01},{0x00E4,0x03},{0x00E5,0x02},{0x00E6,0x01},
 {0x00E7,0x03},{0x00F5,0x02},{0x00D9,0x05},{0x00DB,0xCE},
 {0x00DC,0x03},{0x00DD,0xF8},{0x009F,0x00},{0x00A3,0x3C},
 {0x00B7,0x00},{0x00BB,0x3C},{0x00B2,0x09},{0x00CA,0x09},
 {0x0198,0x01},{0x01B0,0x17},{0x01AD,0x00},{0x00FF,0x05},
 {0x0100,0x05},{0x0199,0x05},{0x01A6,0x1B},{0x01AC,0x3E},
 {0x01A7,0x1F},{0x0030,0x00},
 // ST recommended public-register settings for standard ranging.
 {0x0011,0x10},{0x010A,0x30},{0x003F,0x46},{0x0031,0xFF},
 {0x0041,0x63},{0x002E,0x01},{0x001B,0x09},
 {0x003E,0x31},{0x0014,0x24}
};
esp_err_t wr(uint16_t reg,uint8_t value){return SensorBus::write8(SensorBus::tof(),reg,value);}
esp_err_t rd(uint16_t reg,uint8_t& value){return SensorBus::read8(SensorBus::tof(),reg,value);}
}
size_t initStepCount() { return 2 + sizeof(sr03)/sizeof(sr03[0]); }
esp_err_t initStep(size_t step, uint8_t& id, uint16_t& register_address) {
  if (step == 0) {
    register_address = 0x0000;
    esp_err_t e = rd(register_address, id);
    if (e != ESP_OK) return e;
    return id == 0xB4 ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
  }
  if (step <= sizeof(sr03)/sizeof(sr03[0])) {
    const Reg& item = sr03[step-1];
    register_address = item.reg;
    return wr(item.reg, item.value);
  }
  if (step == initStepCount()-1) {
    register_address = 0x0016; // FRESH_OUT_OF_RESET
    return wr(register_address, 0x00);
  }
  return ESP_ERR_INVALID_ARG;
}
esp_err_t init(uint8_t& id) {
  id = 0;
  uint16_t reg = 0;
  for (size_t step = 0; step < initStepCount(); ++step) {
    const esp_err_t e = initStep(step, id, reg);
    if (e != ESP_OK) return e;
  }
  return ESP_OK;
}
esp_err_t startSingle(){return wr(0x0018,0x01);}
esp_err_t readIfReady(bool& ready,Range& out){
  ready=false;
  uint8_t interrupt=0;
  esp_err_t e=rd(0x004F,interrupt);
  if(e!=ESP_OK)return e;
  if((interrupt&0x04)==0)return ESP_OK;
  ready=true;
  uint8_t status=0,mm=0;
  e=rd(0x004D,status);
  if(e!=ESP_OK)return e;
  e=rd(0x0062,mm);
  if(e!=ESP_OK)return e;
  e=wr(0x0015,0x07); // Always clear after reading.
  if(e!=ESP_OK)return e;
  out.distance_mm=mm;
  out.error_code=(status>>4)&0x0F;
  // Data quality: reject fault codes, zero, and distances beyond nominal 100 mm.
  out.valid=(out.error_code==0 && mm>=1 && mm<=100);
  return ESP_OK;
}
}
