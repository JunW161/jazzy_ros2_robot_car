#include "sensor_bus.h"
#include "robot_config.h"
namespace SensorBus {
namespace {
  i2c_master_bus_handle_t bus = nullptr;
  i2c_master_dev_handle_t imu_handle = nullptr, tof_handle = nullptr;
}
// Reused for both drivers. Config remains exactly GPIO15 SDA/GPIO16 SCL.
esp_err_t start() {
  if(bus) return ESP_OK;
  i2c_master_bus_config_t cfg = {};
  cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  cfg.i2c_port = I2C_NUM_0;
  cfg.sda_io_num = (gpio_num_t)RobotConfig::I2C_SDA_GPIO;
  cfg.scl_io_num = (gpio_num_t)RobotConfig::I2C_SCL_GPIO;
  cfg.glitch_ignore_cnt = 7;
  cfg.flags.enable_internal_pullup = true;
  esp_err_t e = i2c_new_master_bus(&cfg, &bus);
  if(e != ESP_OK) return e;
  i2c_device_config_t dev = {};
  dev.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev.scl_speed_hz = 100000;
  dev.device_address = 0x68;
  e = i2c_master_bus_add_device(bus, &dev, &imu_handle);
  if(e != ESP_OK) return e;
  dev.device_address = 0x29;
  return i2c_master_bus_add_device(bus, &dev, &tof_handle);
}
i2c_master_dev_handle_t imu(){return imu_handle;}
i2c_master_dev_handle_t tof(){return tof_handle;}
esp_err_t mpuWrite(uint8_t reg,uint8_t value){
  const uint8_t tx[2]={reg,value};
  return i2c_master_transmit(imu_handle,tx,2,30);
}
esp_err_t mpuRead(uint8_t reg,uint8_t *out,size_t size){
  return i2c_master_transmit_receive(imu_handle,&reg,1,out,size,30);
}
esp_err_t read(i2c_master_dev_handle_t d,uint16_t reg,uint8_t *out,size_t size){
  const uint8_t addr[2]={uint8_t(reg>>8),uint8_t(reg)};
  return i2c_master_transmit_receive(d,addr,2,out,size,30);
}
esp_err_t read8(i2c_master_dev_handle_t d,uint16_t reg,uint8_t& value){
  return read(d,reg,&value,1);
}
esp_err_t write8(i2c_master_dev_handle_t d,uint16_t reg,uint8_t value){
  const uint8_t tx[3]={uint8_t(reg>>8),uint8_t(reg),value};
  return i2c_master_transmit(d,tx,3,30);
}
} // namespace SensorBus
