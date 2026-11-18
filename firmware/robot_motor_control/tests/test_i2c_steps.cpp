#include "../main/sensor_bus.h"
#include "../main/mpu6500_device.h"
#include "../main/vl6180x_device.h"
#include "../main/robot_config.h"
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <map>
#include <utility>

struct FakeDevice {
    uint16_t addr;
    std::map<uint16_t,uint8_t> regs;
};
static FakeDevice mpu{0x68,{}}, tof{0x29,{}};
static bool bus_created = false;
static int mpu_transactions=0, tof_transactions=0;
const char *esp_err_to_name(esp_err_t){return "TEST";}
esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t* cfg,i2c_master_bus_handle_t* out) {
    assert(!bus_created);
    assert(cfg->sda_io_num == RobotConfig::I2C_SDA_GPIO);
    assert(cfg->scl_io_num == RobotConfig::I2C_SCL_GPIO);
    assert(cfg->sda_io_num == 15 && cfg->scl_io_num == 16);
    bus_created = true;
    *out = reinterpret_cast<void*>(0x1);
    return ESP_OK;
}
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t, const i2c_device_config_t* cfg,
                                     i2c_master_dev_handle_t* out) {
    assert(cfg->scl_speed_hz == 100000);
    if (cfg->device_address == 0x68) *out = &mpu;
    else if (cfg->device_address == 0x29) *out = &tof;
    else return ESP_FAIL;
    return ESP_OK;
}
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t dev, const uint8_t* tx,size_t n,int) {
    auto* d = static_cast<FakeDevice*>(dev);
    if (d == &mpu) {
        assert(n==2);
        d->regs[tx[0]]=tx[1];
        ++mpu_transactions;
        return ESP_OK;
    }
    assert(d == &tof && n==3);
    d->regs[(uint16_t(tx[0])<<8)|tx[1]]=tx[2];
    ++tof_transactions;
    return ESP_OK;
}
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t dev, const uint8_t* tx, size_t n,
                                      uint8_t* rx, size_t count,int) {
    auto* d = static_cast<FakeDevice*>(dev);
    assert(d == &mpu || d == &tof);
    uint16_t reg = 0;
    if (d == &mpu) {assert(n==1); reg=tx[0]; ++mpu_transactions;}
    else {assert(n==2); reg=(uint16_t(tx[0])<<8)|tx[1];++tof_transactions;}
    for(size_t j=0;j<count;j++) rx[j]=d->regs[reg+j];
    return ESP_OK;
}
int main() {
    assert(SensorBus::start()==ESP_OK);
    assert(bus_created);
    assert(SensorBus::imu() != SensorBus::tof());
    mpu.regs[0x75]=0x70;
    tof.regs[0x0000]=0xB4;
    uint8_t imu_id=0,tof_id=0,reg8=0;
    uint16_t reg16=0;
    // Alternate one configuration step per device, as performed by the sensor-owner task.
    const size_t total = Vl6180x::initStepCount();
    const size_t msteps = Mpu6500::initStepCount();
    for(size_t step=0;step<total;step++) {
      if(step<msteps) {
        assert(Mpu6500::initStep(step,imu_id,reg8)==ESP_OK);
        assert(reg8 == (step==0?0x75:reg8));
      }
      int before=tof_transactions;
      assert(Vl6180x::initStep(step,tof_id,reg16)==ESP_OK);
      assert(tof_transactions==before+1); // no 40-register I2C burst
    }
    assert(imu_id==0x70 && tof_id==0xB4);
    assert(tof.regs[0x0016]==0);
    assert(tof.regs[0x0207]==1);
    assert(mpu.regs[0x6B]==1);
    assert(Mpu6500::initStep(msteps,imu_id,reg8)==ESP_ERR_INVALID_ARG);
    assert(Vl6180x::initStep(total,tof_id,reg16)==ESP_ERR_INVALID_ARG);
    std::puts("PASS: shared I2C bus distinct 0x68/0x29, 1-byte/2-byte regs, incremental init");
}
