#include "../main/sensor_bus.h"
#include "../main/vl6180x_device.h"
#include <cassert>
#include <cstdint>
#include <map>
#include <utility>
#include <cstdio>
static std::map<uint16_t,uint8_t> registers;
static int writeCount=0;
const char *esp_err_to_name(esp_err_t){return "MOCK";}
esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t*,i2c_master_bus_handle_t* out){*out=reinterpret_cast<void*>(0x01);return ESP_OK;}
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t,const i2c_device_config_t* cfg,i2c_master_dev_handle_t* out){*out=reinterpret_cast<void*>(uintptr_t(cfg->device_address));return ESP_OK;}
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t dev,const uint8_t *tx,size_t n,int){
    if(uintptr_t(dev)==0x29&&n==3){registers[(uint16_t(tx[0])<<8)|tx[1]]=tx[2];writeCount++;return ESP_OK;}
    return ESP_FAIL;
}
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t dev,const uint8_t *tx,size_t n,uint8_t *rx,size_t count,int){
    if(uintptr_t(dev)!=0x29||n!=2)return ESP_FAIL;
    const uint16_t reg=(uint16_t(tx[0])<<8)|tx[1];
    for(size_t i=0;i<count;i++)rx[i]=registers[reg+i];
    return ESP_OK;
}
int main(){
    assert(SensorBus::start()==ESP_OK);
    uint8_t id=0;
    registers[0]=0xB4;
    assert(Vl6180x::init(id)==ESP_OK);
    assert(id==0xB4 && registers[0x0016]==0x00 && writeCount>30);
    assert(registers[0x0207]==0x01 && registers[0x0014]==0x24);
    assert(Vl6180x::startSingle()==ESP_OK && registers[0x0018]==1);
    bool ready=true;
    Vl6180x::Range range;
    registers[0x004F]=0;
    assert(Vl6180x::readIfReady(ready,range)==ESP_OK && !ready);
    registers[0x004F]=0x04;registers[0x004D]=0;registers[0x0062]=65;
    assert(Vl6180x::readIfReady(ready,range)==ESP_OK && ready && range.valid && range.distance_mm==65);
    assert(registers[0x0015]==0x07);
    registers[0x004D]=0x80;
    assert(Vl6180x::readIfReady(ready,range)==ESP_OK && !range.valid && range.error_code==8);
    registers[0]=0;id=0xFF;
    assert(Vl6180x::init(id)==ESP_ERR_INVALID_RESPONSE && id==0);
    puts("PASS: VL6180X mock bus ID, init, single shot, status filtering");
}
