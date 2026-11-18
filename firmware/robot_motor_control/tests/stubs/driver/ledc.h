#pragma once
#include "esp_err.h"
using ledc_mode_t=int;
using ledc_timer_t=int;
using ledc_channel_t=int;
constexpr int LEDC_LOW_SPEED_MODE=0,LEDC_TIMER_0=0,LEDC_CHANNEL_0=0,
LEDC_CHANNEL_1=1,LEDC_CHANNEL_2=2,LEDC_CHANNEL_3=3,LEDC_TIMER_8_BIT=8,
LEDC_AUTO_CLK=0,LEDC_INTR_DISABLE=0;
struct ledc_timer_config_t {int speed_mode=0,timer_num=0,duty_resolution=0,freq_hz=0,clk_cfg=0;};
struct ledc_channel_config_t {int gpio_num=0,speed_mode=0,channel=0,intr_type=0,timer_sel=0,duty=0,hpoint=0;};
esp_err_t ledc_timer_config(const ledc_timer_config_t*);
esp_err_t ledc_channel_config(const ledc_channel_config_t*);
esp_err_t ledc_set_duty(ledc_mode_t,ledc_channel_t,uint32_t);
esp_err_t ledc_update_duty(ledc_mode_t,ledc_channel_t);
