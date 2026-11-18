#pragma once
#include <stdint.h>
#include "esp_err.h"
using gpio_num_t=int;
constexpr int GPIO_MODE_INPUT=1, GPIO_MODE_OUTPUT=2,GPIO_MODE_OUTPUT_OD=6,GPIO_PULLUP_DISABLE=0,GPIO_PULLDOWN_DISABLE=0,
 GPIO_INTR_DISABLE=0,GPIO_INTR_ANYEDGE=3;
struct gpio_config_t {
    uint64_t pin_bit_mask=0;
    int mode=0,pull_up_en=0,pull_down_en=0,intr_type=0;
};
esp_err_t gpio_config(const gpio_config_t* cfg);

int gpio_get_level(gpio_num_t pin);
esp_err_t gpio_set_level(gpio_num_t pin, int value);
esp_err_t gpio_install_isr_service(int flags);
esp_err_t gpio_isr_handler_add(gpio_num_t pin, void(*handler)(void*),void* arg);
