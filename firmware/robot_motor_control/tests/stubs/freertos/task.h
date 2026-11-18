#pragma once
#include "freertos/FreeRTOS.h"
TickType_t xTaskGetTickCount();
void vTaskDelayUntil(TickType_t*,TickType_t);
void vTaskDelay(TickType_t);
int xTaskCreatePinnedToCore(void(*)(void*),const char*,uint32_t,void*,uint32_t,void*,int);
void vTaskDelete(void*);
