#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
// VL6180X only, verified with physical MODEL_ID=0xB4.
// Based on ST AN4545 SR03 initialization and single-shot ranging.
namespace Vl6180x {
struct Range { uint8_t distance_mm=0; uint8_t error_code=0; bool valid=false; };
esp_err_t init(uint8_t& id);
// Single register operation per step; 0 reads MODEL_ID, final step clears reset flag.
size_t initStepCount();
esp_err_t initStep(size_t step, uint8_t& id, uint16_t& register_address);
esp_err_t startSingle();
esp_err_t readIfReady(bool& ready, Range& out);
}
