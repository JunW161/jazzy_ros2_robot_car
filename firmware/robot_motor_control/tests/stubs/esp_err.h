#pragma once
using esp_err_t=int;
constexpr esp_err_t ESP_OK=0,ESP_FAIL=-1,ESP_ERR_INVALID_ARG=0x102,
 ESP_ERR_INVALID_STATE=0x103,ESP_ERR_TIMEOUT=0x107,ESP_ERR_INVALID_RESPONSE=0x108;
const char* esp_err_to_name(esp_err_t);

constexpr esp_err_t ESP_ERR_NVS_NO_FREE_PAGES=0x110d;
constexpr esp_err_t ESP_ERR_NVS_NEW_VERSION_FOUND=0x1110;
#define ESP_ERROR_CHECK(x) ((void)(x))
