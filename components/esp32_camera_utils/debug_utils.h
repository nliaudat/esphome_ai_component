#pragma once

#include "esphome/core/hal.h"  // For millis()
#include "esphome/core/log.h"

#ifdef DEBUG_DURATION
#define DURATION_START() uint32_t duration_start_ = millis()
#define DURATION_END(func) ESP_LOGD(TAG, "%s duration: %lums", func, millis() - duration_start_)
#define DURATION_LOG(msg, val) ESP_LOGD(TAG, "%s: %lums", msg, val)
#else
#define DURATION_START()
#define DURATION_END(func)
#define DURATION_LOG(msg, val)
#endif
