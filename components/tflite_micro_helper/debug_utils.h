#pragma once

#include "esphome/core/hal.h"  // For millis()
#include "esphome/core/log.h"
#include "tensorflow/lite/c/common.h" // For TfLiteType

#ifdef DEBUG_DURATION
#define DURATION_START() uint32_t duration_start_ = millis()
#define DURATION_END(func) ESP_LOGD(TAG, "%s duration: %lums", func, millis() - duration_start_)
#define DURATION_LOG(msg, val) ESP_LOGD(TAG, "%s: %lums", msg, val)
#else
#define DURATION_START()
#define DURATION_END(func)
#define DURATION_LOG(msg, val)
#endif

// Helper function to convert TfLiteType to string
inline const char* tflite_type_to_string(TfLiteType type) {
    switch (type) {
        case kTfLiteFloat32: return "kTfLiteFloat32";
        case kTfLiteUInt8: return "kTfLiteUInt8";
        case kTfLiteInt8: return "kTfLiteInt8";
        case kTfLiteInt32: return "kTfLiteInt32";
        case kTfLiteInt64: return "kTfLiteInt64";
        case kTfLiteBool: return "kTfLiteBool";
        case kTfLiteString: return "kTfLiteString";
        case kTfLiteNoType: return "kTfLiteNoType";
        default: return "Unknown";
    }
}
