#pragma once

#include "esphome/core/hal.h"  // For millis() and other core functions
#include "esphome/core/log.h"
#include "meter_reader_tflite.h"
#include "model_handler.h"
#include "memory_manager.h"
#include "crop_zones.h"
#include "image_processor.h"

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


/* namespace esphome {
namespace meter_reader_tflite {

// Core debug functions
void print_meter_reader_debug_info(MeterReaderTFLite* component);
void print_core_debug_status(MeterReaderTFLite* component);
void print_model_debug_info(MeterReaderTFLite* component);
void print_crop_zone_debug_info(MeterReaderTFLite* component);
void print_image_processor_debug_info(MeterReaderTFLite* component);
void print_memory_debug_info(MeterReaderTFLite* component);
void print_statistics_debug_info(MeterReaderTFLite* component);

// Model handler debug functions
void print_model_handler_debug_info(const ModelHandler& handler);

// Memory manager debug functions  
void print_memory_manager_debug_info();

// Image processor debug functions
void print_image_processor_debug_info(const ImageProcessor& processor);

// Crop zone debug functions
void print_crop_zone_debug_info(const CropZoneHandler& handler);

}  // namespace meter_reader_tflite
}  // namespace esphome */