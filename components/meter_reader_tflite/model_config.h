#pragma once

#include "esphome/core/defines.h"

#ifdef USE_METER_READER_TFLITE

// Model configuration is now provided dynamically from __init__.py
// at build time via individual setter calls on TFLiteCoordinator.
// 
// The ModelConfig struct is defined in:
//   components/tflite_micro_helper/model_handler.h
//
// See __init__.py for the auto-detection logic that parses model .txt files
// and applies YAML overrides.

#endif // USE_METER_READER_TFLITE
