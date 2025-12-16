#pragma once

#include "esphome/components/tflite_micro_helper/model_handler.h"
#include <unordered_map>
#include <string>

namespace esphome {
namespace meter_reader_tflite {

using tflite_micro_helper::ModelConfig;

static const std::unordered_map<std::string, ModelConfig> MODEL_CONFIGS = {
    {"dig-class100-0180-s2-q", 
        ModelConfig{
            .description = "dig-class100-0180",
            .tensor_arena_size = "512KB", //check_tflite_model.py reports : Total Arena Size: 415.08 KB
            .output_processing = "softmax_jomjol", //logits_jomjol is the good mathematical way to calcultate the confidence, but softmax_jomjol give a greater confidence. //"softmax_jomjol", //"softmax",// "logits_jomjol", //model trained with from_logits=True
            .scale_factor = 10.0f, // For 100-class models (0.0-9.9)
            .input_type = "float32", //"uint8", // Model is float32, not quantized!
            .input_channels = 3,
            .input_order = "RGB", //Keras ImageDataGenerator typically uses RGB order
            .input_size = {32, 20}, // Explicitly set expected size
            .normalize = false //.normalize = false 
        }
    },
    {"class100-0173", 
        ModelConfig{
            .description = "dig-class100-0173",
            .tensor_arena_size = "512KB", //check_tflite_model.py reports : Total Arena Size: 415.08 KB
            .output_processing = "softmax_jomjol",
            .scale_factor = 10.0f,
            .input_type = "float32",  
            .input_channels = 3,
            .input_order = "RGB",
            .input_size = {32, 20}, 
            .normalize = false //.normalize = false      // Quantization handles scaling 
        }
    },
    {"class10-0900", 
        ModelConfig{
            .description = "dig-cont_0900",
            .tensor_arena_size = "800KB", //check_tflite_model.py reports : Total Arena Size: 725.60 KB
            .output_processing = "softmax_jomjol",
            .scale_factor = 1.0f,
            .input_type = "float32", 
            .input_channels = 3,
            .input_order = "RGB",
            .input_size = {32, 20}, 
            .normalize = false //.normalize = false      // Quantization handles scaling 
        }
    },
    {"class10-0810", 
        ModelConfig{
            .description = "dig-cont_0810",
            .tensor_arena_size = "800KB", //check_tflite_model.py reports : Total Arena Size: 725.60 KB
            .output_processing = "softmax_jomjol",
            .scale_factor = 1.0f,
            .input_type = "float32",
            .input_channels = 3,
            .input_order = "RGB",
            .input_size = {32, 20},
            .normalize = false //.normalize = false      // Quantization handles scaling 
        }
    },
    {"mnist", 
        ModelConfig{
            .description = "MNIST Digit Classifier",
            .tensor_arena_size = "900KB", //check_tflite_model.py reports : Total Arena Size: 814.44 KB
            .output_processing = "direct_class",
            .scale_factor = 1.0f,
            .input_type = "float32",
            .input_channels = 1,
            .input_order = "RGB",
            .input_size = {28, 28},
            .normalize = true,
            .invert = true
        }
    },
    {"digit_recognizer_v4_10cls_GRAY", 
        ModelConfig{
            .description = "digit_recognizer_v4_10cls_GRAY",
            .tensor_arena_size = "130KB", //check_tflite_model.py reports : Total Arena Size: 101.21 KB (doubling the model size seems to be validated in my tests)
            .output_processing = "softmax",
            .scale_factor = 1.0f,
            .input_type = "uint8",  
            .input_channels = 1,
            .input_order = "RGB",
            .input_size = {32, 20}, 
            .normalize = false 
        }
    },
    {"digit_recognizer_v4_10cls_RGB", 
        ModelConfig{
            .description = "digit_recognizer_v4_10cls_RGB",
            .tensor_arena_size = "110KB", //check_tflite_model.py reports : Total Arena Size: 104 KB
            .output_processing = "direct_class", //qat_quantized, auto_detect
            .scale_factor = 1.0f,
            .input_type = "uint8",  
            .input_channels = 3,
            .input_order = "RGB",
            .input_size = {32, 20}, 
            .normalize = false 
        }
    },
    {"digit_recognizer_v4_100cls_GRAY", 
        ModelConfig{
            .description = "digit_recognizer_v4_100cls_GRAY",
            .tensor_arena_size = "150KB", //check_tflite_model.py reports : Total Arena Size: 107.45 KB
            .output_processing = "softmax",
            .scale_factor = 10.0f,
            .input_type = "uint8",  
            .input_channels = 1,
            .input_order = "RGB",
            .input_size = {32, 20}, 
            .normalize = false 
        }
    },
    {"digit_recognizer_v4_100cls_RGB", 
        ModelConfig{
            .description = "digit_recognizer_v4_100cls_RGB",
            .tensor_arena_size = "190KB", //check_tflite_model.py reports : Total Arena Size: 136.50 KB 
            .output_processing = "softmax",
            .scale_factor = 10.0f,
            .input_type = "uint8",  
            .input_channels = 3,
            .input_order = "RGB",
            .input_size = {32, 20}, 
            .normalize = false 
        }
    },
    {"digit_recognizer_v3_10cls_RGB", 
        ModelConfig{
            .description = "digit_recognizer_v3_10cls_RGB",
            .tensor_arena_size = "75KB", //check_tflite_model.py reports : Total Arena Size: 70.3 KB
            .output_processing = "softmax", //qat_quantized, auto_detect
            .scale_factor = 1.0f,
            .input_type = "uint8",  
            .input_channels = 3,
            .input_order = "RGB",
            .input_size = {32, 20}, 
            .normalize = false 
        }
    }
};


// static const ModelConfig DEFAULT_MODEL_CONFIG = MODEL_CONFIGS.at("dig-class100-0180-s2-q");

}  // namespace meter_reader_tflite
}  // namespace esphome
