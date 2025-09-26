#pragma once

#include <string>
#include <unordered_map>
#include "model_handler.h"

namespace esphome {
namespace meter_reader_tflite {

static const std::unordered_map<std::string, ModelConfig> MODEL_CONFIGS = {
    {"class100-0180", 
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
    }
};


static const ModelConfig DEFAULT_MODEL_CONFIG = MODEL_CONFIGS.at("class100-0180");

}  // namespace meter_reader_tflite
}  // namespace esphome