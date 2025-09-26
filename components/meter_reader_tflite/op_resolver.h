#pragma once

#include <set>
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "esphome/core/log.h"

namespace esphome {
namespace meter_reader_tflite {

class OpResolverManager {
 public:
  template <size_t tOpCount>
  static bool RegisterOps(tflite::MicroMutableOpResolver<tOpCount> &resolver,
                          const std::set<tflite::BuiltinOperator> &required_ops,
                          const char *tag) {
    for (auto op : required_ops) {
      const char* op_name = tflite::EnumNameBuiltinOperator(op);
      ESP_LOGD(tag, "Registering op: %s", op_name);
      
      TfLiteStatus add_status = kTfLiteError;
      switch (op) {
        // ====== Supported Operators ======
        // Core Neural Network Ops
        case tflite::BuiltinOperator_CONV_2D:
          add_status = resolver.AddConv2D(); break;
        case tflite::BuiltinOperator_DEPTHWISE_CONV_2D:
          add_status = resolver.AddDepthwiseConv2D(); break;
        case tflite::BuiltinOperator_FULLY_CONNECTED:
          add_status = resolver.AddFullyConnected(); break;
        case tflite::BuiltinOperator_SOFTMAX:
          add_status = resolver.AddSoftmax(); break;
        case tflite::BuiltinOperator_AVERAGE_POOL_2D:
          add_status = resolver.AddAveragePool2D(); break;
        case tflite::BuiltinOperator_MAX_POOL_2D:
          add_status = resolver.AddMaxPool2D(); break;

        // Activation Functions
        case tflite::BuiltinOperator_RELU:
          add_status = resolver.AddRelu(); break;
        case tflite::BuiltinOperator_RELU6:
          add_status = resolver.AddRelu6(); break;
        case tflite::BuiltinOperator_LOGISTIC:
          add_status = resolver.AddLogistic(); break;

        // Basic Math Ops
        case tflite::BuiltinOperator_ADD:
          add_status = resolver.AddAdd(); break;
        case tflite::BuiltinOperator_SUB:
          add_status = resolver.AddSub(); break;
        case tflite::BuiltinOperator_MUL:
          add_status = resolver.AddMul(); break;
        case tflite::BuiltinOperator_DIV:
          add_status = resolver.AddDiv(); break;

        // Tensor Operations
        case tflite::BuiltinOperator_RESHAPE:
          add_status = resolver.AddReshape(); break;
        case tflite::BuiltinOperator_QUANTIZE:
          add_status = resolver.AddQuantize(); break;
        case tflite::BuiltinOperator_DEQUANTIZE:
          add_status = resolver.AddDequantize(); break;
        case tflite::BuiltinOperator_CONCATENATION:
          add_status = resolver.AddConcatenation(); break;
        case tflite::BuiltinOperator_STRIDED_SLICE:
          add_status = resolver.AddStridedSlice(); break;
        case tflite::BuiltinOperator_PAD:
          add_status = resolver.AddPad(); break;
        case tflite::BuiltinOperator_PADV2:
          add_status = resolver.AddPadV2(); break;

        // ====== Unsupported Operators (commented out) ======
        /*
        case tflite::BuiltinOperator_L2_POOL_2D:  // Not available in standard TFLM
          add_status = resolver.AddL2Pool2D(); break;
        case tflite::BuiltinOperator_L2_NORMALIZATION:  // Requires custom implementation
          add_status = resolver.AddL2Normalization(); break;
        case tflite::BuiltinOperator_TANH:  // Not optimized for microcontrollers
          add_status = resolver.AddTanh(); break;
        case tflite::BuiltinOperator_HARD_SWISH:  // Requires specific hardware
          add_status = resolver.AddHardSwish(); break;
        case tflite::BuiltinOperator_FLOOR_DIV:  // Not commonly supported
          add_status = resolver.AddFloorDiv(); break;
        case tflite::BuiltinOperator_FLOOR_MOD:  // Not commonly supported
          add_status = resolver.AddFloorMod(); break;
        case tflite::BuiltinOperator_MAXIMUM:  // Limited support
          add_status = resolver.AddMaximum(); break;
        case tflite::BuiltinOperator_MINIMUM:  // Limited support
          add_status = resolver.AddMinimum(); break;
        */

        default:
          ESP_LOGE(tag, "Unsupported operator: %s", op_name);
          return false;
      }

      if (add_status != kTfLiteOk) {
        ESP_LOGE(tag, "Failed to add operator: %s", op_name);
        return false;
      }
    }
    return true;
  }
};

}  // namespace meter_reader_tflite
}  // namespace esphome