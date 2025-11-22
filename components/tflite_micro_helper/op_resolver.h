#pragma once

#include <set>
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tflite_micro_helper {

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
        // ====== Core Neural Network Ops ======
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
        case tflite::BuiltinOperator_MEAN:
          add_status = resolver.AddMean(); break;
        case tflite::BuiltinOperator_L2_POOL_2D:
          add_status = resolver.AddL2Pool2D(); break;
        case tflite::BuiltinOperator_L2_NORMALIZATION:
          add_status = resolver.AddL2Normalization(); break;

        // ====== Activation Functions ======
        case tflite::BuiltinOperator_RELU:
          add_status = resolver.AddRelu(); break;
        case tflite::BuiltinOperator_RELU6:
          add_status = resolver.AddRelu6(); break;
        case tflite::BuiltinOperator_LOGISTIC:
          add_status = resolver.AddLogistic(); break;
        case tflite::BuiltinOperator_TANH:
          add_status = resolver.AddTanh(); break;
        case tflite::BuiltinOperator_HARD_SWISH:
          add_status = resolver.AddHardSwish(); break;
        case tflite::BuiltinOperator_LEAKY_RELU:
          add_status = resolver.AddLeakyRelu(); break;
        case tflite::BuiltinOperator_PRELU:
          add_status = resolver.AddPrelu(); break;

        // ====== Basic Math Ops ======
        case tflite::BuiltinOperator_ADD:
          add_status = resolver.AddAdd(); break;
        case tflite::BuiltinOperator_SUB:
          add_status = resolver.AddSub(); break;
        case tflite::BuiltinOperator_MUL:
          add_status = resolver.AddMul(); break;
        case tflite::BuiltinOperator_DIV:
          add_status = resolver.AddDiv(); break;
        case tflite::BuiltinOperator_FLOOR_DIV:
          add_status = resolver.AddFloorDiv(); break;
        case tflite::BuiltinOperator_FLOOR_MOD:
          add_status = resolver.AddFloorMod(); break;
        case tflite::BuiltinOperator_FLOOR:
          add_status = resolver.AddFloor(); break;
        case tflite::BuiltinOperator_CEIL:
          add_status = resolver.AddCeil(); break;
        case tflite::BuiltinOperator_ROUND:
          add_status = resolver.AddRound(); break;
        case tflite::BuiltinOperator_ABS:
          add_status = resolver.AddAbs(); break;
        case tflite::BuiltinOperator_SQRT:
          add_status = resolver.AddSqrt(); break;
        case tflite::BuiltinOperator_RSQRT:
          add_status = resolver.AddRsqrt(); break;
        case tflite::BuiltinOperator_SQUARE:
          add_status = resolver.AddSquare(); break;
        case tflite::BuiltinOperator_EXP:
          add_status = resolver.AddExp(); break;
        case tflite::BuiltinOperator_LOG:
          add_status = resolver.AddLog(); break;
        case tflite::BuiltinOperator_SIN:
          add_status = resolver.AddSin(); break;
        case tflite::BuiltinOperator_COS:
          add_status = resolver.AddCos(); break;

        // ====== Comparison Ops ======
        case tflite::BuiltinOperator_EQUAL:
          add_status = resolver.AddEqual(); break;
        case tflite::BuiltinOperator_NOT_EQUAL:
          add_status = resolver.AddNotEqual(); break;
        case tflite::BuiltinOperator_GREATER:
          add_status = resolver.AddGreater(); break;
        case tflite::BuiltinOperator_GREATER_EQUAL:
          add_status = resolver.AddGreaterEqual(); break;
        case tflite::BuiltinOperator_LESS:
          add_status = resolver.AddLess(); break;
        case tflite::BuiltinOperator_LESS_EQUAL:
          add_status = resolver.AddLessEqual(); break;

        // ====== Tensor Operations ======
        case tflite::BuiltinOperator_RESHAPE:
          add_status = resolver.AddReshape(); break;
        case tflite::BuiltinOperator_QUANTIZE:
          add_status = resolver.AddQuantize(); break;
        case tflite::BuiltinOperator_DEQUANTIZE:
          add_status = resolver.AddDequantize(); break;
        case tflite::BuiltinOperator_CONCATENATION:
          add_status = resolver.AddConcatenation(); break;
        case tflite::BuiltinOperator_SPLIT:
          add_status = resolver.AddSplit(); break;
        case tflite::BuiltinOperator_SPLIT_V:
          add_status = resolver.AddSplitV(); break;
        case tflite::BuiltinOperator_STRIDED_SLICE:
          add_status = resolver.AddStridedSlice(); break;
        case tflite::BuiltinOperator_SLICE:
          add_status = resolver.AddSlice(); break;
        case tflite::BuiltinOperator_PACK:
          add_status = resolver.AddPack(); break;
        case tflite::BuiltinOperator_UNPACK:
          add_status = resolver.AddUnpack(); break;
        case tflite::BuiltinOperator_PAD:
          add_status = resolver.AddPad(); break;
        case tflite::BuiltinOperator_PADV2:
          add_status = resolver.AddPadV2(); break;
        case tflite::BuiltinOperator_FILL:
          add_status = resolver.AddFill(); break;
        case tflite::BuiltinOperator_EXPAND_DIMS:
          add_status = resolver.AddExpandDims(); break;
        case tflite::BuiltinOperator_SQUEEZE:
          add_status = resolver.AddSqueeze(); break;
        case tflite::BuiltinOperator_TRANSPOSE:
          add_status = resolver.AddTranspose(); break;
        case tflite::BuiltinOperator_TRANSPOSE_CONV:
          add_status = resolver.AddTransposeConv(); break;
        case tflite::BuiltinOperator_SPACE_TO_BATCH_ND:
          add_status = resolver.AddSpaceToBatchNd(); break;
        case tflite::BuiltinOperator_BATCH_TO_SPACE_ND:
          add_status = resolver.AddBatchToSpaceNd(); break;
        case tflite::BuiltinOperator_SPACE_TO_DEPTH:
          add_status = resolver.AddSpaceToDepth(); break;
        case tflite::BuiltinOperator_DEPTH_TO_SPACE:
          add_status = resolver.AddDepthToSpace(); break;

        // ====== Reduction Ops ======
        case tflite::BuiltinOperator_REDUCE_MAX:
          add_status = resolver.AddReduceMax(); break;
        case tflite::BuiltinOperator_REDUCE_MIN:
          add_status = resolver.AddReduceMin(); break;

        // ====== Element-wise Ops ======
        case tflite::BuiltinOperator_MAXIMUM:
          add_status = resolver.AddMaximum(); break;
        case tflite::BuiltinOperator_MINIMUM:
          add_status = resolver.AddMinimum(); break;
        case tflite::BuiltinOperator_ARG_MAX:
          add_status = resolver.AddArgMax(); break;
        case tflite::BuiltinOperator_ARG_MIN:
          add_status = resolver.AddArgMin(); break;
        case tflite::BuiltinOperator_SELECT_V2:
          add_status = resolver.AddSelectV2(); break;
        case tflite::BuiltinOperator_ELU:
          add_status = resolver.AddElu(); break;

        // ====== Logical Ops ======
        case tflite::BuiltinOperator_LOGICAL_AND:
          add_status = resolver.AddLogicalAnd(); break;
        case tflite::BuiltinOperator_LOGICAL_OR:
          add_status = resolver.AddLogicalOr(); break;
        case tflite::BuiltinOperator_LOGICAL_NOT:
          add_status = resolver.AddLogicalNot(); break;

        // ====== Advanced Ops ======
        case tflite::BuiltinOperator_RESIZE_NEAREST_NEIGHBOR:
          add_status = resolver.AddResizeNearestNeighbor(); break;
        case tflite::BuiltinOperator_RESIZE_BILINEAR:
          add_status = resolver.AddResizeBilinear(); break;
        case tflite::BuiltinOperator_GATHER:
          add_status = resolver.AddGather(); break;
        case tflite::BuiltinOperator_GATHER_ND:
          add_status = resolver.AddGatherNd(); break;
        case tflite::BuiltinOperator_REVERSE_V2:
          add_status = resolver.AddReverseV2(); break;
        case tflite::BuiltinOperator_UNIDIRECTIONAL_SEQUENCE_LSTM:
          add_status = resolver.AddUnidirectionalSequenceLSTM(); break;

        // ====== Embedding Ops ======
        case tflite::BuiltinOperator_EMBEDDING_LOOKUP:
          add_status = resolver.AddEmbeddingLookup(); break;

        // ====== Matrix Operations ======
        case tflite::BuiltinOperator_BATCH_MATMUL:
          add_status = resolver.AddBatchMatMul(); break;

        // ====== Skip Ops that are not available in TFLM ======
        case tflite::BuiltinOperator_CUSTOM:
        case tflite::BuiltinOperator_HASHTABLE_LOOKUP:
        case tflite::BuiltinOperator_SVDF:
        case tflite::BuiltinOperator_LSH_PROJECTION:
        case tflite::BuiltinOperator_SKIP_GRAM:
        case tflite::BuiltinOperator_CALL:
          ESP_LOGW(tag, "Operator %s is not available in TFLite Micro or requires custom implementation", op_name);
          return false;

        default:
          ESP_LOGE(tag, "Unknown or unsupported operator: %s", op_name);
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

}  // namespace tflite_micro_helper
}  // namespace esphome
