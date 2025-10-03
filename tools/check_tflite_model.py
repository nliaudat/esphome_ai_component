import argparse
import tensorflow as tf
from datetime import datetime
import sys

def safe_analyze_model(model_path):
    """Wrapper for tf.lite.experimental.Analyzer with error handling."""
    try:
        if hasattr(tf.lite, 'experimental') and hasattr(tf.lite.experimental, 'Analyzer'):
            return tf.lite.experimental.Analyzer.analyze(model_path=model_path)
        return None
    except Exception as e:
        print(f"[Warning] Analyzer failed: {str(e)}", file=sys.stderr)
        return None

def inspect_tflite_model(model_path, verbose=False, output_file=None):
    """Inspect ops, memory, and details in a TFLite model."""
    try:
        # Load model
        interpreter = tf.lite.Interpreter(model_path=model_path)
        interpreter.allocate_tensors()
        
        # Initialize output content
        output_content = []
        
        # --- Simple Op List ---
        print(f"\nOperations in model '{model_path}':")
        for op in interpreter._get_ops_details():
            print(op['op_name'])
        
        # --- Memory Analysis ---
        tensor_details = interpreter.get_tensor_details()
        total_arena_size = 0
        memory_breakdown = []

        for tensor in tensor_details:
            dtype_size = tf.dtypes.as_dtype(tensor['dtype']).size
            num_elements = 1
            for dim in tensor['shape']:
                num_elements *= dim
            tensor_memory = dtype_size * num_elements
            total_arena_size += tensor_memory +16
            if verbose:
                memory_breakdown.append(
                    f"  Tensor '{tensor.get('name', 'unnamed')}': "
                    f"{tensor_memory / 1024:.2f} KB "
                    f"(Shape: {tensor['shape']}, Dtype: {tensor['dtype']})"
                )

        # --- Report Header ---
        header = [
            f"\n=== TFLite Model Inspection Report ===",
            f"Model: {model_path}",
            f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            f"TensorFlow Version: {tf.__version__}",
            f"\n[Memory Usage]",
            f"Total Arena Size: {total_arena_size / 1024:.2f} KB (Approx. RAM needed for inference)"
        ]
        output_content.extend(header)

        # --- Analyzer Output ---
        analyzer_output = safe_analyze_model(model_path)
        if analyzer_output:
            output_content.append("\n[Analyzer Output]\n" + analyzer_output)
        else:
            output_content.append("\n[Warning] Analyzer not available/failed.")

        # --- Verbose Details ---
        if verbose:
            if memory_breakdown:
                output_content.append("\n[Memory Breakdown]")
                output_content.extend(memory_breakdown)
            
            output_content.append("\n[Detailed Operations]")
            for op in interpreter._get_ops_details():
                op_info = [
                    f"\nOp: {op['op_name']} (Index: {op['index']})",
                    f"  Input Tensors: {op['inputs']}",
                    f"  Output Tensors: {op['outputs']}"
                ]
                # Add quantization info
                for tensor_idx in op['inputs'] + op['outputs']:
                    tensor = interpreter.get_tensor_details()[tensor_idx]
                    if 'quantization' in tensor and tensor['quantization'] != (0, 0):
                        op_info.append(
                            f"  Quantization (Tensor {tensor_idx}): "
                            f"Scale={tensor['quantization'][0]}, "
                            f"Zero-Point={tensor['quantization'][1]}"
                        )
                output_content.extend(op_info)

        # --- Output Results ---
        report = '\n'.join(output_content)
        print(report)
        
        if output_file:
            with open(output_file, 'w') as f:
                f.write(report)
            print(f"\nReport saved to: {output_file}")

    except Exception as e:
        print(f"\nError inspecting model: {str(e)}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Inspect TFLite model operations, memory, and details.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "model_path",
        type=str,
        help="Path to the TFLite model file (e.g., 'model.tflite')"
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Show detailed memory/op info (inputs/outputs/quantization)"
    )
    parser.add_argument(
        "--output_file",
        type=str,
        default=None,
        help="Save report to a file (e.g., 'report.txt')"
    )
    args = parser.parse_args()
    inspect_tflite_model(args.model_path, args.verbose, args.output_file)