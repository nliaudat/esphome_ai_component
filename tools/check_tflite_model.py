import argparse
import tensorflow as tf
from datetime import datetime
import sys
import numpy as np
import os
from tqdm.auto import tqdm
import pandas as pd

def safe_print(*args, **kwargs):
    """Print with encoding error handling for Windows compatibility."""
    try:
        print(*args, **kwargs)
    except UnicodeEncodeError:
        # Replace non-ASCII characters with ASCII equivalents
        def safe_str(obj):
            s = str(obj)
            replacements = {
                '\U0001f50d': '[SEARCH]',
                '\u26a0\ufe0f': '[WARN]',
                '\u2713': '[OK]',
                '\u274c': '[FAIL]',
                '\U0001f4be': '[SAVE]',
                '\U0001f9ea': '[TEST]',
                '\U0001f504': '[RETRY]',
                '\u2753': '[?]',
                '\u2757': '[!]',
                '\u2795': '[+]',
                '\u2796': '[-]',
                '\u2705': '[DONE]',
                '\U0001f6a7': '[!]',
                '\U0001f4a1': '[i]',
                '\U0001f4ca': '[DATA]',
                '\U0001f4dd': '[NOTE]',
                '\U0001f4c4': '[FILE]',
                '\U0001f4c1': '[FOLDER]',
                '\U0001f50e': '[INSPECT]',
                '\U0001f3af': '[TARGET]',
                '\U0001f4ad': '[THINK]',
                '\U0001f4a0': '[DIAMOND]',
                '\U0001f4cc': '[PIN]',
                '\U0001f4cb': '[CLIP]',
                '\U0001f4c8': '[CHART]',
                '\U0001f4c9': '[DOWN]',
                '\U0001f4c7': '[CARD]',
                '\U0001f4c6': '[CAL]',
                '\U0001f4c5': '[DATE]',
                '\U0001f4c3': '[PAGE]',
                '\U0001f4c2': '[OPEN]',
                '\U0001f4bf': '[CD]',
                '\U0001f4bb': '[PC]',
                '\U0001f4f1': '[PHONE]',
                '\U0001f4f7': '[CAM]',
                '\U0001f4f8': '[PHOTO]',
                '\U0001f4e6': '[PACK]',
                '\U0001f4e5': '[OUT]',
                '\U0001f4e4': '[IN]',
                '\U0001f4e3': '[MEG]',
                '\U0001f4e2': '[BELL]',
                '\U0001f4e1': '[SAT]',
                '\U0001f4e0': '[MAIL]',
                '\U0001f4dc': '[SCROLL]',
                '\U0001f4d6': '[BOOK]',
                '\U0001f4d5': '[NOTE]',
                '\U0001f4d1': '[TAB]',
                '\U0001f4d0': '[TRI]',
                '\U0001f4cf': '[RUL]',
                '\U0001f4ce': '[MARK]',
                '\U0001f50b': '[BAT]',
                '\U0001f50a': '[VOL]',
                '\U0001f509': '[LOW]',
                '\U0001f50c': '[PLUG]',
                '\U0001f511': '[KEY]',
                '\U0001f512': '[LOCK]',
                '\U0001f513': '[UNLOCK]',
                '\U0001f514': '[BELL]',
                '\U0001f515': '[NOBELL]',
                '\U0001f516': '[BOOK]',
                '\U0001f517': '[LINK]',
                '\U0001f518': '[RADIO]',
                '\U0001f519': '[BACK]',
                '\U0001f51a': '[END]',
                '\U0001f51b': '[FAST]',
                '\U0001f51c': '[FAST]',
                '\U0001f51d': '[TOP]',
                '\U0001f51e': '[NO]',
                '\U0001f51f': '[NEW]',
                '\U0001f520': '[ABC]',
                '\U0001f521': '[ABC]',
                '\U0001f522': '[123]',
                '\U0001f523': '[SYM]',
                '\U0001f524': '[ABC]',
                '\U0001f525': '[FIRE]',
                '\U0001f526': '[TORCH]',
                '\U0001f527': '[WRENCH]',
                '\U0001f528': '[HAMMER]',
                '\U0001f529': '[NUT]',
                '\U0001f52a': '[KNIFE]',
                '\U0001f52b': '[GUN]',
                '\U0001f52c': '[MIC]',
                '\U0001f52d': '[TEL]',
                '\U0001f52e': '[CRYSTAL]',
                '\U0001f52f': '[STAR]',
                '\U0001f530': '[SIX]',
                '\U0001f531': '[TRI]',
                '\U0001f532': '[SQUARE]',
                '\U0001f533': '[SQUARE]',
                '\U0001f534': '[RED]',
                '\U0001f535': '[BLUE]',
                '\U0001f536': '[ORANGE]',
                '\U0001f537': '[ORANGE]',
                '\U0001f538': '[ORANGE]',
                '\U0001f539': '[BLUE]',
                '\U0001f53a': '[RED]',
                '\U0001f53b': '[TRI]',
                '\U0001f53c': '[TRI]',
                '\U0001f53d': '[DOWN]',
                '\U0001f53e': '[UP]',
                '\U0001f53f': '[UP]',
                '\U0001f540': '[DIAMOND]',
                '\U0001f541': '[DIAMOND]',
                '\U0001f542': '[DROP]',
                '\U0001f543': '[DROP]',
                '\U0001f544': '[DROP]',
                '\U0001f545': '[SYM]',
                '\U0001f546': '[CHURCH]',
                '\U0001f547': '[CHURCH]',
                '\U0001f548': '[CHURCH]',
                '\U0001f549': '[OM]',
                '\U0001f54a': '[DOVE]',
                '\U0001f54b': '[JEW]',
                '\U0001f54c': '[MOSQUE]',
                '\U0001f54d': '[SYN]',
                '\U0001f54e': '[MEN]',
                '\U0001f550': '[CLOCK]',
                '\U0001f551': '[CLOCK]',
                '\U0001f552': '[CLOCK]',
                '\U0001f553': '[CLOCK]',
                '\U0001f554': '[CLOCK]',
                '\U0001f555': '[CLOCK]',
                '\U0001f556': '[CLOCK]',
                '\U0001f557': '[CLOCK]',
                '\U0001f558': '[CLOCK]',
                '\U0001f559': '[CLOCK]',
                '\U0001f55a': '[CLOCK]',
                '\U0001f55b': '[CLOCK]',
                '\U0001f55c': '[CLOCK]',
                '\U0001f55d': '[CLOCK]',
                '\U0001f55e': '[CLOCK]',
                '\U0001f55f': '[CLOCK]',
                '\U0001f560': '[CLOCK]',
                '\U0001f561': '[CLOCK]',
                '\U0001f562': '[CLOCK]',
                '\U0001f563': '[CLOCK]',
                '\U0001f564': '[CLOCK]',
                '\U0001f565': '[CLOCK]',
                '\U0001f566': '[CLOCK]',
                '\U0001f567': '[CLOCK]',
                '\u00a9': '(c)',
                '\u00ae': '(r)',
                '\u2122': '(tm)',
                '\u2013': '-',
                '\u2014': '--',
                '\u2018': "'",
                '\u2019': "'",
                '\u201c': '"',
                '\u201d': '"',
                '\u2026': '...',
                '\u00b0': 'deg',
            }
            for old, new in replacements.items():
                s = s.replace(old, new)
            # Remove any remaining non-ASCII characters
            s = s.encode('ascii', 'replace').decode('ascii')
            return s
        args = [safe_str(a) for a in args]
        kwargs_str = {k: safe_str(v) for k, v in kwargs.items()}
        print(*args, **kwargs_str)

def safe_analyze_model(model_path):
    """Wrapper for tf.lite.experimental.Analyzer with error handling."""
    try:
        if hasattr(tf.lite, 'experimental') and hasattr(tf.lite.experimental, 'Analyzer'):
            return tf.lite.experimental.Analyzer.analyze(model_path=model_path)
        return None
    except Exception as e:
        print(f"[Warning] Analyzer failed: {str(e)}", file=sys.stderr)
        return None

def model_summary(interpreter):
    """Keras-style model summary for TFLite models"""
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    tensor_details = interpreter.get_tensor_details()
    ops_details = interpreter._get_ops_details()
    
    # Calculate memory usage and parameters
    total_memory = 0
    total_params = 0
    layer_info = []
    
    # Group tensors by operations
    op_tensors = {}
    for op in ops_details:
        op_name = op['op_name']
        if op_name not in op_tensors:
            op_tensors[op_name] = []
        op_tensors[op_name].extend(op['inputs'] + op['outputs'])
    
    # Analyze each operation as a "layer"
    for op in ops_details:
        op_name = op['op_name']
        
        # Calculate output shape
        output_shape = "Unknown"
        if op['outputs']:
            first_output_idx = op['outputs'][0]
            if first_output_idx < len(tensor_details):
                output_shape = tensor_details[first_output_idx]['shape']
        
        # Calculate parameters (for weights/biases)
        op_params = 0
        op_memory = 0
        
        for tensor_idx in op['inputs'] + op['outputs']:
            if tensor_idx < len(tensor_details):
                tensor = tensor_details[tensor_idx]
                dtype_size = tf.dtypes.as_dtype(tensor['dtype']).size
                num_elements = 1
                for dim in tensor['shape']:
                    if dim is not None:
                        num_elements *= dim
                
                tensor_memory = dtype_size * num_elements
                op_memory += tensor_memory
                
                # Count as parameters if it's a weight tensor
                if (tensor['name'] and 
                    any(keyword in tensor['name'].lower() for keyword in ['weight', 'bias', 'kernel'])):
                    op_params += num_elements
        
        layer_info.append({
            'layer_type': op_name,
            'output_shape': output_shape,
            'params': op_params,
            'memory_kb': op_memory / 1024
        })
        total_params += op_params
        total_memory += op_memory
    
    # Build summary table
    summary_lines = []
    summary_lines.append("\n" + "=" * 80)
    summary_lines.append("MODEL SUMMARY: TFLite Model")
    summary_lines.append("=" * 80)
    summary_lines.append(f"{'Layer (type)':<25} {'Output Shape':<20} {'Param #':<15} {'Size (KB)':<10}")
    summary_lines.append("-" * 80)
    
    for i, info in enumerate(layer_info):
        layer_name = f"{info['layer_type']}_{i}"
        output_shape_str = str(info['output_shape']).replace("(", "[").replace(")", "]")
        params_str = f"{info['params']:,}" if info['params'] > 0 else "0"
        memory_str = f"{info['memory_kb']:.2f}"
        
        summary_lines.append(f"{layer_name:<25} {output_shape_str:<20} {params_str:<15} {memory_str:<10}")
    
    # Add totals
    summary_lines.append("-" * 80)
    summary_lines.append(f"Total params: {total_params:,}")
    summary_lines.append(f"Total memory: {total_memory / 1024:.2f} KB")
    summary_lines.append(f"Total operations: {len(ops_details)}")
    
    # Input/Output summary
    summary_lines.append("\n" + "=" * 80)
    summary_lines.append("INPUT/OUTPUT SUMMARY")
    summary_lines.append("=" * 80)
    
    for i, inp in enumerate(input_details):
        shape_str = str(inp['shape']).replace("(", "[").replace(")", "]")
        dtype_str = str(inp['dtype'])
        summary_lines.append(f"Input {i}:  {shape_str:<15} {dtype_str:<10}")
    
    for i, out in enumerate(output_details):
        shape_str = str(out['shape']).replace("(", "[").replace(")", "]")
        dtype_str = str(out['dtype'])
        summary_lines.append(f"Output {i}: {shape_str:<15} {dtype_str:<10}")
    
    return '\n'.join(summary_lines)

def get_detailed_operations_summary(interpreter):
    """More detailed operations breakdown"""
    ops_details = interpreter._get_ops_details()
    tensor_details = interpreter.get_tensor_details()
    
    summary_lines = []
    summary_lines.append("\n" + "=" * 80)
    summary_lines.append("DETAILED OPERATIONS BREAKDOWN")
    summary_lines.append("=" * 80)
    
    # Count operations by type
    op_counts = {}
    for op in ops_details:
        op_name = op['op_name']
        op_counts[op_name] = op_counts.get(op_name, 0) + 1
    
    summary_lines.append("\nOPERATION COUNTS:")
    for op_name, count in sorted(op_counts.items()):
        summary_lines.append(f"  {op_name}: {count}")
    
    # Detailed operation info
    summary_lines.append("\nDETAILED OPERATION INFO:")
    for op in ops_details:
        summary_lines.append(f"\n{op['op_name']} (index: {op['index']}):")
        summary_lines.append(f"  Input tensors: {op['inputs']}")
        summary_lines.append(f"  Output tensors: {op['outputs']}")
        
        # Show tensor details
        for tensor_idx in op['inputs']:
            if tensor_idx < len(tensor_details):
                tensor = tensor_details[tensor_idx]
                shape_str = str(tensor['shape'])
                summary_lines.append(f"    Input {tensor_idx}: shape={shape_str}, dtype={tensor['dtype']}")
        
        for tensor_idx in op['outputs']:
            if tensor_idx < len(tensor_details):
                tensor = tensor_details[tensor_idx]
                shape_str = str(tensor['shape'])
                summary_lines.append(f"    Output {tensor_idx}: shape={shape_str}, dtype={tensor['dtype']}")
    
    return '\n'.join(summary_lines)

def get_memory_analysis(interpreter):
    """Detailed memory usage analysis"""
    tensor_details = interpreter.get_tensor_details()
    
    summary_lines = []
    summary_lines.append("\n" + "=" * 80)
    summary_lines.append("MEMORY USAGE ANALYSIS")
    summary_lines.append("=" * 80)
    
    memory_breakdown = []
    total_memory = 0
    
    for tensor in tensor_details:
        dtype_size = tf.dtypes.as_dtype(tensor['dtype']).size
        num_elements = 1
        for dim in tensor['shape']:
            if dim is not None:
                num_elements *= dim
        
        tensor_memory = dtype_size * num_elements
        total_memory += tensor_memory
        
        memory_breakdown.append({
            'name': tensor.get('name', 'unnamed'),
            'shape': tensor['shape'],
            'dtype': tensor['dtype'],
            'memory_kb': tensor_memory / 1024,
            'elements': num_elements
        })
    
    # Sort by memory usage
    memory_breakdown.sort(key=lambda x: x['memory_kb'], reverse=True)
    
    summary_lines.append(f"{'Tensor Name':<30} {'Shape':<20} {'Dtype':<10} {'Elements':<12} {'Memory (KB)':<12}")
    summary_lines.append("-" * 80)
    
    for tensor_info in memory_breakdown[:20]:  # Show top 20
        shape_str = str(tensor_info['shape'])[:18] + "..." if len(str(tensor_info['shape'])) > 18 else str(tensor_info['shape'])
        summary_lines.append(
            f"{tensor_info['name'][:28]:<30} {shape_str:<20} {str(tensor_info['dtype']):<10} "
            f"{tensor_info['elements']:<12,} {tensor_info['memory_kb']:<12.2f}"
        )
    
    summary_lines.append("-" * 80)
    summary_lines.append(f"Total Memory Usage: {total_memory / 1024:.2f} KB")
    summary_lines.append(f"Number of Tensors: {len(tensor_details)}")
    
    return '\n'.join(summary_lines)


def check_delegate_ops(interpreter):
    """Check if the model contains DELEGATE operations (not supported by TFLite Micro)"""
    ops_details = interpreter._get_ops_details()
    delegate_ops = [op for op in ops_details if op['op_name'] == 'DELEGATE']
    
    summary_lines = []
    summary_lines.append("\n" + "=" * 80)
    summary_lines.append("TFLITE MICRO COMPATIBILITY CHECK")
    summary_lines.append("=" * 80)
    
    if delegate_ops:
        summary_lines.append(f"\n[WARN] Found {len(delegate_ops)} DELEGATE operation(s)!")
        summary_lines.append("   TFLite Micro does NOT support delegate operations.")
        summary_lines.append("   This model will FAIL to load on microcontrollers.")
        summary_lines.append("   The model was likely exported with XNNPACK delegate enabled.")
        summary_lines.append("   Re-export with: converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]")
        summary_lines.append("   Or disable delegates during conversion.")
        for op in delegate_ops:
            summary_lines.append(f"\n   DELEGATE op index {op['index']}:")
            summary_lines.append(f"     Inputs: {op['inputs']}")
            summary_lines.append(f"     Outputs: {op['outputs']}")
    else:
        summary_lines.append("\n[OK] No DELEGATE operations found - compatible with TFLite Micro")
    
    # Check total ops vs MAX_OPERATORS
    total_ops = len(ops_details)
    # Use total_ops + 5 as recommended MAX_OPERATORS (safety margin)
    recommended_max_ops = total_ops + 5
    summary_lines.append(f"\n[INFO] Model has {total_ops} operations")
    summary_lines.append(f"   Recommended MAX_OPERATORS: {recommended_max_ops} (total ops + 5 margin)")
    summary_lines.append(f"   Set via build flag: -DMAX_OPERATORS={recommended_max_ops}")
    
    # Check unique operator types
    unique_ops = set(op['op_name'] for op in ops_details)
    summary_lines.append(f"\nUnique operator types: {len(unique_ops)}")
    for op_name in sorted(unique_ops):
        count = sum(1 for op in ops_details if op['op_name'] == op_name)
        summary_lines.append(f"  {op_name}: {count}")
    
    return '\n'.join(summary_lines)


def get_peak_memory_analysis(interpreter):
    """Analyze peak memory usage using lifetime analysis (simulates TFLite Micro arena)"""
    ops_details = interpreter._get_ops_details()
    tensor_details = interpreter.get_tensor_details()
    
    summary_lines = []
    summary_lines.append("\n" + "=" * 80)
    summary_lines.append("PEAK MEMORY ANALYSIS (TFLite Micro Arena Estimate)")
    summary_lines.append("=" * 80)
    
    # Build tensor lifetime map
    # For each tensor, find first op that uses it as output and last op that uses it as input
    tensor_first_use = {}
    tensor_last_use = {}
    
    for op_idx, op in enumerate(ops_details):
        for tensor_idx in op['inputs']:
            if tensor_idx >= 0:
                if tensor_idx not in tensor_first_use:
                    tensor_first_use[tensor_idx] = op_idx
                tensor_last_use[tensor_idx] = op_idx
        for tensor_idx in op['outputs']:
            if tensor_idx >= 0:
                if tensor_idx not in tensor_first_use:
                    tensor_first_use[tensor_idx] = op_idx
                tensor_last_use[tensor_idx] = op_idx
    
    # Calculate memory at each op boundary
    peak_memory = 0
    peak_op = 0
    memory_by_op = []
    
    for op_idx in range(len(ops_details)):
        active_memory = 0
        for tensor_idx, tensor in enumerate(tensor_details):
            if tensor_idx in tensor_first_use and tensor_idx in tensor_last_use:
                if tensor_first_use[tensor_idx] <= op_idx <= tensor_last_use[tensor_idx]:
                    dtype_size = tf.dtypes.as_dtype(tensor['dtype']).size
                    num_elements = 1
                    for dim in tensor['shape']:
                        if dim is not None:
                            num_elements *= dim
                    active_memory += dtype_size * num_elements
        
        memory_by_op.append(active_memory)
        if active_memory > peak_memory:
            peak_memory = active_memory
            peak_op = op_idx
    
    total_memory = sum(
        tf.dtypes.as_dtype(t['dtype']).size * 
        (lambda s: 1 if not any(s) else (lambda: __import__('functools').reduce(lambda a,b: a*b, [d for d in s if d is not None], 1))())(t['shape'])
        for t in tensor_details
    )
    
    # Actually compute total properly
    total_memory = 0
    for t in tensor_details:
        dtype_size = tf.dtypes.as_dtype(t['dtype']).size
        num_elements = 1
        for dim in t['shape']:
            if dim is not None:
                num_elements *= dim
        total_memory += dtype_size * num_elements
    
    summary_lines.append(f"\nTotal memory (sum of all tensors): {total_memory / 1024:.2f} KB")
    summary_lines.append(f"Peak active memory (lifetime analysis): {peak_memory / 1024:.2f} KB at op index {peak_op}")
    summary_lines.append(f"TFLite Micro arena estimate (1.5x peak): {peak_memory * 1.5 / 1024:.2f} KB")
    summary_lines.append(f"\nRecommended tensor_arena_size: {int(peak_memory * 1.5 / 1024) + 1}KB")
    
    # Show top tensors by memory
    tensor_memory = []
    for idx, t in enumerate(tensor_details):
        dtype_size = tf.dtypes.as_dtype(t['dtype']).size
        num_elements = 1
        for dim in t['shape']:
            if dim is not None:
                num_elements *= dim
        tensor_memory.append((idx, dtype_size * num_elements / 1024, t['shape'], t['dtype'], t.get('name', '')))
    
    tensor_memory.sort(key=lambda x: x[1], reverse=True)
    
    summary_lines.append(f"\nTop tensors by memory:")
    for idx, mem_kb, shape, dtype, name in tensor_memory[:10]:
        summary_lines.append(f"  [{idx:3d}] {mem_kb:8.2f} KB  {str(shape):<25} {str(dtype):<20} {name[:50]}")
    
    return '\n'.join(summary_lines)


def evaluate_tflite_model(tflite_path, x_test=None, y_test=None):
    """Evaluate TFLite model accuracy if test data is provided"""
    if x_test is None or y_test is None:
        return None
    
    safe_print("[TEST] Evaluating TFLite model accuracy...")
    
    try:
        interpreter = tf.lite.Interpreter(model_path=tflite_path)
        interpreter.allocate_tensors()
        
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        
        input_dtype = input_details[0]['dtype']
        
        correct_predictions = 0
        total_samples = len(x_test)
        all_predictions = []
        all_confidences = []
        
        for i in tqdm(range(total_samples), desc="Evaluating", leave=False):
            input_data = x_test[i:i+1]
            
            # Handle quantization
            if input_dtype in [np.int8, np.uint8]:
                input_scale, input_zero_point = input_details[0]['quantization']
                input_data = input_data / input_scale + input_zero_point
                input_data = input_data.astype(input_dtype)
            else:
                input_data = input_data.astype(np.float32)
            
            interpreter.set_tensor(input_details[0]['index'], input_data)
            interpreter.invoke()
            
            output_data = interpreter.get_tensor(output_details[0]['index'])
            
            # Handle output quantization
            if output_details[0]['dtype'] in [np.int8, np.uint8]:
                output_scale, output_zero_point = output_details[0]['quantization']
                output_data = (output_data.astype(np.float32) - output_zero_point) * output_scale
            
            predicted_class = np.argmax(output_data)
            confidence = np.max(output_data)
            
            all_predictions.append(predicted_class)
            all_confidences.append(confidence)
            
            # Get true class
            if len(y_test.shape) > 1 and y_test.shape[1] > 1:
                true_class = np.argmax(y_test[i])
            else:
                true_class = y_test[i]
            
            if predicted_class == true_class:
                correct_predictions += 1
        
        accuracy = correct_predictions / total_samples
        
        # Additional metrics
        avg_confidence = np.mean(all_confidences)
        confidence_std = np.std(all_confidences)
        
        return {
            'accuracy': accuracy,
            'correct_predictions': correct_predictions,
            'total_samples': total_samples,
            'avg_confidence': avg_confidence,
            'confidence_std': confidence_std,
            'predictions': np.array(all_predictions),
            'confidences': np.array(all_confidences)
        }
    
    except Exception as e:
        safe_print(f"[FAIL] Evaluation failed: {e}")
        return None

def inspect_tflite_model(model_path, verbose=False, output_file=None, 
                        evaluate=False, test_data=None, debug=False,
                        summary_type="basic"):
    """Enhanced TFLite model inspection with comprehensive analysis"""
    try:
        # Load model WITHOUT default delegates (XNNPACK) to avoid false positives
        # in delegate detection. Using default delegates wraps builtin ops in
        # DELEGATE at runtime, making _get_ops_details() report DELEGATE even
        # when the flatbuffer is clean.
        interpreter = tf.lite.Interpreter(
            model_path=model_path,
            experimental_op_resolver_type=tf.lite.experimental.OpResolverType.BUILTIN_WITHOUT_DEFAULT_DELEGATES
        )
        interpreter.allocate_tensors()
        
        # Initialize output content
        output_content = []
        
        # --- Report Header ---
        header = [
            f"\n{'='*80}",
            "TFLITE MODEL COMPREHENSIVE INSPECTION REPORT",
            f"{'='*80}",
            f"Model: {model_path}",
            f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            f"TensorFlow Version: {tf.__version__}",
            f"File Size: {os.path.getsize(model_path) / 1024:.2f} KB"
        ]
        output_content.extend(header)
        
        # --- Model Summary (Keras-style) ---
        output_content.append(model_summary(interpreter))
        
        # --- Memory Analysis ---
        output_content.append(get_memory_analysis(interpreter))
        
        # --- Peak Memory Analysis (TFLite Micro Arena Estimate) ---
        output_content.append(get_peak_memory_analysis(interpreter))
        
        # --- TFLite Micro Compatibility Check ---
        output_content.append(check_delegate_ops(interpreter))
        
        # --- Detailed Operations Summary ---
        if verbose:
            output_content.append(get_detailed_operations_summary(interpreter))
        
        # --- Analyzer Output ---
        analyzer_output = safe_analyze_model(model_path)
        if analyzer_output:
            output_content.append("\n[EXPERIMENTAL ANALYZER OUTPUT]\n" + analyzer_output)
        
        # --- Evaluation ---
        if evaluate and test_data:
            x_test, y_test = test_data
            eval_results = evaluate_tflite_model(model_path, x_test, y_test)
            if eval_results:
                output_content.append("\n" + "="*80)
                output_content.append("MODEL EVALUATION RESULTS")
                output_content.append("="*80)
                output_content.append(f"Accuracy: {eval_results['accuracy']:.4f} "
                                    f"({eval_results['correct_predictions']}/{eval_results['total_samples']})")
                output_content.append(f"Average Confidence: {eval_results['avg_confidence']:.4f} "
                                    f"(±{eval_results['confidence_std']:.4f})")
        
        # --- Output Results ---
        report = '\n'.join(output_content)
        safe_print(report)
        
        if output_file:
            with open(output_file, 'w') as f:
                f.write(report)
            safe_print(f"\n[SAVE] Comprehensive report saved to: {output_file}")

    except Exception as e:
        safe_print(f"\n[FAIL] Error inspecting model: {str(e)}")
        sys.exit(1)

def create_sample_data(input_shape, num_classes=10):
    """Create sample test data for evaluation"""
    num_samples = 100
    x_sample = np.random.random((num_samples,) + input_shape).astype(np.float32)
    y_sample = np.random.randint(0, num_classes, num_samples)
    return x_sample, y_sample

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Enhanced TFLite model inspection with Keras-style summary",
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
        help="Show detailed operation info and tensor details"
    )
    parser.add_argument(
        "--output_file",
        type=str,
        default=None,
        help="Save comprehensive report to a file"
    )
    parser.add_argument(
        "--evaluate",
        action="store_true",
        help="Evaluate model accuracy (requires --test_data or will use random data)"
    )
    parser.add_argument(
        "--test_data",
        type=str,
        default=None,
        help="Path to numpy file with test data (x_test.npy, y_test.npy)"
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Run debug analysis with sample inference"
    )
    parser.add_argument(
        "--input_shape",
        type=str,
        default="28,28,1",
        help="Input shape for model (comma-separated, e.g., '28,28,1')"
    )
    
    args = parser.parse_args()
    
    # Prepare test data if needed
    test_data = None
    if args.evaluate:
        if args.test_data:
            # Load test data from file
            try:
                x_test = np.load(os.path.join(args.test_data, 'x_test.npy'))
                y_test = np.load(os.path.join(args.test_data, 'y_test.npy'))
                test_data = (x_test, y_test)
            except Exception as e:
                safe_print(f"[FAIL] Failed to load test data: {e}")
                safe_print("[RETRY] Using random sample data instead...")
                input_shape = tuple(map(int, args.input_shape.split(',')))
                test_data = create_sample_data(input_shape)
        else:
            # Create sample data
            safe_print("[RETRY] Using random sample data for evaluation...")
            input_shape = tuple(map(int, args.input_shape.split(',')))
            test_data = create_sample_data(input_shape)
    
    inspect_tflite_model(
        model_path=args.model_path,
        verbose=args.verbose,
        output_file=args.output_file,
        evaluate=args.evaluate,
        test_data=test_data,
        debug=args.debug
    )
    
# Basic inspection
# py check_tflite_model.py ../models/digit_recognizer_v4_10cls_RGB.tflite

# Comprehensive analysis with evaluation
# python check_tflite_model.py ../models/digit_recognizer_v4_10cls_RGB.tflite --verbose --evaluate --debug

# Save detailed report to file
# python check_tflite_model.py ../models/digit_recognizer_v4_10cls_RGB.tflite --output_file digit_recognizer_v4_10cls_RGB.txt --verbose

# Evaluate with custom test data
# python check_tflite_model.py ../models/digit_recognizer_v4_10cls_RGB.tflite --evaluate --test_data /path/to/test_data