"""Analyze peak memory usage of a TFLite model using tensor lifetime analysis."""
import tensorflow as tf
import numpy as np
import sys

def analyze_peak_memory(model_path):
    interpreter = tf.lite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()

    tensor_details = interpreter.get_tensor_details()
    ops_details = interpreter._get_ops_details()

    # Build tensor lifetime: first and last op index where each tensor is used
    tensor_first = {}
    tensor_last = {}
    for op in ops_details:
        for t_idx in op['inputs']:
            if t_idx not in tensor_first:
                tensor_first[t_idx] = op['index']
            tensor_last[t_idx] = op['index']
        for t_idx in op['outputs']:
            if t_idx not in tensor_first:
                tensor_first[t_idx] = op['index']
            tensor_last[t_idx] = op['index']

    # Calculate peak memory at each op boundary
    peak_memory = 0
    peak_op = -1
    memory_by_op = []
    
    for op_idx in range(len(ops_details)):
        active_memory = 0
        for t_idx, tensor in enumerate(tensor_details):
            if t_idx in tensor_first and t_idx in tensor_last:
                if tensor_first[t_idx] <= op_idx <= tensor_last[t_idx]:
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

    # Also calculate TFLite Micro arena estimate (adds overhead for scratch buffers)
    # TFLM typically needs ~1.5x the peak tensor memory for scratch/overhead
    tflm_estimate = peak_memory * 1.5

    print(f"Model: {model_path}")
    print(f"File size: {interpreter._get_ops_details()[-1]['index']}")
    print(f"Input type: {tensor_details[0]['dtype']}")
    print(f"Input shape: {tensor_details[0]['shape']}")
    print(f"Number of tensors: {len(tensor_details)}")
    print(f"Number of ops: {len(ops_details)}")
    
    # Check for DELEGATE operations (not supported by TFLite Micro)
    delegate_ops = [op for op in ops_details if op['op_name'] == 'DELEGATE']
    if delegate_ops:
        print(f"\n[WARN] Found {len(delegate_ops)} DELEGATE operation(s)!")
        print(f"   TFLite Micro does NOT support delegate operations.")
        print(f"   This model will FAIL to load on microcontrollers.")
        print(f"   Re-export with delegates disabled.")
    print(f"")
    print(f"Total memory (sum of all tensors): {sum(tf.dtypes.as_dtype(t['dtype']).size * np.prod([d for d in t['shape'] if d is not None]) for t in tensor_details) / 1024:.2f} KB")
    print(f"Peak active memory (lifetime analysis): {peak_memory / 1024:.2f} KB at op index {peak_op}")
    print(f"TFLite Micro arena estimate (1.5x peak): {tflm_estimate / 1024:.2f} KB")
    print(f"")
    print(f"Recommended tensor_arena_size: {int(np.ceil(tflm_estimate / 1024)) + 1}KB")
    
    # Show top memory tensors
    print(f"\nTop tensors by memory:")
    tensor_mem = []
    for t_idx, tensor in enumerate(tensor_details):
        dtype_size = tf.dtypes.as_dtype(tensor['dtype']).size
        num_elements = 1
        for dim in tensor['shape']:
            if dim is not None:
                num_elements *= dim
        mem = dtype_size * num_elements
        tensor_mem.append((mem, t_idx, tensor['name'], tensor['shape'], tensor['dtype']))
    
    tensor_mem.sort(reverse=True)
    for mem, t_idx, name, shape, dtype in tensor_mem[:15]:
        print(f"  [{t_idx:2d}] {mem/1024:8.2f} KB  {str(shape):20s}  {str(dtype):20s}  {name}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python analyze_peak_memory.py <model.tflite>")
        sys.exit(1)
    analyze_peak_memory(sys.argv[1])
