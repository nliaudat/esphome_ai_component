"""List all operations in a TFLite model."""
import tensorflow as tf
import sys

model_path = sys.argv[1]
interpreter = tf.lite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()

ops = interpreter._get_ops_details()
for o in ops:
    print(f"Op {o['index']:2d}: {o['op_name']}")

print(f"\nTotal ops: {len(ops)}")

# Also check operator codes
model = interpreter._get_model()
if hasattr(model, 'operator_codes'):
    codes = model.operator_codes
    print(f"\nOperator codes ({len(codes)}):")
    for i, code in enumerate(codes):
        print(f"  [{i}] builtin_code={code.builtin_code}")
