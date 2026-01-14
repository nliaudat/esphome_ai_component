# Data Collector Component

The **Data Collector** component is a critical tool for implementing an "Active Learning" loop. It allows the system to automatically identify and export images that are "difficult" for the current model to recognize, enabling you to build a targeted dataset for retraining.

## 🚀 Why use this?

Machine learning models, like the TFLite digit recognizer used in this project, improve significantly when trained on "hard examples"—images where the model is unsure or incorrect.

Instead of manually saving thousands of images, this component works with `meter_reader_tflite` to:
1.  **Monitor Confidence**: Watch the inference results in real-time.
2.  **Detect Uncertainty**: Trigger when the model's confidence falls below your standard threshold.
3.  **Ensure Quality**: Automatically turn on the flashlight and retake the picture to ensure the collected training data is well-lit and high-quality.
4.  **Export**: Upload the image immediately to a server (Webhook/HTTP POST).

By attempting to label these "edge cases" and adding them to your training set (using the [digit_recognizer](https://github.com/nliaudat/digit_recognizer) repo), you can create a robust model that handles shadows, glares, and intermediate digit positions perfectly.

## ⚙️ Configuration

Add the component to your ESPHome configuration:

```yaml
data_collector:
  id: my_data_collector
  # Target Server / Webhook
  upload_url: "http://192.168.1.50:5000/api/upload"
  debug: false
  
  # Optional Authentication
  # api_key: "Bearer my-token"  # Adds Authorization header
  # OR
  # api_key: "my-key"           # Adds X-Api-Key header
  
  # Basic Auth
  # username: "admin"
  # password: "password"
  
  # Optional Switch to Toggle Collection from HA
  web_submit:
    name: "Allow Data Collection"
```

> **Note**: This component is fully optional. If you do not include the `data_collector` block in your YAML, the feature is disabled and no code is compiled.

## 🔗 Integration with Meter Reader

Enable the collection logic in your `meter_reader_tflite` config:

```yaml
meter_reader_tflite:
  ...
  # Key configuration to link the collector
  data_collector: my_data_collector
  collect_low_confidence: true
```

## 📸 The "Retake with Flash" Workflow

When `collect_low_confidence: true` is set, the system follows this smart workflow:

1.  **Inference**: The model reads the meter (e.g., using ambient light or IR).
2.  **Check**: If confidence is low (e.g., < 85%) but not zero:
3.  **Trigger**: The `DataCollector` takes over.
4.  **Flash On**: The flashlight is forcibly turned on.
5.  **Stabilize**: The system waits for the configured `flash_pre_time` (e.g., 5s) to allow the camera to adjust exposure.
6.  **Capture**: A new, high-quality image is captured.
7.  **Upload**: The image is JPEG encoded and sent to your `upload_url` with headers `X-Meter-Value` and `X-Meter-Confidence`.
8.  **Flash Off**: System returns to normal operation.

## 📥 Receiving Images

The component sends a standard HTTP POST request with the raw JPEG image as the body. You can receive this with any web server, Node-RED, or N8N.

### Python (Flask) Example

```python
from flask import Flask, request
import time

app = Flask(__name__)

@app.route('/api/upload', methods=['POST'])
def upload():
    # Save file with timestamp and prediction value
    value = request.headers.get('X-Meter-Value', 'unknown')
    conf = request.headers.get('X-Meter-Confidence', '0.0')
    filename = f"collect_{int(time.time())}_{value}_{float(conf):.2f}.jpg"
    
    with open(filename, "wb") as f:
        f.write(request.data)
        
    print(f"Saved {filename}")
    return "OK", 200
```
