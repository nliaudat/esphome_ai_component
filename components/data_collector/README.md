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

### Deployment with Docker (Synology/Portainer)

A complete Docker solution is provided in the `components/data_collector/server` directory.

#### 1. Copy Files to Server
Create a folder on your NAS at `/volume1/docker/esphome-data-collector` (this matches the `docker-compose.yml` volume mapping) and upload the following files from `components/data_collector/server`:
- `synology_container_compose.yaml` (rename to `docker-compose.yml` or use explicitly)
- `docker-compose.yml` (Generic version)
- `Dockerfile`
- `app.py`
- `requirements.txt`

#### 2. Configure & Run
SSH into your NAS or use Portainer to run the stack.

**For Synology Users:**
Use `synology_container_compose.yaml`. This version uses `network_mode: host` and maps the entire `/volume1/docker/esphome-data-collector` directory.

```bash
cd /volume1/docker/esphome-data-collector
# Rename if you want to use default command, or specify file:
docker-compose -f synology_container_compose.yaml up -d --build
```

**For Standard Docker:**
Use the standard `docker-compose.yml`.

```bash
docker-compose up -d --build
```

**Using Portainer:**
1. Create a new stack.
2. Paste the contents of `docker-compose.yml`.
3. Ensure the `build: .` context is handled or just use the local file method above.

The server will start on port `5123`.

#### 3. Mandatory Authentication
You **must** set the `API_KEY` environment variable in your compose file to a secure secret. The server will fail to start without it.

```yaml
    environment:
      - FLASK_ENV=production
      - API_KEY=my-secret-token
```

Then update your ESPHome config:

```yaml
data_collector:
  id: ${id_prefix}_data_collector
  upload_url: "http://192.168.1.50:5123/api/upload/${id_prefix}"
  api_key: "change-me-to-a-secure-key"
```

The server will automatically create a subfolder named after the prefix (e.g., `uploads/${id_prefix}/`) to store the images.

## 🛠️ Data Extractor Processing Pipeline

Once you have accumulated a substantial amount of raw images on your server, you can use the tooling inside the `components/data_collector/data_extractor` folder to process, deduplicate, and validate these images for training.

The workflow consists of six main stages:
1. **Extraction** (`1_extractor.py`): Parses the injected EXIF metadata to crop out and isolate individual digit zones.
2. **Deduplication** (`2_deduplicate.py`): Cleans the dataset of exact and perceptual duplicates to prevent bias and reduce bloat.
3. **Error Correction** (`3_correct_errors.py`): Runs the cropped dataset through a larger/newer TFLite model to identify cases where the on-device inference was incorrect, exporting them to a `training/` folder.
4. **Training Deduplication** (`4_clean_duplicates_training.py`): Runs a final deduplication pass specifically on the mismatched images in the `training/` folder.
5. **Visual Validation** (Important): Manually review the images within the `training/` folder to ensure the new AI model's "corrected" guesses are actually accurate. Delete or manually rename any files where the larger model also guessed incorrectly.
6. **Retraining**: Upload the visually validated dataset as a zip file to the [digit_recognizer repository](https://github.com/nliaudat/digit_recognizer) (via issues or pull requests) to contribute to the global model, or use the tooling there to retrain your own custom model locally.

For full instructions, view the [Data Extractor README](data_extractor/README.md).

