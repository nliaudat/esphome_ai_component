# Data Extractor Pipeline

The `data_extractor` provides a suite of tools built to process raw images collected by the `data_collector` server. The goal of this pipeline is to parse EXIF metadata containing inference data, extract the specific digit zones, deduplicate the datasets, and cross-reference them against higher-accuracy neural network models to identify and correct inference errors.

This allows you to generate robust training datasets to improve your edge AI models.

## Workflow Scripts

### 1. `1_extractor.py`
This script processes the raw images saved by your server. It reads the custom EXIF JSON payload injected by the `app.py` server to understand what the ESP32 inferred at the time of capture. 
- **Rotates** the image based on EXIF orientation.
- **Crops** out individual digit zones using the bounding boxes in the metadata.
- **Saves** the cropped digit zones to an `extracted/` directory.

**Filename Format generated:**
`[inference_detection]_[confidence]_[timestamp]_[random_string].jpg`

### 2. `2_deduplicate.py`
Training datasets often become bloated with visually identical or extremely similar images (e.g., a dial hovering between two digits for an hour). 
- Recursively scans the `extracted/` folder.
- Uses **MD5 hashing** to remove exact duplicate files.
- Uses **Perceptual Hashing (pHash)** to group visually similar images that have the *exact same* inferred value.
- Keeps only the newest image from each similar group, significantly reducing dataset size without losing variant data.

### 3. `3_correct_errors.py`
This script is used for the final validation pass. It runs a specified TensorFlow Lite model against the `extracted/` images to verify the original on-device inferences.
- Recursively processes images in `extracted/`.
- Parses the original inference value from the filename.
- Runs the image through the selected `.tflite` model.
- **Error Correction:** If the new model's prediction differs from the original inference (e.g., the ESP32 guessed `1` but the new model confidently guesses `7`), the image is copied to a `training/` folder for review.

**Corrected Filename Format:**
`[new_inference]_[new_confidence]_[old_inference]_err_corr.[ext]`

### 4. `4_clean_duplicates_training.py`
Once your error dataset is generated in `training/`, this script runs a final deduplication pass specifically on the mismatched images.
- Validates the `[new_inference]` value to ensure it only deduplicates identically categorized errors.
- Groups visually similar errors utilizing Perceptual Hashing (pHash) and MD5 hashing.
- Discards duplicates, saving substantial dataset review time.

### 5. `Visual Validation` (Important)
Before utilizing the dataset, you **must** manually review the images within the `training/` folder to ensure the new AI model's "corrected" guesses are actually accurate. Delete or manually rename any files where the larger model also guessed incorrectly.

### 6. `Retraining`
Once your dataset is visually validated, upload it as a zip file to the [digit_recognizer repository](https://github.com/nliaudat/digit_recognizer) (via issues or pull requests) to contribute to the global model, or use the tooling there to retrain your own custom model locally.

## Usage Example

Assuming your raw images are in `../server/uploads`:

```bash
# Install requirements
pip install -r requirements.txt

# 1. Extract digit zones
python 1_extractor.py --input ../server/uploads --output extracted

# 2. Deduplicate the dataset
python 2_deduplicate.py --folder extracted --delete

# 3. Find inference mismatches and export them to training/
python 3_correct_errors.py --folder extracted --model super_high_accuracy_validator_100cls_RGB

# 4. Clean duplicates inside the training/ directory
python 4_clean_duplicates_training.py

# 5. Visually validate the images in the training/ folder!

# 6. Zip and upload the validated dataset to https://github.com/nliaudat/digit_recognizer
zip -r training_dataset.zip training/
```

The resulting `training/` folder will contain the critical edge cases where your models disagree, providing the perfect dataset for fine-tuning.
