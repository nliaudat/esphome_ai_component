"""
Enhanced Meter Reading Detection Script

This script uses TensorFlow Lite models to detect and read meter values from images.
It supports multiple model types, automatic region detection, and provides detailed output.
"""

import cv2
import numpy as np
import tensorflow as tf
import logging
import argparse
import requests
import os
import sys
import json
import ast
import csv
from typing import List, Tuple, Optional, Dict, Any, Union
from pathlib import Path
from dataclasses import dataclass
from tqdm import tqdm

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('meter_reading.log')
    ]
)
logger = logging.getLogger(__name__)

# Configuration Constants
MODELS_DIR = Path("models").resolve()
DEFAULT_REGIONS_FILE = Path("regions.json")
DEFAULT_MODEL = "super_high_accuracy_validator_100cls_RGB" 
DEFAULT_RESULT_IMAGE = Path("result.jpg")
MAX_IMAGE_SIZE = (1920, 1080)  # For memory safety

@dataclass
class ModelConfig:
    path: Path
    description: str
    output_processing: str
    scale_factor: float
    input_type: str  # 'float32', 'uint8', 'int8' - this determines quantization
    input_channels: int = 3
    input_size: Optional[Tuple[int, int]] = None
    normalize: bool = False
    invert: bool = False

    @property
    def quantized(self) -> bool:
        """Check if model is quantized based on input type"""
        return self.input_type in ['uint8', 'int8']
    
    @property
    def esp_dl_quantization(self) -> bool:
        """Check if model uses ESP-DL quantization (int8)"""
        return self.input_type == 'int8'

# Model Configuration - UPDATED: Added support for default model.tflite
MODELS: Dict[str, ModelConfig] = {
    "model": ModelConfig(  # Default model when no specific model is specified
        path=MODELS_DIR / "model.tflite",
        description="Default model",
        output_processing="softmax_scale10",
        scale_factor=10.0,
        input_type="float32"
    ),
    "class100-0180": ModelConfig(
        path=MODELS_DIR / "dig-class100-0180-s2-q.tflite",
        description="dig-class100-0180",
        output_processing="softmax_scale10",
        scale_factor=10.0,
        input_type="float32"
    ),
    "digit_recognizer_v12_100cls_RGB": ModelConfig(
        path=MODELS_DIR / "digit_recognizer_v12_100cls_RGB.tflite",
        description="digit_recognizer_v12_100cls_RGB",
        output_processing="softmax_scale10",
        scale_factor=10.0,
        input_type="uint8",
        input_channels=3,
        input_size=(32, 20),
        normalize=True,
        invert=False
    ),
    "digit_recognizer_v4_10cls_GRAY": ModelConfig(
        path=MODELS_DIR / "digit_recognizer_v4_10cls_GRAY.tflite",
        description="digit_recognizer_v4 Digit Classifier 10 classes GRAY",
        output_processing="softmax",
        scale_factor=1.0,
        input_type="uint8",  
        input_channels=1,
        input_size=(32, 20),
        normalize=True,
        invert=False
    ),
    "digit_recognizer_v4_10cls_RGB": ModelConfig(
        path=MODELS_DIR / "digit_recognizer_v4_10cls_RGB.tflite",
        description="digit_recognizer_v4 Digit Classifier 10 classes RGB",
        output_processing="softmax",
        scale_factor=1.0,
        input_type="uint8",
        input_channels=3,
        input_size=(32, 20),
        normalize=True,
        invert=False
    ),
    "digit_recognizer_v4_100cls_GRAY": ModelConfig(
        path=MODELS_DIR / "digit_recognizer_v4_100cls_GRAY.tflite",
        description="digit_recognizer_v4 Digit Classifier 100 classes GRAY",
        output_processing="softmax_scale10",
        scale_factor=10.0,
        input_type="uint8",  
        input_channels=1,
        input_size=(32, 20),
        normalize=True,
        invert=False
    ),
    "digit_recognizer_v4_100cls_RGB": ModelConfig(
        path=MODELS_DIR / "digit_recognizer_v4_100cls_RGB.tflite",
        description="digit_recognizer_v4 Digit Classifier 100 classes RGB",
        output_processing="softmax_scale10",
        scale_factor=10.0,
        input_type="uint8",
        input_channels=3,
        input_size=(32, 20),
        normalize=True,
        invert=False
    ),
    "super_high_accuracy_validator_100cls_RGB": ModelConfig(
        path=MODELS_DIR / "super_high_accuracy_validator_100cls_RGB.tflite",
        description="super_high_accuracy_validator Digit Classifier 100 classes RGB",
        output_processing="softmax_scale10",
        scale_factor=10.0,
        input_type="uint8",
        input_channels=3,
        input_size=(32, 20),
        normalize=True,
        invert=False
    ),

}

class MeterReader:
    """Class for reading meter values using TensorFlow Lite models."""
    
    def __init__(self, model_type: str = DEFAULT_MODEL) -> None:
        """Initialize the MeterReader with a specific model type."""
        # NEW: Check if default model.tflite exists when no specific model is specified
        if Path(model_type).is_file():
            # Support loading arbitrary model paths directly
            logger.info(f"Using explicitly resolved model path: {model_type}")
            self.model_config = ModelConfig(
                path=Path(model_type),
                description=f"Custom loaded from {model_type}",
                output_processing="softmax_scale10" if "100cls" in model_type else "softmax",
                scale_factor=10.0 if "100cls" in model_type else 1.0,
                input_type="float32" # Unused with our robust parsing now
            )
            self.model_type = Path(model_type).stem
        elif model_type not in MODELS:
            # Check if model.tflite exists as default
            default_model_path = MODELS_DIR / "model.tflite"
            if default_model_path.exists():
                logger.info(f"Using default model: {default_model_path}")
                # Create a temporary model config for the default model
                self.model_config = ModelConfig(
                    path=default_model_path,
                    description="Default model",
                    output_processing="softmax_scale10",
                    scale_factor=10.0,
                    input_type="float32"
                )
                self.model_type = "model"
            else:
                raise ValueError(f"Unknown model type: {model_type}. Available: {list(MODELS.keys())}")
        else:
            self.model_config = MODELS[model_type]
            self.model_type = model_type
        
        logger.info(f"Loading {self.model_type} model from: {self.model_config.path.absolute()}")
        
        if not self.model_config.path.exists():
            raise FileNotFoundError(f"Model file not found: {self.model_config.path}")
            
        try:
            self.interpreter = tf.lite.Interpreter(model_path=str(self.model_config.path))
            self.interpreter.allocate_tensors()
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            
            # Store quantization parameters
            self.input_quantization = self.input_details[0].get('quantization', (1.0, 0))
            self.output_quantization = self.output_details[0].get('quantization', (1.0, 0))
            
            # Log quantization info
            self._log_quantization_info()
            
            # Validate model configuration matches actual model
            self._validate_model_config()
            
            # Debug logging
            self._debug_model_info()
            
        except Exception as e:
            raise RuntimeError(f"Failed to initialize model: {str(e)}")

    def _log_quantization_info(self) -> None:
        """Log detailed quantization information."""
        if self.model_config.quantized:
            input_scale, input_zero_point = self.input_quantization
            output_scale, output_zero_point = self.output_quantization
            
            logger.info(f"Quantization detected:")
            logger.info(f"  Input type: {self.model_config.input_type}")
            logger.info(f"  Input scale: {input_scale}, zero_point: {input_zero_point}")
            logger.info(f"  Output scale: {output_scale}, zero_point: {output_zero_point}")
            
            # Detect actual quantization scheme based on parameters
            if input_zero_point == -128 and abs(input_scale - 1/255.0) < 1e-6:
                logger.info("  Actual scheme: uint8 in int8 container [0,255] -> [-128,127]")
            elif input_zero_point == 0 and self.model_config.esp_dl_quantization:
                logger.info("  Actual scheme: True ESP-DL [-128,127]")
            else:
                logger.info("  Actual scheme: Standard quantization")
            
            # Additional output quantization info
            logger.info(f"  Output quantization - scale: {output_scale}, zero_point: {output_zero_point}")
            if output_zero_point != 0:
                logger.info(f"  Output requires dequantization: (output - {output_zero_point}) * {output_scale}")

    def _validate_model_config(self) -> None:
        """Validate that the model configuration matches the actual model."""
        input_shape = self.input_details[0]['shape']
        if len(input_shape) == 4:
            _, height, width, channels = input_shape
        elif len(input_shape) == 3:
            _, height, width = input_shape
            channels = 1
        else:
            raise ValueError(f"Unsupported input shape: {input_shape}")

        logger.info(f"Model actual input: {width}x{height}, channels: {channels}")

        if self.model_config.input_size:
            config_height, config_width = self.model_config.input_size
            if (height, width) != (config_height, config_width):
                logger.warning(f"Model config input_size {self.model_config.input_size} (HxW) doesn't match actual model input size {(height, width)} (HxW)")

        if channels != self.model_config.input_channels:
            logger.warning(f"Model config input_channels {self.model_config.input_channels} doesn't match actual model channels {channels}")
            
    def debug_output(self, output: np.ndarray) -> None:
        """Debug method to analyze output tensor."""
        logger.debug("=== OUTPUT DEBUG ===")
        logger.debug(f"Raw output shape: {output.shape}")
        logger.debug(f"Raw output dtype: {output.dtype}")
        logger.debug(f"Raw output range: [{output.min()}, {output.max()}]")
        logger.debug(f"Raw output values: {output[0]}")
        
        # Check if output is quantized
        if self.model_config.quantized:
            output_scale, output_zero_point = self.output_quantization
            logger.debug(f"Output quantization: scale={output_scale}, zero_point={output_zero_point}")
            
            # Dequantize manually
            dequantized = (output.astype(np.float32) - output_zero_point) * output_scale
            logger.debug(f"Dequantized range: [{dequantized.min():.6f}, {dequantized.max():.6f}]")
            logger.debug(f"Dequantized values: {dequantized[0]}")
            
            # Apply softmax to dequantized
            probs_dequant = tf.nn.softmax(dequantized[0]).numpy()
            logger.debug(f"Probabilities from dequant: {[f'{p:.4f}' for p in probs_dequant]}")
            
            # Apply softmax directly to raw output
            probs_raw = tf.nn.softmax(output[0].astype(np.float32)).numpy()
            logger.debug(f"Probabilities from raw: {[f'{p:.4f}' for p in probs_raw]}")

    def preprocess_image(self, image: np.ndarray) -> np.ndarray:
        """Preprocess image according to model input requirements."""
        if image is None or image.size == 0:
            raise ValueError("Invalid or empty image provided")

        input_shape = self.input_details[0]['shape']
        input_dtype = self.input_details[0]['dtype']

        # Handle different input shapes
        if len(input_shape) == 4:
            _, height, width, channels = input_shape
        elif len(input_shape) == 3:
            _, height, width = input_shape
            channels = 1
        else:
            raise ValueError(f"Unsupported input shape: {input_shape}")

        # Use model config dimensions if specified, otherwise use model's actual dimensions
        if self.model_config.input_size:
            target_height, target_width = self.model_config.input_size
        else:
            target_height, target_width = height, width

        logger.debug(f"Target size: {target_width}x{target_height}, Model expects: {width}x{height}")

        # Resize to expected size - OpenCV uses (width, height)
        image = cv2.resize(image, (target_width, target_height))

        # Handle channel conversion based on model requirements
        if channels == 1 or self.model_config.input_channels == 1:
            # Model expects grayscale
            if len(image.shape) == 3:
                image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # Ensure single channel for 4D input
            if len(image.shape) == 2 and len(input_shape) == 4:
                image = np.expand_dims(image, axis=-1)  # Add channel dimension (H, W, 1)
        elif channels == 3 or self.model_config.input_channels == 3:
            # Model expects color (RGB)
            if len(image.shape) == 2:
                # Convert grayscale to RGB by repeating channels
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            elif len(image.shape) == 3 and image.shape[2] == 1:
                # Convert single channel to 3 channels
                image = np.repeat(image, 3, axis=2)
            # Ensure BGR to RGB conversion
            if image.shape[2] == 3:
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Apply preprocessing operations
        if self.model_config.invert:
            image = cv2.bitwise_not(image)
        
        # Handle quantization and normalization - ROBUST DYNAMIC SCALING
        # Always normalize to [0, 1] first as a consistent starting point
        image = image.astype(np.float32) / 255.0
        
        if input_dtype == np.uint8:
            image = (image * 255.0).astype(np.uint8)
        elif input_dtype == np.int8:
            # Check if it's ESP-DL ([-128, 127]) or standard TFLite mapping
            input_scale, input_zero_point = self.input_quantization
            if input_zero_point == -128 and abs(input_scale - 1/255.0) < 1e-6:
                # uint8 encoded within int8
                image = (image * 255.0).astype(np.uint8)
                image = (image.astype(np.float32) / input_scale + input_zero_point).astype(np.int8)
            else:
                image = (image * 255.0 - 128.0).astype(np.int8)
        else:
            # Keep as float32 [0.0, 1.0]
            pass
        
        # Add batch dimension
        if len(input_shape) == 4:
            image = np.expand_dims(image, axis=0)  # Add batch dimension (1, H, W, C)
        elif len(input_shape) == 3:
            image = np.expand_dims(image, axis=0)  # Add batch dimension (1, H, W)

        logger.debug(f"Final preprocessed image shape: {image.shape}, dtype: {image.dtype}")
        logger.debug(f"Image value range: [{np.min(image)}, {np.max(image)}]")
        return image

    def _validate_input_image(self, image: np.ndarray) -> None:
        """Validate input image dimensions and type."""
        if image is None or image.size == 0:
            raise ValueError("Invalid or empty image provided")
        
        # Check if image dimensions match expected input size
        if self.model_config.input_size:
            expected_height, expected_width = self.model_config.input_size
            if image.shape[:2] != (expected_height, expected_width):
                logger.warning(f"Image size {image.shape[:2]} (HxW) doesn't match expected size {(expected_height, expected_width)} (HxW)")
                
    def _debug_model_info(self) -> None:
        """Log detailed model information for debugging."""
        logger.debug("=== Model Input Details ===")
        logger.debug(f"Input shape: {self.input_details[0]['shape']}")
        logger.debug(f"Input dtype: {self.input_details[0]['dtype']}")
        logger.debug(f"Input name: {self.input_details[0]['name']}")
        if 'quantization' in self.input_details[0]:
            logger.debug(f"Input quantization: {self.input_details[0]['quantization']}")
        
        logger.debug("=== Model Output Details ===")
        logger.debug(f"Output shape: {self.output_details[0]['shape']}")
        logger.debug(f"Output dtype: {self.output_details[0]['dtype']}")
        logger.debug(f"Output name: {self.output_details[0]['name']}")
        if 'quantization' in self.output_details[0]:
            logger.debug(f"Output quantization: {self.output_details[0]['quantization']}")
        
        logger.debug("=== Model Configuration ===")
        logger.debug(f"Model type: {self.model_type}")
        logger.debug(f"Input size: {self.model_config.input_size}")
        logger.debug(f"Input channels: {self.model_config.input_channels}")
        logger.debug(f"Input type: {self.model_config.input_type}")
        logger.debug(f"Quantized: {self.model_config.quantized}")
        logger.debug(f"ESP-DL Quantization: {self.model_config.esp_dl_quantization}")

    def _dequantize_output(self, output: np.ndarray) -> np.ndarray:
        """Dequantize output if the model is quantized."""
        if self.model_config.quantized and self.output_details[0]['dtype'] in [np.uint8, np.int8]:
            output_scale, output_zero_point = self.output_quantization
            # Convert to float32 first, then dequantize
            output_float = output.astype(np.float32)
            return (output_float - output_zero_point) * output_scale
        return output.astype(np.float32)

    def predict(self, image: np.ndarray) -> Tuple[float, float]:
        """Predict meter reading from an image - FIXED OUTPUT PROCESSING ONLY"""
        try:
            # Use the ORIGINAL working preprocessing
            input_image = self.preprocess_image(image)
            
            # Verify shape matches expected input shape
            expected_shape = self.input_details[0]['shape']
            if input_image.shape != tuple(expected_shape):
                logger.warning(f"Input shape {input_image.shape} doesn't match expected {tuple(expected_shape)}")
                if input_image.size == np.prod(expected_shape):
                    input_image = input_image.reshape(expected_shape)
                    logger.info(f"Reshaped input to: {input_image.shape}")
            
            # Set input tensor and run inference
            self.interpreter.set_tensor(self.input_details[0]['index'], input_image)
            self.interpreter.invoke()
            
            # Get model output
            output = self.interpreter.get_tensor(self.output_details[0]['index'])
            
            # DEBUG: Check what we're getting
            logger.debug(f"Raw output: {output[0]}")
            logger.debug(f"Raw output dtype: {output.dtype}")
            
            # Determine if output is quantized based on tensor details, not just config
            output_dtype = self.output_details[0]['dtype']
            
            if output_dtype in [np.uint8, np.int8]:
                # Dequantize output
                output_scale, output_zero_point = self.output_quantization
                output_values = (output[0].astype(np.float32) - output_zero_point) * output_scale
            else:
                # Float model, already float32 probabilities
                output_values = output[0]
                
            predicted_class = int(np.argmax(output_values))
            raw_confidence = float(np.max(output_values))

            # Clamp confidence
            confidence = max(0.0, min(1.0, raw_confidence))

            logger.debug(f"Predicted class: {predicted_class}, raw_confidence: {raw_confidence:.4f}, normalized: {confidence:.4f}")
            
            # Apply scaling based on model type
            if self.model_config.output_processing == "direct_class":
                meter_reading = float(predicted_class)
            elif self.model_config.output_processing == "softmax":
                meter_reading = float(predicted_class)
            else:  # "softmax_scale10"
                meter_reading = float(predicted_class) / self.model_config.scale_factor
            
            return meter_reading, confidence
            
        except Exception as e:
            logger.error(f"Prediction error: {str(e)}", exc_info=True)
            raise ValueError(f"Error during prediction: {str(e)}")

def load_regions(regions_source: Union[str, Path]) -> List[Tuple[int, int, int, int]]:
    """Load regions from file or string representation."""
    try:
        regions_path = Path(regions_source)
        
        if regions_path.exists():
            with regions_path.open('r') as f:
                regions = json.load(f)
        else:
            # Try to parse as string representation
            regions_str = str(regions_source).strip().strip('"').strip("'")
            regions = ast.literal_eval(regions_str)
        
        # Validate regions format and convert to tuples
        valid_regions = []
        for region in regions:
            if len(region) == 4:
                try:
                    valid_regions.append(tuple(map(int, region)))
                except (ValueError, TypeError):
                    logger.warning(f"Invalid region format: {region}")
            else:
                logger.warning(f"Region must have 4 coordinates, got: {region}")
        
        if not valid_regions:
            logger.error("No valid regions found in input")
        
        return valid_regions
        
    except (FileNotFoundError, json.JSONDecodeError, SyntaxError, ValueError) as e:
        logger.error(f"Error loading regions: {e}", exc_info=True)
        return []

def load_image(image_source: Union[str, Path], input_channels: int = 1) -> Optional[np.ndarray]:
    """Load image from local file or remote URL based on model's input requirements."""
    try:
        image_source_str = str(image_source)
        
        # Determine loading mode based on input_channels
        if input_channels == 1:
            # Load as grayscale
            load_mode = cv2.IMREAD_GRAYSCALE
        else:
            # Load as color (BGR)
            load_mode = cv2.IMREAD_COLOR
        
        if image_source_str.startswith(('http://', 'https://')):
            # Load from URL
            response = requests.get(image_source_str, timeout=10)
            response.raise_for_status()
            
            # Check content type
            content_type = response.headers.get('content-type', '')
            if 'image' not in content_type:
                logger.error(f"URL doesn't point to an image (Content-Type: {content_type})")
                return None
                
            image_array = np.asarray(bytearray(response.content), dtype=np.uint8)
            image = cv2.imdecode(image_array, load_mode)
        else:
            # Load from file
            image_path = Path(image_source_str)
            if not image_path.exists():
                logger.error(f"Image file not found: {image_path}")
                return None
                
            image = cv2.imread(str(image_path), load_mode)
        
        if image is None:
            logger.error(f"Unable to decode image from: {image_source_str}")
            return None
            
        # Convert BGR to RGB for color images if needed
        if input_channels == 3 and len(image.shape) == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
        # Check image size
        if image.nbytes > 50 * 1024 * 1024:  # 50MB
            logger.warning("Large image detected, consider resizing")
            
        # logger.info(f"Loaded image: {image_source_str}, shape: {image.shape}, channels: {1 if len(image.shape) == 2 else image.shape[2]}")
        return image
        
    except requests.exceptions.RequestException as e:
        logger.error(f"Error downloading image: {e}", exc_info=True)
        return None
    except Exception as e:
        logger.error(f"Unexpected error loading image: {e}", exc_info=True)
        return None

def validate_arguments(args: argparse.Namespace) -> bool:
    """Validate command line arguments."""
    # NEW: Allow default model.tflite if no specific model is specified
    if args.model not in MODELS and not Path(args.model).is_file():
        default_model_path = MODELS_DIR / "model.tflite"
        if not default_model_path.exists():
            logger.error(f"Invalid model type: {args.model}. Available models: {list(MODELS.keys())}")
            return False
    
    if not args.image_source and not args.folder:
        logger.error("Either --image_source or --folder must be specified")
        return False
    
    # UPDATED: Don't require regions file to exist - will process full image if not found
    return True

def print_help() -> None:
    """Print help information for the script."""
    help_text = f"""
Meter Reading Detection Script
{'=' * 50}

Usage: python meter_reading.py [options]

Options:
  --help                     Show this help message and exit
  --model MODEL_TYPE        Model type to use (default: {DEFAULT_MODEL} or model.tflite if exists)
                             Available models:"""
    
    for model_name, config in MODELS.items():
        help_text += f"\n                               {model_name}: {config.description}"
    
    help_text += f"""
  --regions REGIONS_SOURCE  Path to JSON file or string representation of regions (default: {DEFAULT_REGIONS_FILE})
                             If not specified or file doesn't exist, the entire image will be processed
  --image_source IMAGE_SOURCE Path to local image file or URL of remote image
  --folder FOLDER_PATH      Path to folder containing images to process
  --no-gui                  Disable GUI (no image display) - always disabled for folder processing
  --no-output-image         Do not save the output image with annotations
  --no-confidence           Do not display confidence scores on output image
  --test-all-models         Test all models with the same image and regions
  --expected_result EXPECTED_RESULT The real number read by human (for comparison with model results)

Examples:
  # Process single image
  python meter_reading.py --image_source sample.jpg
  
  # Process folder of images
  python meter_reading.py --folder ./images
  
  # Process folder with specific model
  python meter_reading.py --folder ./images --model class100-0180
  
  # Process single image with regions
  python meter_reading.py --model {DEFAULT_MODEL} --regions custom_regions.json --image_source sample.jpg

Note: If --regions is not specified, the script will look for {DEFAULT_REGIONS_FILE} in the current directory.
      If no regions file is found, the entire image will be processed as a single region.
      If --model is not specified, model.tflite in the models directory will be used if available.
      When processing a folder, GUI is automatically disabled and images are renamed with prediction_ prefix.
      Output files are named like: 12345_original_name.jpg where 12345 is the predicted reading.
"""
    print(help_text)

import shutil

def get_output_filename(prediction: float, confidence: float, original_path: Union[str, Path]) -> Path:
    """Generate output filename for the training folder."""
    original_path = Path(original_path)
    training_dir = Path("training")
    training_dir.mkdir(exist_ok=True)
    
    # Maintain the subfolder structure (e.g. training/coldwater/...)
    rel_dir = original_path.parent.name
    if rel_dir and rel_dir != "extracted":
        target_dir = training_dir / rel_dir
    else:
        target_dir = training_dir
    target_dir.mkdir(exist_ok=True)
    
    name_parts = original_path.stem  # e.g. "5_0.996_1771866086_q7o"
    extension = original_path.suffix
    
    # [new_inference]_[new_confidence]_[old_name]_err_corr.[ext]
    # Ensure prediction retains its decimal structure implicitly (e.g. 5.0)
    pred_str = str(prediction)
    conf_str = f"{confidence:.3f}"
    new_filename = f"{pred_str}_{conf_str}_{name_parts}_err_corr{extension}"
        
    return target_dir / new_filename

def process_full_image_as_region(image: np.ndarray) -> List[Tuple[int, int, int, int]]:
    """Process the entire image as a single region when no regions file is specified."""
    height, width = image.shape[:2]
    # Return the entire image as a single region
    return [(0, 0, width, height)]

def get_supported_image_extensions() -> List[str]:
    """Get list of supported image extensions."""
    return ['.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.tif', '.webp']

def find_images_in_folder(folder_path: Union[str, Path]) -> List[Path]:
    """Find all supported images in a folder."""
    folder = Path(folder_path)
    if not folder.exists() or not folder.is_dir():
        logger.error(f"Folder not found: {folder_path}")
        return []
    
    image_extensions = get_supported_image_extensions()
    images = []
    
    for ext in image_extensions:
        images.extend(folder.glob(f"*{ext}"))
        images.extend(folder.glob(f"*{ext.upper()}"))
    
    # Sort for consistent processing order
    images.sort()
    logger.info(f"Found {len(images)} images in folder: {folder_path}")
    return images

def parse_filename(filepath):
    """Parses [val]_[conf]_[timestamp]_[uuid].jpg"""
    filename = Path(filepath).name
    parts = filename.split('_')
    if len(parts) >= 4:
        try:
            return float(parts[0]), float(parts[1])
        except ValueError:
            pass
    return None, None

def process_single_image(
    meter_reader: MeterReader,
    image_path: Union[str, Path],
    regions: List[Tuple[int, int, int, int]],
    no_confidence: bool = True,
    save_output: bool = True,
    confidence_threshold: float = 0.9
) -> Dict[str, Any]:
    """
    Process a single image and return results.
    """
    try:
        input_channels = meter_reader.model_config.input_channels
        image = load_image(image_path, input_channels)
        if image is None:
            return {"error": f"Failed to load image: {image_path}"}

        if not regions:
            regions = process_full_image_as_region(image)

        result = process_image(
            meter_reader, 
            image, 
            regions, 
            no_confidence=no_confidence,
            image_source=image_path
        )

        final_reading = result["final_reading"]
        confidence = result["confidence_scores"][0] if result["confidence_scores"] else 0.0

        if final_reading != -1.0:
            orig_val, orig_conf = parse_filename(image_path)
            
            # If the new inference represents a different integer than the original prediction
            if orig_val is not None:
                def get_cpp_integer(val: float) -> int:
                    import math
                    decimal_part = val - math.floor(val)
                    if decimal_part >= 0.7:
                        digit = math.ceil(val)
                    else:
                        digit = math.floor(val)
                    
                    if meter_reader.model_type != "mnist" and digit >= 10:
                        digit = 0
                    return digit
                
                if get_cpp_integer(final_reading) != get_cpp_integer(orig_val):
                    result["output_filename"] = get_output_filename(final_reading, confidence, image_path)
                    if save_output:
                        # Copy the original image
                        shutil.copy2(image_path, result["output_filename"])
                        # logger.info(f"Prediction mismatch (orig: {orig_val}, new: {final_reading}), copied to training: {result['output_filename']}")
                else:
                    result["output_filename"] = None # Don't save if it matches the same integer
                    # logger.info(f"Match (val: {final_reading}, conf: {confidence:.3f}). No action needed.")

        return result

    except Exception as e:
        logger.error(f"Error processing {image_path}: {str(e)}")
        return {"error": str(e)}

def process_image(
    meter_reader: MeterReader,
    image: np.ndarray,
    regions: List[Tuple[int, int, int, int]],
    no_confidence: bool = False,
    image_source: Optional[Union[str, Path]] = None
) -> Dict[str, Any]:
    """
    Process an image with the given regions and return results.
    """
    results = {
        "raw_readings": [],
        "processed_readings": [],
        "confidence_scores": [],
        "final_reading": -1.0,
        "result_image": None
    }
    
    valid_regions = []
    
    for region in regions:
        x1, y1, x2, y2 = region
        try:
            if (x1 >= x2 or y1 >= y2 or x1 < 0 or y1 < 0 or 
                x2 > image.shape[1] or y2 > image.shape[0]):
                continue

            region_image = image[y1:y2, x1:x2]
            if region_image.size == 0:
                continue

            raw_reading, confidence = meter_reader.predict(region_image)
            results["raw_readings"].append(raw_reading)
            results["confidence_scores"].append(confidence)

            processed_reading = round(raw_reading)
            if meter_reader.model_type != "mnist" and processed_reading == 10:
                processed_reading = 0
            results["processed_readings"].append(processed_reading)
            valid_regions.append(region)
            
        except Exception as e:
            logger.error(f"Error processing region {region}: {str(e)}", exc_info=True)
            continue

    if not results["raw_readings"]:
        raise ValueError("No valid readings were processed")

    try:
        if len(results["raw_readings"]) == 1:
            raw_val = results["raw_readings"][0]
            # Match C++ wraparound behavior for 9.5-9.9
            if meter_reader.model_type != "mnist" and raw_val >= 9.5:
                raw_val -= 10.0
                if raw_val < 0:
                    raw_val = 0.0
            results["final_reading"] = round(float(raw_val), 1)
        else:
            final_str = ""
            for raw_val in results["raw_readings"]:
                digit = int(round(raw_val))
                if meter_reader.model_type != "mnist" and digit == 10:
                    digit = 0
                final_str += str(digit)
            results["final_reading"] = float(final_str)
    except Exception as e:
        logger.error(f"Error calculating final reading: {str(e)}")
        results["final_reading"] = -1.0

    return results

def process_folder(
    folder_path: Union[str, Path],
    meter_reader: MeterReader,
    regions: List[Tuple[int, int, int, int]],
    save_output: bool = True
) -> List[Dict[str, Any]]:
    """
    Process all images in a folder recursively.
    """
    folder = Path(folder_path)
    if not folder.exists() or not folder.is_dir():
        logger.error(f"Folder not found: {folder_path}")
        return []
    
    image_extensions = get_supported_image_extensions()
    images = []
    
    # Recursively find images
    for root, dirs, files in os.walk(folder):
        for file in files:
            if any(file.lower().endswith(ext) for ext in image_extensions):
                images.append(Path(root) / file)
                
    if not images:
        logger.error(f"No images found recursively in folder: {folder_path}")
        return []
        
    results = []
    successful = 0
    failed = 0
    
    logger.info(f"Starting batch processing of {len(images)} images...")
    
    for image_path in tqdm(images, desc="Processing images", unit="img"):
        try:
            result = process_single_image(
                meter_reader=meter_reader,
                image_path=image_path,
                regions=regions,
                no_confidence=True,
                save_output=save_output
            )
            
            if "error" not in result:
                successful += 1
                # logger.info(f"SUCCESS: {image_path.name} -> {result['final_reading']}")
            else:
                failed += 1
                # logger.error(f"FAILED: {image_path.name} -> Error: {result['error']}")
            
            result["image_path"] = str(image_path)
            results.append(result)
            
        except Exception as e:
            failed += 1
            # logger.error(f"FAILED: {image_path.name} -> Unexpected error: {str(e)}")
            results.append({
                "image_path": str(image_path),
                "error": str(e)
            })
    
    logger.info(f"Batch processing completed: {successful} successful, {failed} failed")
    logger.info(f"Folder processing complete: {successful}/{len(results)} successful")
    return results

def main() -> int:
    """Main function to run the meter reading detection."""
    parser = argparse.ArgumentParser(description="Meter Reading Detection", add_help=False)
    parser.add_argument("--help", action="store_true", help="Show help message and exit")
    parser.add_argument("--model", type=str, default=DEFAULT_MODEL, help="Model type to use")
    parser.add_argument("--regions", type=str, default=DEFAULT_REGIONS_FILE, help="Path to regions file or string")
    parser.add_argument("--image_source", type=str, help="Path to image file or URL")
    parser.add_argument("--folder", type=str, default="extracted", help="Path to folder containing images to process (default: extracted)")
    parser.add_argument("--no-gui", action="store_true", help="Disable GUI")
    parser.add_argument("--no-output-image", action="store_true", help="Do not save output image")
    parser.add_argument("--no-confidence", action="store_true", help="Do not display confidence scores")
    parser.add_argument("--test-all-models", action="store_true", help="Test all models")
    parser.add_argument("--expected_result", type=int, help="The real number read by human")
    
    args = parser.parse_args()
    
    if args.help:
        print_help()
        return 0
    
    if not validate_arguments(args):
        return 1
    
    try:
        # Initialize meter reader
        meter_reader = MeterReader(args.model)
        
        # Load regions
        regions = load_regions(args.regions)
        if not regions:
            logger.info(f"No valid regions found in {args.regions}, will process entire images as single regions")

        # Handle single image processing (original functionality)
        if args.image_source:
            # Load image
            image = load_image(args.image_source, input_channels=1)
            if image is None:
                logger.error("Failed to load image")
                return 1

            # Process regions
            if not regions:
                logger.info("Processing entire image as single region")
                regions = process_full_image_as_region(image)

            # Process image
            result = process_single_image(
                meter_reader=meter_reader,
                image_path=args.image_source,
                regions=regions,
                no_confidence=args.no_confidence,
                save_output=not args.no_output_image
            )

            # Display results
            logger.info("=" * 50)
            logger.info("METER READING RESULTS")
            logger.info("=" * 50)
            logger.info(f"Model used: {args.model}")
            logger.info(f"Number of regions processed: {len(regions)}")

            for i, (raw, processed, confidence) in enumerate(zip(
                result.get("raw_readings", []), 
                result.get("processed_readings", []), 
                result.get("confidence_scores", [])
            )):
                logger.info(f"Digit {i+1}: Raw={raw:.2f}, Processed={processed}, Confidence={confidence:.2%}")

            logger.info(f"Final Reading: {result.get('final_reading', -1):.2f}")
            logger.info("=" * 50)

            # Display image (only if not in no-gui mode and not processing folder)
            if not args.no_gui and result.get("result_image") is not None:
                cv2.imshow("Meter Reading Result", result["result_image"])
                cv2.waitKey(0)
                cv2.destroyAllWindows()

            return 0
            
        # Handle folder processing
        if args.folder:
            logger.info(f"Processing folder: {args.folder}")
            results = process_folder(
                folder_path=args.folder,
                meter_reader=meter_reader,
                regions=regions,
                save_output=not args.no_output_image
            )
            return 0

    except KeyboardInterrupt:
        logger.info("Execution interrupted by user")
        return 130
    except Exception as e:
        logger.error(f"Unexpected error: {str(e)}", exc_info=True)
        return 1

if __name__ == "__main__":
    sys.exit(main())
    
    
# # Process all images in a folder with default settings
# python correct_errors.py --folder all_errors --model class100-0180
