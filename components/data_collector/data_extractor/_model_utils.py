"""
Shared model configuration for data_extractor scripts.

Provides ModelConfig dataclass, parse_model_txt_file(), discover_models(),
and constants used by 5_low_confidence.py, 3_correct_errors.py, etc.

Model config is auto-discovered from .txt files in the models/ directory,
using the same parsing logic as meter_reader_tflite/__init__.py.
"""

import logging
import re
from pathlib import Path
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

# Configuration Constants
# Look for models in multiple locations (project root and local data_extractor)
_PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent.parent  # esphome_ai_component/
MODELS_DIRS = [
    _PROJECT_ROOT / "models",                                      # main models/
    Path(__file__).resolve().parent / "models",                    # data_extractor/models/
]
# discover_models() checks existence before scanning, so no static exists() filter here
MODELS_DIR = MODELS_DIRS[0] if MODELS_DIRS[0].exists() else MODELS_DIRS[1]
DEFAULT_REGIONS_FILE = Path("regions.json")
DEFAULT_MODEL = "digit_recognizer_v23_10cls_RGB"
DEFAULT_RESULT_IMAGE = Path("result.jpg")
MAX_IMAGE_SIZE = (1920, 1080)  # For memory safety


@dataclass
class ModelConfig:
    path: Path
    description: str
    output_processing: str
    scale_factor: float
    input_type: str  # 'float32', 'uint8', 'int8'
    input_channels: int = 3
    input_size: Optional[Tuple[int, int]] = None
    normalize: bool = False
    invert: bool = False
    input_order: str = 'RGB'

    @property
    def quantized(self) -> bool:
        return self.input_type in ['uint8', 'int8']

    @property
    def esp_dl_quantization(self) -> bool:
        return self.input_type == 'int8'


def parse_model_txt_file(txt_path: Path) -> Optional[ModelConfig]:
    """Parse a model .txt file to extract configuration parameters.
    Uses the same approach as meter_reader_tflite/__init__.py parse_model_txt_file().
    """
    if not txt_path.exists() or not txt_path.is_file():
        return None

    with open(txt_path, 'r') as f:
        content = f.read()

    config = {}

    # Input shape and dtype
    input_match = re.search(r'Input\s+0:\s+\[\s*\d+\s+(\d+)\s+(\d+)\s+(\d+)\].*?numpy\.(\w+)', content)
    if input_match:
        config['input_height'] = int(input_match.group(1))
        config['input_width'] = int(input_match.group(2))
        config['input_channels'] = int(input_match.group(3))
        dtype = input_match.group(4)
        if dtype == 'float32':
            config['input_type'] = 'float32'
        elif dtype == 'int8':
            config['input_type'] = 'int8'
        else:
            config['input_type'] = 'uint8'

    # Num classes + scale_factor
    output_match = re.search(r'Output\s+0:\s+\[\s*\d+\s+(\d+)\]', content)
    num_classes = 10
    if output_match:
        num_classes = int(output_match.group(1))

    if num_classes == 10:
        config['scale_factor'] = 1.0
    elif num_classes == 100:
        config['scale_factor'] = 10.0
    else:
        config['scale_factor'] = 1.0

    # Output processing
    has_softmax = bool(re.search(r'^\s+SOFTMAX:\s+\d+', content, re.MULTILINE))
    config['output_processing'] = 'direct_class' if has_softmax else 'softmax'
    logger.info(f"  Output processing: {config['output_processing']} (SOFTMAX={'yes' if has_softmax else 'no'})")

    # Input size tuple
    if 'input_height' in config and 'input_width' in config:
        config['input_size'] = (config['input_height'], config['input_width'])

    # Input order from filename heuristics
    name = txt_path.stem
    if '_GRAY' in name or '_GRAYSCALE' in name:
        config['input_order'] = 'GRAY'
    elif '_BGR' in name:
        config['input_order'] = 'BGR'
    else:
        config['input_order'] = 'RGB'

    # Quantization family
    if re.search(r'shape=\[1 1 1 3\]', content):
        config['quantization_family'] = 'qat'
        logger.info(f"  Quantization: QAT (learned 1x1 conv)")
    else:
        config['quantization_family'] = 'tqt'
        logger.info(f"  Quantization: TQT")

    # Hybrid quantization warning
    input_dtype = config.get('input_type', '')
    has_float32_io = (input_dtype == 'float32')
    has_int8_weights = bool(re.search(r"<class 'numpy\.(int8|uint8)'>", content))
    if has_float32_io and has_int8_weights:
        logger.warning(f"  ⚠ Model '{txt_path.name}' has hybrid quantization (not TFLite Micro compatible)")

    # DELEGATE ops warning
    if re.search(r'Found \d+ DELEGATE operation', content):
        logger.warning(f"  ⚠ Model '{txt_path.name}' contains DELEGATE ops (incompatible)")

    # Description
    config['description'] = name

    # Model file path
    model_path = txt_path.with_suffix('.tflite')
    if not model_path.exists():
        model_path = txt_path.parent / f"{name}.tflite"
        if not model_path.exists():
            logger.warning(f"Model file not found: {model_path}")
            return None

    return ModelConfig(
        path=model_path,
        description=config.get('description', name),
        output_processing=config.get('output_processing', 'direct_class'),
        scale_factor=config.get('scale_factor', 1.0),
        input_type=config.get('input_type', 'uint8'),
        input_channels=config.get('input_channels', 3),
        input_size=config.get('input_size'),
        input_order=config.get('input_order', 'RGB'),
    )


def discover_models(models_dirs: Optional[List[Path]] = None) -> Dict[str, ModelConfig]:
    """Discover all models by scanning for .txt files in models_dirs."""
    if models_dirs is None:
        models_dirs = MODELS_DIRS
    models: Dict[str, ModelConfig] = {}
    for models_dir in models_dirs:
        if not models_dir.exists():
            continue
        for txt_file in sorted(models_dir.glob("*.txt")):
            name = txt_file.stem
            config = parse_model_txt_file(txt_file)
            if config:
                models[name] = config
                logger.info(f"Discovered model '{name}': {config.input_channels}ch {config.input_size} {config.input_type}")
    if not models:
        logger.warning(f"No models discovered from any directory")
    return models


# Auto-discover models at module load time
MODELS: Dict[str, ModelConfig] = discover_models()