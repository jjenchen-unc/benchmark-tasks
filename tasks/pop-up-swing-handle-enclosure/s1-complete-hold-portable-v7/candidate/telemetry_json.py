"""Task-local lossless numeric telemetry; unknown and nonfinite values fail closed."""
import json
import numpy as np

def json_scalar(value):
    if isinstance(value, np.generic):
        converted = value.item()
        if isinstance(converted, (bool, int, float)):
            return converted
    if isinstance(value, np.ndarray) and value.dtype.kind in "biuf":
        return value.tolist()
    raise TypeError("Unsupported telemetry JSON type: " + type(value).__name__)

def dumps(value, **kwargs):
    return json.dumps(value, default=json_scalar, allow_nan=False, **kwargs)
