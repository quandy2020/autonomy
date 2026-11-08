"""VNT-PA + DeFM training, ONNX deployment and autosim bridge."""

from .data import ContextMemory, Keyframe, preprocess_depth
from .training.evaluation.metrics import Benchmark, BenchmarkReport
from .runtime.inference.predictor import Predictor, VNT, VNTResult
from .models import DefmOutput, OnnxDefmEncoder, OnnxPlanner, PlanResult
from .runtime.sim.action_adapter import (
    Action,
    ActionPrimitive,
    Twist,
    decode_logits,
    primitive,
    primitive_to_twist,
)
from .training.trainer import (
    Callback,
    CallbackList,
    CheckpointCallback,
    JsonlCallback,
    Trainer,
    ValConfig,
    Validator,
)
from .utils.config import (
    DEFAULT_REGISTRY,
    AutosimConfig,
    DeployConfig,
    ModelRegistry,
    ModelSpec,
    PlannerConfig,
)
from .version import __version__
from .utils.errors import (
    InvalidConfigError,
    ModelNotFoundError,
    ModelValidationError,
    UnsupportedFormatError,
    VNTError,
)

__all__ = [
    "__version__",
    "Action",
    "ActionPrimitive",
    "AutosimConfig",
    "Benchmark",
    "BenchmarkReport",
    "Callback",
    "CallbackList",
    "CheckpointCallback",
    "ContextMemory",
    "DEFAULT_REGISTRY",
    "DefmOutput",
    "DeployConfig",
    "InvalidConfigError",
    "Keyframe",
    "ModelNotFoundError",
    "ModelRegistry",
    "ModelSpec",
    "ModelValidationError",
    "OnnxDefmEncoder",
    "OnnxPlanner",
    "PlanResult",
    "PlannerConfig",
    "Predictor",
    "Trainer",
    "ValConfig",
    "Validator",
    "JsonlCallback",
    "Twist",
    "UnsupportedFormatError",
    "VNT",
    "VNTError",
    "VNTResult",
    "decode_logits",
    "preprocess_depth",
    "primitive",
    "primitive_to_twist",
]
