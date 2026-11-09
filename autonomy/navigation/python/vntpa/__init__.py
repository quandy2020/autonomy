# Copyright 2026 The Navigation Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""VNT-PA + DeFM training, ONNX deployment and autosim bridge."""

from vntpa.data import ContextMemory, Keyframe, preprocess
from vntpa.models import DefmOutput, OnnxDefmEncoder, OnnxPlanner, PlanResult
from vntpa.engine.model import VNT
from vntpa.engine.predictor import Predictor
from vntpa.engine.result import VNTResult
from vntpa.simulator.action import (
    Action,
    ActionPrimitive,
    Twist,
    primitive,
    twist,
    decode,
)
from vntpa.engine.metric import BenchmarkReport, benchmark
from vntpa.engine.trainer import (
    EndHook,
    StartHook,
    ValConfig,
    build_checkpoint_hook,
    build_jsonl_end_hook,
    build_jsonl_start_hook,
    train,
    validate,
)
from vntpa.utils.config import (
    MODELS,
    AutosimConfig,
    DeployConfig,
    ModelSpec,
    PlannerConfig,
    lookup,
    names,
    register,
    resolve,
)
from vntpa.utils.errors import (
    InvalidConfigError,
    ModelNotFoundError,
    ModelValidationError,
    UnsupportedFormatError,
    VNTError,
)
from vntpa.version import __version__

__all__ = [
    "Action",
    "ActionPrimitive",
    "AutosimConfig",
    "BenchmarkReport",
    "ContextMemory",
    "DefmOutput",
    "DeployConfig",
    "EndHook",
    "InvalidConfigError",
    "Keyframe",
    "MODELS",
    "ModelNotFoundError",
    "ModelSpec",
    "ModelValidationError",
    "OnnxDefmEncoder",
    "OnnxPlanner",
    "PlanResult",
    "PlannerConfig",
    "Predictor",
    "StartHook",
    "Twist",
    "UnsupportedFormatError",
    "VNT",
    "VNTError",
    "VNTResult",
    "ValConfig",
    "__version__",
    "benchmark",
    "build_checkpoint_hook",
    "build_jsonl_end_hook",
    "build_jsonl_start_hook",
    "decode",
    "lookup",
    "names",
    "preprocess",
    "primitive",
    "register",
    "resolve",
    "train",
    "twist",
    "validate",
]
