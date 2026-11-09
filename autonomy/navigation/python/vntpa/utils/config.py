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

"""Configuration objects shared by training, deployment and autosim."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Tuple

from vntpa.utils.errors import InvalidConfigError


@dataclass
class PlannerConfig:
    """Single VNT-PA planner ONNX contract."""

    context_size: int = 64
    token_count: int = 256
    token_dim: int = 384
    chunk: int = 8
    action_count: int = 4
    forward_step: float = 0.25
    turn_step_rad: float = 0.5235987756
    action_dt_s: float = 0.5
    execute_horizon: int = 4
    decision_rate_hz: float = 5.0
    providers: Tuple[str, ...] = ("CPUExecutionProvider",)

    def validate_config(self) -> None:
        checks = [
            (self.context_size > 0, "context_size must be positive"),
            (self.token_count > 0 and self.token_dim > 0, "token shape must be positive"),
            (self.chunk > 0, "chunk must be positive"),
            (self.action_count == 4, "action_count must be 4"),
            (self.action_dt_s > 0, "action_dt_s must be positive"),
            (self.decision_rate_hz > 0, "decision_rate_hz must be positive"),
            (0 < self.execute_horizon <= self.chunk, "execute_horizon must be in (0, chunk]"),
        ]
        for ok, message in checks:
            if not ok:
                raise InvalidConfigError(message)


@dataclass
class DeployConfig:
    """Deployment paths and runtime options."""

    planner_onnx: str = "conf/models/vntpa.onnx"
    defm_onnx: str = "conf/models/defm_vit_s14.onnx"
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    providers: Tuple[str, ...] = ("CPUExecutionProvider",)


@dataclass
class AutosimConfig:
    """autosim/autolink topics and rates."""

    node_name: str = "vntpa"
    depth_topic: str = "/camera/depth/image_raw"
    odom_topic: str = "/odom"
    goal_topic: str = "/navigation/goal"
    cmd_topic: str = "/cmd_vel"
    timer_hz: float = 10.0
    map_frame: str = "map"
    base_frame: str = "base_link"
    goal_type: int = 0  # GoalSpec.TYPE_POSE
    min_keyframe_distance: float = 0.2
    min_keyframe_interval_s: float = 0.2
    max_keyframes: int = 64


@dataclass
class ProductConfig:
    """Deployment + autosim settings loaded from one YAML file."""

    deploy: DeployConfig = field(default_factory=DeployConfig)
    autosim: AutosimConfig = field(default_factory=AutosimConfig)


def configure(path: str | Path) -> ProductConfig:
    """Load YAML into the typed product configuration.

    Paths are kept as written; callers may resolve them against their own
    working directory. No network access or implicit download is performed.
    """
    import yaml

    raw: Dict[str, Any] = yaml.safe_load(Path(path).read_text()) or {}
    planner = PlannerConfig(**raw.get("planner", {}))
    providers = tuple(raw.get("providers", planner.providers))
    deploy = DeployConfig(
        planner_onnx=raw.get("planner_onnx", DeployConfig.planner_onnx),
        defm_onnx=raw.get("defm_onnx", DeployConfig.defm_onnx),
        planner=planner,
        providers=providers,
    )
    autosim = AutosimConfig(**raw.get("autosim", {}))
    return ProductConfig(deploy=deploy, autosim=autosim)


def locate() -> Path:
    """Locate ``configs/train/default.yaml`` from cwd or the source checkout."""
    for base in (Path.cwd(), Path(__file__).resolve().parents[2]):
        candidate = base / "configs/train/default.yaml"
        if candidate.is_file():
            return candidate
    return Path(__file__).resolve().parents[2] / "configs/train/default.yaml"


@dataclass(frozen=True)
class ModelSpec:
    """A local model pair; no remote resolution."""

    name: str
    planner_onnx: str
    defm_onnx: str
    task: str = "navigate"
    description: str = ""


MODELS: Dict[str, ModelSpec] = {}


def register(spec: ModelSpec) -> None:
    """Register one local model pair."""
    MODELS[spec.name] = spec


def lookup(name: str) -> ModelSpec | None:
    """Return a registered model or ``None``."""
    return MODELS.get(name)


def names() -> list[str]:
    """Return registered model names in sorted order."""
    return sorted(MODELS)


def resolve(name_or_path: str) -> ModelSpec:
    """Resolve a registry name or an ONNX path to a model pair."""
    spec = lookup(name_or_path)
    if spec is not None:
        return spec
    path = Path(name_or_path)
    if path.is_file():
        default = lookup("vntpa-s14")
        return ModelSpec(path.stem, str(path), default.defm_onnx if default else "")
    raise KeyError(f"unknown model: {name_or_path}")


def registry(path: str | Path) -> None:
    """Merge a YAML model list into :data:`MODELS`."""
    import yaml

    raw = yaml.safe_load(Path(path).read_text()) or {}
    for item in raw.get("models", []):
        register(
            ModelSpec(
                name=item["name"],
                planner_onnx=item["planner"],
                defm_onnx=item["defm"],
                task=item.get("task", "navigate"),
                description=item.get("description", ""),
            )
        )


register(
    ModelSpec(
        name="vntpa-s14",
        planner_onnx="conf/models/vntpa.onnx",
        defm_onnx="conf/models/defm_vit_s14.onnx",
        description="VNT-PA ViT-S/14 planner + DeFM depth encoder",
    )
)
