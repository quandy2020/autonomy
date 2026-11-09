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

"""FrontierNet-style frontier detection with a swappable depth encoder.

The numpy half (pre/post-processing) imports without torch so the C++ runtime
contract can be checked in a bare environment; the torch half is resolved
lazily on first attribute access.
"""

from __future__ import annotations

from typing import Any, Dict, Tuple

from frontiernet.postprocess import (
    FrontierMap,
    cls2gainmap,
    cls_bin_edges,
    denormalize_df,
    df2region,
    logits_to_labels,
    prediction2frontiermap,
    region_iou,
)
from frontiernet.preprocess import (
    DEFM_INPUT_SIZE,
    DEFM_MEAN,
    DEFM_STD,
    defm_depth_channels,
    defm_preprocess,
    frontiernet_preprocess,
    frontiernet_preprocess_aligned,
    resize,
    resize_center_crop,
)
from frontiernet.version import __version__

_LAZY: Dict[str, Tuple[str, str]] = {
    "DefmEncoder": ("frontiernet.encoder_defm", "DefmEncoder"),
    "DefmViTEncoder": ("frontiernet.encoder_defm", "DefmViTEncoder"),
    "ViTToUnetAdapter": ("frontiernet.encoder_defm", "ViTToUnetAdapter"),
    "FrontierNetDeFM": ("frontiernet.model", "FrontierNetDeFM"),
    "TwoHeadUnet": ("frontiernet.unet", "TwoHeadUnet"),
    "UnetDecoder": ("frontiernet.unet", "UnetDecoder"),
    "FrontierPredictor": ("frontiernet.infer", "FrontierPredictor"),
    "LoadReport": ("frontiernet.checkpoint", "LoadReport"),
    "load_frontiernet_heads": ("frontiernet.checkpoint", "load_frontiernet_heads"),
    "load_heads": ("frontiernet.checkpoint", "load_heads"),
    "read_state_dict": ("frontiernet.checkpoint", "read_state_dict"),
    "save_checkpoint": ("frontiernet.checkpoint", "save_checkpoint"),
    "FrontierLoss": ("frontiernet.loss", "FrontierLoss"),
    "LossWeights": ("frontiernet.loss", "LossWeights"),
    "class_weights_from_histogram": ("frontiernet.loss", "class_weights_from_histogram"),
    "FrontierFrameDataset": ("frontiernet.dataset", "FrontierFrameDataset"),
    "TeacherLabeler": ("frontiernet.label", "TeacherLabeler"),
    "label_frames": ("frontiernet.label", "label_frames"),
    "TrainSettings": ("frontiernet.train", "TrainSettings"),
    "compare_predictions": ("frontiernet.verify", "compare_predictions"),
    "load_frame": ("frontiernet.verify", "load_frame"),
    "verify_frame": ("frontiernet.verify", "verify_frame"),
}

__all__ = [
    "DEFM_INPUT_SIZE",
    "DEFM_MEAN",
    "DEFM_STD",
    "FrontierMap",
    "__version__",
    "cls2gainmap",
    "cls_bin_edges",
    "defm_depth_channels",
    "defm_preprocess",
    "denormalize_df",
    "df2region",
    "frontiernet_preprocess",
    "frontiernet_preprocess_aligned",
    "logits_to_labels",
    "prediction2frontiermap",
    "region_iou",
    "resize",
    "resize_center_crop",
    *_LAZY,
]


def __getattr__(name: str) -> Any:
    target = _LAZY.get(name)
    if target is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    import importlib

    module = importlib.import_module(target[0])
    value = getattr(module, target[1])
    globals()[name] = value
    return value


def __dir__() -> list:
    return sorted(set(list(globals()) + list(_LAZY)))
