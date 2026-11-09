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

"""Three-way acceptance check for one captured frame.

The trained checkpoint (PyTorch), the exported graph (ONNX Runtime) and the
official RGB-D teacher are run on the same frame. A regression in
preprocessing, export or the C++ decoder then shows up as a pixel-count
mismatch instead of a silently empty frontier map.
"""

from __future__ import annotations

import os
from typing import Dict, Optional

import numpy as np

from frontiernet.postprocess import FrontierMap, prediction2frontiermap, region_iou

__all__ = ["compare_predictions", "verify_frame", "load_frame"]


def load_frame(path: str) -> Dict[str, np.ndarray]:
    """Read one ``frames/<idx>.npz`` produced by :mod:`frontiernet.collect`."""
    with np.load(path) as data:
        if "depth" not in data:
            raise ValueError(f"{path} has no depth array")
        depth = np.asarray(data["depth"], dtype=np.float32)
        rgb = np.asarray(data["rgb"]) if "rgb" in data else None
    if depth.ndim != 2:
        raise ValueError(f"expected a 2-D depth frame, got {depth.shape}")
    return {"depth": depth, "rgb": rgb}


def compare_predictions(
    student: FrontierMap,
    onnx: Optional[FrontierMap] = None,
    teacher: Optional[FrontierMap] = None,
    *,
    onnx_atol: float = 1e-3,
) -> Dict[str, object]:
    """Summarise how the three predictions of one frame agree."""
    report: Dict[str, object] = {
        "student_pixels": int(student.region.sum()),
        "student_df_max": float(student.df.max()),
    }
    ok = report["student_pixels"] > 0
    if onnx is not None:
        delta = float(np.abs(student.df - onnx.df).max())
        report.update(
            {
                "onnx_pixels": int(onnx.region.sum()),
                "onnx_df_max": float(onnx.df.max()),
                "onnx_df_max_abs_delta": delta,
                "onnx_pixels_match": report["student_pixels"] == int(onnx.region.sum()),
            }
        )
        ok = ok and report["onnx_pixels_match"] and delta <= onnx_atol
    if teacher is not None:
        report.update(
            {
                "teacher_pixels": int(teacher.region.sum()),
                "teacher_df_max": float(teacher.df.max()),
                "teacher_iou": region_iou(student.region, teacher.region),
            }
        )
    report["ok"] = bool(ok)
    return report


def _student_map(
    frame: Dict[str, np.ndarray],
    checkpoint: str,
    defm_weights: Optional[str],
    device: str,
    input_size: int,
    classes: int,
    df_normalizer: float,
    df_threshold: float,
) -> FrontierMap:
    import torch

    from frontiernet.checkpoint import read_state_dict, resolve_weights
    from frontiernet.model import FrontierNetDeFM
    from frontiernet.preprocess import defm_preprocess

    model = FrontierNetDeFM(
        classes=classes,
        defm_weights=resolve_weights(defm_weights) if defm_weights else None,
        input_size=input_size,
    )
    model.load_state_dict(read_state_dict(resolve_weights(checkpoint)), strict=True)
    model.to(device).eval()
    tensor = defm_preprocess(frame["depth"], input_size)
    with torch.no_grad():
        df, logits = model(torch.from_numpy(tensor).unsqueeze(0).to(device))
    return prediction2frontiermap(
        df.detach().cpu().numpy(),
        logits.detach().cpu().numpy(),
        n_classes=classes,
        df_normalizer=df_normalizer,
        threshold=df_threshold,
    )


def _onnx_map(
    frame: Dict[str, np.ndarray],
    onnx_path: str,
    input_size: int,
    classes: int,
    df_normalizer: float,
    df_threshold: float,
) -> FrontierMap:
    import onnxruntime as ort

    from frontiernet.preprocess import defm_preprocess

    session = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    name = session.get_inputs()[0].name
    tensor = defm_preprocess(frame["depth"], input_size)[None].astype(np.float32)
    df, logits = session.run(None, {name: tensor})
    return prediction2frontiermap(
        df,
        logits,
        n_classes=classes,
        df_normalizer=df_normalizer,
        threshold=df_threshold,
    )


def _teacher_map(
    frame: Dict[str, np.ndarray],
    weights: str,
    device: str,
    input_size: int,
    classes: int,
    df_normalizer: float,
    df_threshold: float,
) -> FrontierMap:
    import torch

    from frontiernet.checkpoint import read_state_dict, resolve_weights
    from frontiernet.preprocess import frontiernet_preprocess_aligned
    from frontiernet.unet import TwoHeadUnet

    if frame["rgb"] is None:
        raise ValueError("the teacher needs an rgb frame")
    model = TwoHeadUnet(classes=classes, in_channels=4)
    model.load_state_dict(read_state_dict(resolve_weights(weights)), strict=True)
    model.to(device).eval()
    tensor = frontiernet_preprocess_aligned(
        frame["rgb"], frame["depth"], size=input_size
    )
    with torch.no_grad():
        df, logits = model(torch.from_numpy(tensor).unsqueeze(0).to(device))
    return prediction2frontiermap(
        df.detach().cpu().numpy(),
        logits.detach().cpu().numpy(),
        n_classes=classes,
        df_normalizer=df_normalizer,
        threshold=df_threshold,
    )


def verify_frame(
    frame_path: str,
    checkpoint: str,
    *,
    defm_weights: Optional[str] = None,
    onnx_path: Optional[str] = None,
    teacher_weights: Optional[str] = None,
    device: str = "cpu",
    input_size: int = 224,
    classes: int = 11,
    df_normalizer: float = 10.0,
    df_threshold: float = 0.5,
    onnx_atol: float = 1e-3,
) -> Dict[str, object]:
    """Run the checkpoint, the ONNX graph and the teacher on one frame."""
    frame = load_frame(frame_path)
    student = _student_map(
        frame, checkpoint, defm_weights, device, input_size, classes,
        df_normalizer, df_threshold,
    )
    onnx_map = None
    if onnx_path:
        onnx_map = _onnx_map(
            frame, onnx_path, input_size, classes, df_normalizer, df_threshold
        )
    teacher_map = None
    if teacher_weights:
        teacher_map = _teacher_map(
            frame, teacher_weights, device, input_size, classes,
            df_normalizer, df_threshold,
        )
    report = compare_predictions(student, onnx_map, teacher_map, onnx_atol=onnx_atol)
    report["frame"] = os.path.basename(frame_path)
    report["checkpoint"] = checkpoint
    if onnx_path:
        report["onnx"] = os.path.basename(onnx_path)
    if teacher_weights:
        report["teacher"] = os.path.basename(teacher_weights)
    return report
