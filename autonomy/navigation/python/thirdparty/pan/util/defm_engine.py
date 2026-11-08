"""Frozen DeFM ViT-S/14 depth encoder: metric depth frames -> patch tokens.

DeFM's `preprocess_depth_batch` maps metric depth to its 3-channel log-depth input and resizes it
to the pretrain square DEFM_IMAGE_RESOLUTION.
"""
from __future__ import annotations

import os
from pathlib import Path

import numpy as np

os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")
os.environ.setdefault("XFORMERS_DISABLED", "1")     # DeFM's DINOv2 layers: pure-torch path

import torch

from thirdparty.pan.util.constants import DEFM_IMAGE_RESOLUTION, DEFM_PATCH_SIZE, REPO_ROOT
# ViT-S/14 kwargs of third_party/defm/defm/configs/defm_vit_s14.yaml.
_VIT_S_KWARGS = dict(
    img_size=DEFM_IMAGE_RESOLUTION, patch_size=DEFM_PATCH_SIZE, in_chans=3,
    init_values=1e-5, ffn_layer="swiglufused", block_chunks=4, qkv_bias=True,
    proj_bias=True, ffn_bias=True, num_register_tokens=0, interpolate_offset=0.1,
    interpolate_antialias=False,
)


class DeFMEncoder:
    """Frozen DeFM ViT-S/14 on `device`, run under `amp_dtype` autocast on CUDA."""

    def __init__(self, device: torch.device, amp_dtype: torch.dtype = torch.float32,
                 weights_path: str | Path | None = None):
        from thirdparty.defm.models.vision_transformer import vit_small

        self.device, self.amp_dtype = device, amp_dtype
        model = vit_small(**_VIT_S_KWARGS)
        path = Path(weights_path or os.environ.get(
            "VNT_PA_DEFM_WEIGHTS", REPO_ROOT.parent / "conf/models/defm_vit_s14.pth"))
        if not path.is_file():
            raise FileNotFoundError(f"DeFM checkpoint not found: {path}")
        state = torch.load(path, map_location="cpu", weights_only=True)
        msg = model.load_state_dict(state.get("model", state), strict=False)
        assert not msg.missing_keys and not msg.unexpected_keys, (
            f"DeFM ViT-S state_dict mismatch: missing={msg.missing_keys[:3]} "
            f"unexpected={msg.unexpected_keys[:3]}")
        self.model = model.to(device).eval().requires_grad_(False)

    def encode(self, depth: np.ndarray) -> torch.Tensor:
        """(M, H, W) or (H, W) metric depth in metres -> (M, 256, 384) fp16 patch tokens on the
        encoder's device."""
        from thirdparty.defm.utils.utils import preprocess_depth_batch

        d = torch.from_numpy(np.ascontiguousarray(depth)).float().to(self.device)
        if d.ndim == 2:
            d = d.unsqueeze(0)
        assert d.ndim == 3, f"depth must be (M, H, W) metric metres, got {tuple(d.shape)}"
        x = preprocess_depth_batch(d, target_size=DEFM_IMAGE_RESOLUTION,
                                   patch_size=DEFM_PATCH_SIZE, device=self.device)
        use_amp = self.device.type == "cuda" and self.amp_dtype != torch.float32
        with torch.inference_mode(), torch.autocast(
                self.device.type, dtype=self.amp_dtype, enabled=use_amp):
            out = self.model.get_intermediate_layers(x, n=1, reshape=False)
        return out[0].to(torch.float16)
