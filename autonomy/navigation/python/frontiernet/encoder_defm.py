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

"""DeFM ViT-S/14 as a drop-in replacement for FrontierNet's ResNet-34 encoder.

The official decoder consumes five feature maps::

    stride 2   64 ch   112x112
    stride 4   64 ch    56x56
    stride 8  128 ch    28x28
    stride 16 256 ch    14x14
    stride 32 512 ch     7x7

DeFM is a plain ViT: every block produces ``16x16`` tokens of width 384 for a
224x224 input (patch 14). A ViT has no pyramid, so this module adds one:

1. ``DefmViTEncoder`` taps five blocks (shallow -> deep) and keeps the token
   grid, giving five ``(B, 384, 16, 16)`` maps with increasing semantics.
2. ``ViTToUnetAdapter`` normalizes each map, projects it to the channel count
   the official decoder expects and resamples it to the matching stride.

The adapter is new (no pretrained weights), so it is trainable even when the
DeFM backbone is frozen. That is exactly the "freeze the encoder, retrain the
decoder + two heads" configuration.

Known approximation: the ViT grid is stride 14 while the UNet expects stride 32
at the coarsest level, so the resampling is geometric rather than exact. It is
documented in ``docs/17-frontiernet.md`` and does not affect checkpoint
compatibility, because none of the official encoder weights are reused.
"""

from __future__ import annotations

from typing import List, Optional, Sequence

import torch
import torch.nn as nn
import torch.nn.functional as F

__all__ = [
    "DEFM_OUT_INDICES",
    "DEFM_UNET_CHANNELS",
    "DefmEncoder",
    "DefmViTEncoder",
    "ViTToUnetAdapter",
]

DEFM_OUT_INDICES: Sequence[int] = (2, 4, 6, 8, 11)
DEFM_UNET_CHANNELS: Sequence[int] = (64, 64, 128, 256, 512)
DEFM_UNET_STRIDES: Sequence[int] = (2, 4, 8, 16, 32)


class DefmViTEncoder(nn.Module):
    """Thin wrapper over ``thirdparty.defm`` that returns five token grids."""

    def __init__(
        self,
        model_name: str = "defm_vit_s14",
        weights_path: Optional[str] = None,
        out_indices: Sequence[int] = DEFM_OUT_INDICES,
        input_size: int = 224,
    ) -> None:
        super().__init__()
        from thirdparty.defm.model_factory import create_defm_model

        self.model_name = model_name
        self.input_size = int(input_size)
        self.out_indices = tuple(int(index) for index in out_indices)
        self.backbone = create_defm_model(
            model_name,
            pretrained=weights_path is not None,
            pretrained_path=weights_path,
        )
        self.embed_dim = int(getattr(self.backbone, "embed_dim", 384))
        self.patch_size = int(getattr(self.backbone, "patch_size", 14))

    @property
    def grid_size(self) -> int:
        return self.input_size // self.patch_size

    def forward(self, x: torch.Tensor) -> List[torch.Tensor]:
        features = self.backbone.get_intermediate_layers(
            x, n=list(self.out_indices), reshape=True
        )
        return [feature for feature in features]


class ViTToUnetAdapter(nn.Module):
    """Project and resample ViT token grids onto the UNet skip contract."""

    def __init__(
        self,
        in_dim: int = 384,
        out_channels: Sequence[int] = DEFM_UNET_CHANNELS,
        strides: Sequence[int] = DEFM_UNET_STRIDES,
        input_size: int = 224,
        use_norm: bool = True,
    ) -> None:
        super().__init__()
        if len(out_channels) != len(strides):
            raise ValueError("out_channels and strides must have the same length")
        self.in_dim = int(in_dim)
        self.out_channels = tuple(int(channel) for channel in out_channels)
        self.strides = tuple(int(stride) for stride in strides)
        self.input_size = int(input_size)
        self.sizes = tuple(int(input_size) // stride for stride in self.strides)
        self.norms = nn.ModuleList(
            [
                nn.GroupNorm(1, self.in_dim) if use_norm else nn.Identity()
                for _ in self.out_channels
            ]
        )
        self.projections = nn.ModuleList(
            [nn.Conv2d(self.in_dim, channel, kernel_size=1) for channel in self.out_channels]
        )

    def target_sizes(self, size: Optional[Sequence[int]] = None) -> List[tuple]:
        """Per-level ``(height, width)`` for an input of ``size``."""
        if size is None:
            height = width = int(self.input_size)
        else:
            height, width = int(size[0]), int(size[1])
        return [(height // stride, width // stride) for stride in self.strides]

    def forward(
        self,
        features: Sequence[torch.Tensor],
        size: Optional[Sequence[int]] = None,
    ) -> List[torch.Tensor]:
        if len(features) != len(self.out_channels):
            raise ValueError(
                f"expected {len(self.out_channels)} ViT feature maps, got {len(features)}"
            )
        outputs = []
        targets = self.target_sizes(size)
        for feature, norm, projection, target in zip(
            features, self.norms, self.projections, targets
        ):
            projected = projection(norm(feature))
            if tuple(projected.shape[-2:]) != target:
                projected = F.interpolate(
                    projected, size=target, mode="bilinear", align_corners=False
                )
            outputs.append(projected)
        return outputs


class DefmEncoder(nn.Module):
    """Encoder interface expected by :class:`frontiernet.unet.UnetDecoder`.

    ``forward`` returns six features ``[input, s2, s4, s8, s16, s32]`` so it can
    be passed straight to the official decoder, which drops ``features[0]``.
    """

    def __init__(
        self,
        model_name: str = "defm_vit_s14",
        weights_path: Optional[str] = None,
        input_size: int = 224,
        out_channels: Sequence[int] = DEFM_UNET_CHANNELS,
        out_indices: Sequence[int] = DEFM_OUT_INDICES,
        freeze_backbone: bool = True,
        backbone: Optional[nn.Module] = None,
    ) -> None:
        super().__init__()
        self.input_size = int(input_size)
        self.backbone = (
            backbone
            if backbone is not None
            else DefmViTEncoder(
                model_name=model_name,
                weights_path=weights_path,
                out_indices=out_indices,
                input_size=input_size,
            )
        )
        in_dim = int(getattr(self.backbone, "embed_dim", 384))
        self.adapter = ViTToUnetAdapter(
            in_dim=in_dim, out_channels=out_channels, input_size=input_size
        )
        self._out_channels = (3,) + tuple(int(channel) for channel in out_channels)
        self.set_backbone_frozen(freeze_backbone)

    @property
    def out_channels(self) -> List[int]:
        """Same contract as ``ResNet34Encoder.out_channels`` (input channel first)."""
        return list(self._out_channels)

    @property
    def output_stride(self) -> int:
        return 32

    @property
    def in_channels(self) -> int:
        return 3

    def set_backbone_frozen(self, frozen: bool) -> None:
        for parameter in self.backbone.parameters():
            parameter.requires_grad = not frozen

    def backbone_frozen(self) -> bool:
        return not any(parameter.requires_grad for parameter in self.backbone.parameters())

    def forward(self, x: torch.Tensor) -> List[torch.Tensor]:
        return [x, *self.adapter(self.backbone(x), size=x.shape[-2:])]
