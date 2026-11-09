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

"""FrontierNet with the ResNet-34 encoder swapped for DeFM.

The decoder and both heads are the *official* modules, so ``rgbd_11cls.pth``
supplies their initial weights. Only the adapter between DeFM and the decoder
has no pretrained counterpart, which is why the default training recipe is

    frozen DeFM backbone  +  trainable adapter + decoder + two heads

``parameter_groups`` returns the three learning rates that recipe needs.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence

import torch
import torch.nn as nn

from frontiernet.encoder_defm import DEFM_OUT_INDICES, DEFM_UNET_CHANNELS, DefmEncoder
from frontiernet.unet import (
    DEFAULT_DECODER_CHANNELS,
    DfRegressionHead,
    SegmentationHead,
    UnetDecoder,
    initialize_head,
    initialize_module,
)

__all__ = ["FrontierNetDeFM"]

HEAD_PREFIXES = ("decoder.", "segmentation_head.", "df_regression_head.")


class FrontierNetDeFM(nn.Module):
    """DeFM encoder + official UNet decoder + official two heads."""

    def __init__(
        self,
        classes: int = 11,
        defm_model: str = "defm_vit_s14",
        defm_weights: Optional[str] = None,
        input_size: int = 224,
        decoder_channels: Sequence[int] = DEFAULT_DECODER_CHANNELS,
        regression_downsample_factor: float = 1.0,
        freeze_backbone: bool = True,
        backbone: Optional[nn.Module] = None,
    ) -> None:
        super().__init__()
        self.encoder = DefmEncoder(
            model_name=defm_model,
            weights_path=defm_weights,
            input_size=input_size,
            out_channels=DEFM_UNET_CHANNELS,
            out_indices=DEFM_OUT_INDICES,
            freeze_backbone=freeze_backbone,
            backbone=backbone,
        )
        self.decoder = UnetDecoder(
            encoder_channels=self.encoder.out_channels,
            decoder_channels=decoder_channels,
            n_blocks=len(decoder_channels),
            use_batchnorm=True,
        )
        self.segmentation_head = SegmentationHead(
            in_channels=decoder_channels[-1], out_channels=classes
        )
        self.df_regression_head = DfRegressionHead(
            in_channels=decoder_channels[-1],
            out_channels=1,
            downsample_factor=regression_downsample_factor,
        )
        self.input_size = int(input_size)
        self.output_stride = 32
        self.n_classes = int(classes)
        self.n_channels = 3
        self.initialize_heads()

    def initialize_heads(self) -> None:
        """smp-style init for the parts that have no official checkpoint."""
        initialize_module(self.decoder)
        initialize_head(self.segmentation_head)
        initialize_head(self.df_regression_head)

    def forward(self, depth: torch.Tensor):
        """``depth`` is ``(B, 3, H, W)`` DeFM-normalized input; returns ``(df, masks)``."""
        height, width = depth.shape[-2:]
        pad_h = (-height) % self.output_stride
        pad_w = (-width) % self.output_stride
        x = depth
        if pad_h or pad_w:
            x = nn.functional.pad(x, (0, pad_w, 0, pad_h))
        decoded = self.decoder(*self.encoder(x))
        masks = self.segmentation_head(decoded)
        df = self.df_regression_head(decoded)
        if pad_h or pad_w:
            masks = masks[:, :, :height, :width]
            df = df[:, :, :height, :width]
        return df, masks

    @torch.no_grad()
    def predict(self, depth: torch.Tensor):
        self.eval()
        return self.forward(depth)

    # -- freezing ---------------------------------------------------------

    def set_backbone_frozen(self, frozen: bool) -> None:
        self.encoder.set_backbone_frozen(frozen)

    def set_adapter_frozen(self, frozen: bool) -> None:
        for parameter in self.encoder.adapter.parameters():
            parameter.requires_grad = not frozen

    def set_heads_frozen(self, frozen: bool) -> None:
        for name, parameter in self.named_parameters():
            if name.startswith(HEAD_PREFIXES):
                parameter.requires_grad = not frozen

    def head_parameters(self) -> List[nn.Parameter]:
        return [
            parameter
            for name, parameter in self.named_parameters()
            if name.startswith(HEAD_PREFIXES)
        ]

    def adapter_parameters(self) -> List[nn.Parameter]:
        return list(self.encoder.adapter.parameters())

    def backbone_parameters(self) -> List[nn.Parameter]:
        return list(self.encoder.backbone.parameters())

    def trainable_parameters(self) -> List[nn.Parameter]:
        return [parameter for parameter in self.parameters() if parameter.requires_grad]

    def parameter_groups(
        self,
        head_lr: float,
        adapter_lr: Optional[float] = None,
        backbone_lr: Optional[float] = None,
    ) -> List[Dict]:
        """Return optimizer groups; frozen groups are omitted."""
        groups: List[Dict] = []
        adapter_lr = head_lr if adapter_lr is None else adapter_lr
        backbone_lr = adapter_lr if backbone_lr is None else backbone_lr
        backbone = [p for p in self.backbone_parameters() if p.requires_grad]
        adapter = [p for p in self.adapter_parameters() if p.requires_grad]
        heads = [p for p in self.head_parameters() if p.requires_grad]
        if backbone:
            groups.append({"params": backbone, "lr": backbone_lr, "name": "backbone"})
        if adapter:
            groups.append({"params": adapter, "lr": adapter_lr, "name": "adapter"})
        if heads:
            groups.append({"params": heads, "lr": head_lr, "name": "heads"})
        return groups

    def describe(self) -> Dict[str, int]:
        def count(parameters) -> int:
            return sum(parameter.numel() for parameter in parameters)

        return {
            "backbone_total": count(self.backbone_parameters()),
            "backbone_trainable": count(
                [p for p in self.backbone_parameters() if p.requires_grad]
            ),
            "adapter_total": count(self.adapter_parameters()),
            "heads_total": count(self.head_parameters()),
            "trainable_total": count(self.trainable_parameters()),
            "total": count(self.parameters()),
        }

    def extra_repr(self) -> str:
        return (
            f"classes={self.n_classes}, input_size={self.input_size}, "
            f"backbone_frozen={self.encoder.backbone_frozen()}"
        )
