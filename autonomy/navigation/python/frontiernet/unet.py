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

"""FrontierNet ``TwoHeadUnet`` without the ``segmentation_models_pytorch`` dependency.

The official model is ``smp.Unet``-shaped: a ResNet-34 encoder, a 5-block UNet
decoder and two heads (11-way class logits + distance field). It is rebuilt here
with the *same module names and channel layout* so that

* ``rgbd_11cls.pth`` loads with ``strict=True``, and
* the decoder/heads can be transplanted onto a different encoder.

Layout (input 224x224)::

    encoder features  [x, s2, s4, s8, s16, s32]
    channels          [3, 64, 64, 128, 256, 512]
    decoder blocks    (512+256)->256, (256+128)->128, (128+64)->64,
                      (64+64)->32, (32+0)->16
    heads             segmentation 16->11, regression 16->1

``smp``'s ``UnetDecoder`` drops the first encoder feature (same resolution as the
input), which is why six features are produced and only five are consumed.
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Union

import torch
import torch.nn as nn
import torch.nn.functional as F

__all__ = [
    "Conv2dReLU",
    "DecoderBlock",
    "DfRegressionHead",
    "PredictionModel",
    "ResNet34Encoder",
    "SegmentationHead",
    "TwoHeadUnet",
    "UnetDecoder",
    "initialize_head",
    "initialize_module",
]

RESNET34_CHANNELS: Sequence[int] = (3, 64, 64, 128, 256, 512)
DEFAULT_DECODER_CHANNELS: Sequence[int] = (256, 128, 64, 32, 16)


def initialize_module(module: nn.Module) -> None:
    """``smp.base.initialization.initialize_decoder`` equivalent."""
    for child in module.modules():
        if isinstance(child, nn.Conv2d):
            nn.init.kaiming_uniform_(child.weight, mode="fan_in", nonlinearity="relu")
            if child.bias is not None:
                nn.init.zeros_(child.bias)
        elif isinstance(child, nn.BatchNorm2d):
            nn.init.ones_(child.weight)
            nn.init.zeros_(child.bias)


def initialize_head(module: nn.Module) -> None:
    """``smp.base.initialization.initialize_head``: xavier uniform, zero bias."""
    for child in module.modules():
        if isinstance(child, (nn.Linear, nn.Conv2d)):
            nn.init.xavier_uniform_(child.weight)
            if child.bias is not None:
                nn.init.zeros_(child.bias)


class _Activation(nn.Module):
    """Minimal stand-in for ``smp.base.modules.Activation``."""

    def __init__(self, name: Optional[Union[str, nn.Module]] = None) -> None:
        super().__init__()
        if name is None:
            self.activation: nn.Module = nn.Identity()
        elif isinstance(name, nn.Module):
            self.activation = name
        elif name == "sigmoid":
            self.activation = nn.Sigmoid()
        elif name == "softmax":
            self.activation = nn.Softmax(dim=1)
        elif name == "identity":
            self.activation = nn.Identity()
        else:
            raise ValueError(f"unsupported activation {name!r}")

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.activation(x)


class Conv2dReLU(nn.Sequential):
    """``conv -> batchnorm -> relu`` with smp's exact submodule ordering."""

    def __init__(
        self,
        in_channels: int,
        out_channels: int,
        kernel_size: int,
        padding: int = 0,
        stride: int = 1,
        use_batchnorm: bool = True,
    ) -> None:
        conv = nn.Conv2d(
            in_channels,
            out_channels,
            kernel_size,
            stride=stride,
            padding=padding,
            bias=not use_batchnorm,
        )
        bn: nn.Module = nn.BatchNorm2d(out_channels) if use_batchnorm else nn.Identity()
        super().__init__(conv, bn, nn.ReLU(inplace=True))


class DecoderBlock(nn.Module):
    """One UNet decoder stage: upsample, concat skip, two 3x3 convs."""

    def __init__(
        self,
        in_channels: int,
        skip_channels: int,
        out_channels: int,
        use_batchnorm: bool = True,
    ) -> None:
        super().__init__()
        self.conv1 = Conv2dReLU(
            in_channels + skip_channels, out_channels, kernel_size=3, padding=1,
            use_batchnorm=use_batchnorm,
        )
        self.conv2 = Conv2dReLU(
            out_channels, out_channels, kernel_size=3, padding=1,
            use_batchnorm=use_batchnorm,
        )

    def forward(self, x: torch.Tensor, skip: Optional[torch.Tensor] = None) -> torch.Tensor:
        x = F.interpolate(x, scale_factor=2, mode="nearest")
        if skip is not None:
            x = torch.cat([x, skip], dim=1)
        return self.conv2(self.conv1(x))


class UnetDecoder(nn.Module):
    """smp-compatible decoder; ``encoder_channels`` includes the input channel."""

    def __init__(
        self,
        encoder_channels: Sequence[int],
        decoder_channels: Sequence[int] = DEFAULT_DECODER_CHANNELS,
        n_blocks: int = 5,
        use_batchnorm: bool = True,
    ) -> None:
        super().__init__()
        if n_blocks != len(decoder_channels):
            raise ValueError(
                f"model depth is {n_blocks}, but {len(decoder_channels)} decoder "
                "channel entries were provided"
            )
        channels = list(encoder_channels[1:])[::-1]
        head_channels = channels[0]
        in_channels = [head_channels] + list(decoder_channels[:-1])
        skip_channels = list(channels[1:]) + [0]
        self.blocks = nn.ModuleList(
            [
                DecoderBlock(in_ch, skip_ch, out_ch, use_batchnorm=use_batchnorm)
                for in_ch, skip_ch, out_ch in zip(in_channels, skip_channels, decoder_channels)
            ]
        )

    def forward(self, *features: torch.Tensor) -> torch.Tensor:
        features = list(features[1:])[::-1]
        x = features[0]
        skips = features[1:]
        for index, block in enumerate(self.blocks):
            skip = skips[index] if index < len(skips) else None
            x = block(x, skip)
        return x


class SegmentationHead(nn.Sequential):
    """Official class head: two 3x3 convs then a 1x1 projection."""

    def __init__(
        self,
        in_channels: int,
        out_channels: int,
        kernel_size: int = 3,
        activation: Optional[str] = None,
        upsampling: int = 1,
    ) -> None:
        super().__init__(
            nn.Conv2d(in_channels, 64, kernel_size=kernel_size, padding=kernel_size // 2),
            nn.ReLU(),
            nn.BatchNorm2d(64),
            nn.Conv2d(64, 64, kernel_size=kernel_size, padding=kernel_size // 2),
            nn.ReLU(),
            nn.BatchNorm2d(64),
            nn.Conv2d(64, out_channels, kernel_size=1),
            nn.Upsample(scale_factor=upsampling) if upsampling > 1 else nn.Identity(),
            _Activation(activation),
        )


class DfRegressionHead(nn.Sequential):
    """Official distance-field head; ``downsample_factor`` must be 1.0 for 11cls."""

    def __init__(
        self,
        in_channels: int,
        out_channels: int = 1,
        downsample_factor: float = 1.0,
        kernel_size: int = 3,
    ) -> None:
        super().__init__(
            nn.Conv2d(in_channels, 64, kernel_size=kernel_size, padding=kernel_size // 2),
            nn.ReLU(),
            nn.BatchNorm2d(64),
            nn.Conv2d(64, 64, kernel_size=kernel_size, padding=kernel_size // 2),
            nn.ReLU(),
            nn.BatchNorm2d(64),
            nn.Conv2d(64, out_channels, kernel_size=1),
            (
                nn.Conv2d(
                    out_channels,
                    out_channels,
                    kernel_size=3,
                    stride=int(1 / downsample_factor),
                    padding=1,
                )
                if downsample_factor != 1
                else nn.Identity()
            ),
            nn.ReLU(),
        )
        self.downsample_factor = downsample_factor


class ResNet34Encoder(nn.Module):
    """smp ``ResNetEncoder`` stage layout with the official 4-channel conv1 patch."""

    def __init__(
        self,
        in_channels: int = 4,
        depth: int = 5,
        weights: Optional[str] = None,
    ) -> None:
        super().__init__()
        from torchvision.models import resnet34
        from torchvision.models.resnet import ResNet34_Weights

        torch_weights = None
        if weights == "imagenet":
            torch_weights = ResNet34_Weights.IMAGENET1K_V1
        backbone = resnet34(weights=torch_weights)
        self.conv1 = backbone.conv1
        self.bn1 = backbone.bn1
        self.relu = backbone.relu
        self.maxpool = backbone.maxpool
        self.layer1 = backbone.layer1
        self.layer2 = backbone.layer2
        self.layer3 = backbone.layer3
        self.layer4 = backbone.layer4
        self.depth = depth
        self._out_channels = list(RESNET34_CHANNELS)
        self._out_channels[0] = in_channels
        if in_channels != 3:
            self.conv1 = self.modify_first_conv(self.conv1, in_channels)

    @property
    def out_channels(self) -> List[int]:
        return list(self._out_channels[: self.depth + 1])

    @property
    def output_stride(self) -> int:
        return 32

    @staticmethod
    def modify_first_conv(conv_layer: nn.Conv2d, in_channels: int) -> nn.Conv2d:
        """Official channel adaptation: slice or tile the pretrained kernel."""
        if in_channels == conv_layer.in_channels:
            return conv_layer
        new_conv = nn.Conv2d(
            in_channels,
            conv_layer.out_channels,
            kernel_size=conv_layer.kernel_size,
            stride=conv_layer.stride,
            padding=conv_layer.padding,
            bias=conv_layer.bias is not None,
        )
        with torch.no_grad():
            if in_channels < conv_layer.in_channels:
                new_conv.weight.data = conv_layer.weight.data[:, :in_channels, :, :]
            else:
                new_conv.weight.data[:, : conv_layer.in_channels, :, :] = conv_layer.weight.data
                new_conv.weight.data[:, conv_layer.in_channels :, :, :] = conv_layer.weight.data[
                    :, : in_channels - conv_layer.in_channels, :, :
                ]
        return new_conv

    def forward(self, x: torch.Tensor) -> List[torch.Tensor]:
        features = [x]
        x = self.relu(self.bn1(self.conv1(x)))
        features.append(x)
        x = self.maxpool(x)
        for layer in (self.layer1, self.layer2, self.layer3, self.layer4):
            x = layer(x)
            features.append(x)
        return features[: self.depth + 1]


class PredictionModel(nn.Module):
    """Shared forward/padding logic of the official ``PredictionModel``."""

    output_stride: int = 32
    n_classes: int = 1
    n_channels: int = 4

    def initialize(self) -> None:
        initialize_module(self.decoder)
        initialize_head(self.segmentation_head)
        initialize_head(self.df_regression_head)

    def forward(self, x: torch.Tensor):
        height, width = x.shape[-2:]
        pad_h = (-height) % self.output_stride
        pad_w = (-width) % self.output_stride
        if pad_h or pad_w:
            x = F.pad(x, (0, pad_w, 0, pad_h))
        features = self.encoder(x)
        decoded = self.decoder(*features)
        masks = self.segmentation_head(decoded)
        df = self.df_regression_head(decoded)
        if pad_h or pad_w:
            masks = masks[:, :, :height, :width]
            factor = getattr(self.df_regression_head, "downsample_factor", 1.0)
            df = df[:, :, : int(height * factor), : int(width * factor)]
        return df, masks

    @torch.no_grad()
    def predict(self, x: torch.Tensor):
        self.eval()
        return self.forward(x)


class TwoHeadUnet(PredictionModel):
    """Official FrontierNet model (``rgbd_11cls.pth`` target)."""

    def __init__(
        self,
        encoder_name: str = "resnet34",
        encoder_depth: int = 5,
        encoder_weights: Optional[str] = None,
        decoder_use_batchnorm: bool = True,
        decoder_channels: Sequence[int] = DEFAULT_DECODER_CHANNELS,
        in_channels: int = 4,
        classes: int = 11,
        activation: Optional[str] = None,
        regression_downsample_factor: float = 1.0,
    ) -> None:
        super().__init__()
        if encoder_name != "resnet34":
            raise ValueError("only the resnet34 encoder is supported")
        self.encoder = ResNet34Encoder(
            in_channels=in_channels, depth=encoder_depth, weights=encoder_weights
        )
        self.decoder = UnetDecoder(
            encoder_channels=self.encoder.out_channels,
            decoder_channels=decoder_channels,
            n_blocks=encoder_depth,
            use_batchnorm=decoder_use_batchnorm,
        )
        self.segmentation_head = SegmentationHead(
            in_channels=decoder_channels[-1],
            out_channels=classes,
            activation=activation,
            kernel_size=3,
        )
        self.df_regression_head = DfRegressionHead(
            in_channels=decoder_channels[-1],
            out_channels=1,
            downsample_factor=regression_downsample_factor,
            kernel_size=3,
        )
        self.name = f"u-{encoder_name}"
        self.output_stride = 32
        self.n_classes = classes
        self.n_channels = in_channels
        self.initialize()
