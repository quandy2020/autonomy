"""Export the DeFM ViT-S/14 checkpoint used by VNTPA to ONNX (opset 17)."""

from __future__ import annotations

import argparse
import os
import sys

import torch


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", default="conf/models/defm_vit_s14.pth")
    parser.add_argument("--out", default="conf/models/defm_vit_s14.onnx")
    parser.add_argument(
        "--defm-source",
        default="third_party/defm",
        help="Path containing the upstream defm Python package",
    )
    args = parser.parse_args()
    sys.path.insert(0, os.path.abspath(args.defm_source))
    os.environ.setdefault("XFORMERS_DISABLED", "1")
    from defm.models.vision_transformer import vit_small  # noqa: PLC0415

    model = vit_small(
        img_size=224,
        patch_size=14,
        in_chans=3,
        init_values=1e-5,
        ffn_layer="swiglufused",
        block_chunks=4,
        qkv_bias=True,
        proj_bias=True,
        ffn_bias=True,
        num_register_tokens=0,
        interpolate_offset=0.1,
        interpolate_antialias=False,
    )
    state = torch.load(args.checkpoint, map_location="cpu", weights_only=True)
    model.load_state_dict(state.get("model", state))
    model.eval()

    class Export(torch.nn.Module):
        def __init__(self, encoder: torch.nn.Module) -> None:
            super().__init__()
            self.encoder = encoder

        def forward(self, depth: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
            tokens = self.encoder.get_intermediate_layers(depth, n=1, reshape=False)[0]
            return tokens, tokens.mean(dim=1)

    torch.onnx.export(
        Export(model),
        torch.randn(1, 3, 224, 224),
        args.out,
        opset_version=17,
        input_names=["depth"],
        output_names=["tokens", "pooled"],
        dynamo=False,
    )
    print("exported", args.out)


if __name__ == "__main__":
    main()
