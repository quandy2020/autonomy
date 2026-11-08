"""Export a VNTPA checkpoint to ONNX opset 17."""

from __future__ import annotations

import argparse

import torch

from model import VntpaPlanner


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--token-count", type=int, default=256)
    parser.add_argument("--token-dim", type=int, default=384)
    parser.add_argument("--context-size", type=int, default=32)
    parser.add_argument("--pose-dim", type=int, default=64)
    parser.add_argument("--chunk-length", type=int, default=8)
    args = parser.parse_args()
    model = VntpaPlanner(token_dim=args.token_dim, pose_dim=args.pose_dim,
                         context_size=args.context_size, chunk_length=args.chunk_length)
    model.load_state_dict(torch.load(args.checkpoint, map_location="cpu"))
    model.eval()
    dummy = (
        torch.randn(1, args.token_count, args.token_dim),
        torch.randn(1, args.pose_dim),
        torch.randn(1, args.context_size, args.token_dim),
        torch.randn(1, args.context_size, args.pose_dim),
        torch.zeros(1, args.context_size, dtype=torch.bool),
    )
    torch.onnx.export(
        model, dummy, args.out, opset_version=17,
        input_names=["depth_feature", "query_token", "context", "context_pose",
                     "context_mask"],
        output_names=["action_chunk"],
        dynamic_axes=None,
    )
    print("exported", args.out)


if __name__ == "__main__":
    main()
