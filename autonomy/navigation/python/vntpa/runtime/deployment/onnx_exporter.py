"""Export a trained VNT-PA Planner to a fixed-shape single planner ONNX."""

from __future__ import annotations

import argparse

import torch


class FixedPlanner(torch.nn.Module):
    """Fix K/T and expose the C++/ONNX tensor contract."""

    def __init__(self, planner: torch.nn.Module, context_size: int) -> None:
        super().__init__()
        self.planner = planner
        self.context_size = context_size

    def forward(self, kf_tokens: torch.Tensor, kf_pose: torch.Tensor,
                kf_valid: torch.Tensor, query_pose: torch.Tensor,
                goal: torch.Tensor) -> torch.Tensor:
        valid = kf_valid.bool()
        valid_batch = valid.unsqueeze(0)
        memory = self.planner.encode_map(
            self.planner.summarize(kf_tokens[valid], valid_batch),
            kf_pose.unsqueeze(0), valid_batch, goal.unsqueeze(0))
        return self.planner.read(memory, query_pose.unsqueeze(0), goal.unsqueeze(0))[:, 0]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--out", default="conf/models/vntpa.onnx")
    parser.add_argument("--context-size", type=int, default=64)
    parser.add_argument("--token-count", type=int, default=256)
    parser.add_argument("--token-dim", type=int, default=384)
    parser.add_argument("--chunk", type=int, default=8)
    args = parser.parse_args()

    from thirdparty.pan.planner.train import load_planner

    planner, _ = load_planner(args.checkpoint)
    planner.eval()
    wrapper = FixedPlanner(planner, args.context_size).eval()

    dummy = (
        torch.randn(args.context_size, args.token_count, args.token_dim),
        torch.randn(args.context_size, 3),
        torch.ones(args.context_size),
        torch.randn(1, 3),
        torch.randn(1, 2),
    )
    torch.onnx.export(
        wrapper, dummy, args.out, opset_version=17,
        input_names=["kf_tokens", "kf_pose", "kf_valid", "query_pose", "goal"],
        output_names=["logits"], dynamo=False)
    print(f"exported {args.out}")


if __name__ == "__main__":
    main()
