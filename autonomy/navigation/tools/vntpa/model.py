"""VNTPA transformer planner."""

from __future__ import annotations

import torch
import torch.nn as nn


class VntpaPlanner(nn.Module):
    def __init__(self, token_dim: int = 384, pose_dim: int = 64,
                 context_size: int = 32, chunk_length: int = 8,
                 layers: int = 4, heads: int = 8):
        super().__init__()
        self.context_size = context_size
        self.chunk_length = chunk_length
        self.depth_proj = nn.Linear(token_dim, token_dim)
        self.query_proj = nn.Linear(pose_dim, token_dim)
        self.context_proj = nn.Linear(token_dim + pose_dim, token_dim)
        encoder = nn.TransformerEncoderLayer(d_model=token_dim, nhead=heads,
                                             batch_first=True, dim_feedforward=token_dim * 4)
        self.encoder = nn.TransformerEncoder(encoder, num_layers=layers)
        self.head = nn.Linear(token_dim, 3)

    def forward(self, depth_feature: torch.Tensor, query: torch.Tensor,
                context: torch.Tensor, context_pose: torch.Tensor,
                context_mask: torch.Tensor) -> torch.Tensor:
        # depth_feature: [B,N,D], query: [B,D], context: [B,K,D], pose: [B,K,P]
        query_token = self.query_proj(query).unsqueeze(1)
        context_token = self.context_proj(torch.cat([context, context_pose], dim=-1))
        tokens = torch.cat([query_token, self.depth_proj(depth_feature), context_token], dim=1)
        mask = torch.cat([
            torch.zeros((query.shape[0], 1 + depth_feature.shape[1]), dtype=torch.bool,
                        device=query.device),
            context_mask.bool(),
        ], dim=1)
        encoded = self.encoder(tokens, src_key_padding_mask=mask)
        decoded = self.head(encoded[:, 0])
        return decoded.unsqueeze(1).repeat(1, self.chunk_length, 1)
