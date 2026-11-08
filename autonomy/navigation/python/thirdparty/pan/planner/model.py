"""Planner: imitation of the navmesh expert from a bag of pose-stamped tour keyframes.

    summarize   each keyframe's frozen DeFM patch tokens -> one map entry
    encode_map  entries (+ a goal token at the goal position, no heading, when goal_entry)
                mixed by self-attention
    read        one query per agent pose cross-attends the map
    decode      an autoregressive action chunk (GRU; teacher-forced in training)

pos_mode "spatial": 3-axis RoPE on (x, z, yaw) for q and k.
"temporal" (control arm): 1-D RoPE on the tour rank, with poses added as content.
"none" (control arm, VNT-PF): no positional embedding, poses added as content only.
"""
from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from typing import Optional

import torch
import torch.nn as nn
import torch.nn.functional as F

from thirdparty.pan.util.config import arg
from thirdparty.pan.util.constants import DEFM_TOKEN_DIM, N_ACTIONS

POS_MODES = ("spatial", "temporal", "none")
_CONTENT_METRES = 10.0          # metres per unit of the pose-as-content features


# ---------------------------------------------------------------- RoPE
def _rope_freqs(n_pairs: int, base: float) -> torch.Tensor:
    return 1.0 / (base ** (torch.arange(n_pairs, dtype=torch.float32) / n_pairs))


def _rotate_pairs(x: torch.Tensor, angle: torch.Tensor) -> torch.Tensor:
    """Rotate consecutive feature pairs of x (..., 2p) by angle (..., p)."""
    c, s = angle.cos(), angle.sin()
    g = x.reshape(*x.shape[:-1], -1, 2)
    return torch.stack([g[..., 0] * c - g[..., 1] * s,
                        g[..., 0] * s + g[..., 1] * c], dim=-1).flatten(-2)


def apply_1d_rope(x: torch.Tensor, t: torch.Tensor, freqs: torch.Tensor) -> torch.Tensor:
    """x (..., head_dim) at positions t (...,)."""
    return _rotate_pairs(x, t.unsqueeze(-1) * freqs)


def apply_3d_rope(x: torch.Tensor, pose: torch.Tensor, freqs_xz: torch.Tensor,
                  freqs_yaw: torch.Tensor) -> torch.Tensor:
    """x (..., head_dim) at pose (..., 3): x, z and yaw on disjoint thirds of head_dim."""
    third = x.shape[-1] // 3
    return torch.cat([_rotate_pairs(x[..., :third], pose[..., 0:1] * freqs_xz),
                      _rotate_pairs(x[..., third:2 * third], pose[..., 1:2] * freqs_xz),
                      _rotate_pairs(x[..., 2 * third:], pose[..., 2:3] * freqs_yaw)], dim=-1)


def compile_rope() -> None:
    """Swap the RoPE kernels for torch.compile(dynamic=True) versions; `RoPE` looks them up at
    call time."""
    global apply_1d_rope, apply_3d_rope
    apply_1d_rope = torch.compile(apply_1d_rope, dynamic=True)
    apply_3d_rope = torch.compile(apply_3d_rope, dynamic=True)


class RoPE(nn.Module):
    """Rotary position: pose (x, z, yaw) when `spatial`, x and z at shared metric frequencies
    and yaw at integer harmonics (2 pi periodic); otherwise the 1-D rank."""

    def __init__(self, head_dim: int, spatial: bool, base: float = 100.0,
                 metres_per_unit: float = 1.0):
        super().__init__()
        self.spatial = spatial
        if spatial:
            assert head_dim % 6 == 0, f"head dim {head_dim} must divide by 6 for (x, z, yaw) RoPE"
            n = head_dim // 6
            self.register_buffer("freqs_xz", _rope_freqs(n, base) / float(metres_per_unit),
                                 persistent=False)
            self.register_buffer("freqs_yaw", torch.arange(1, n + 1, dtype=torch.float32),
                                 persistent=False)
            self.register_buffer("yaw_dims", torch.arange(head_dim) >= 2 * (head_dim // 3),
                                 persistent=False)
        else:
            assert head_dim % 2 == 0, f"head dim {head_dim} must be even for 1-D RoPE"
            self.register_buffer("freqs", _rope_freqs(head_dim // 2, 10000.0), persistent=False)

    def forward(self, x: torch.Tensor, pos: torch.Tensor,
                yaw_free: Optional[torch.Tensor] = None) -> torch.Tensor:
        """x (B, H, N, head_dim) at pos, (B, N, 3) poses or (B, N) ranks. Spatial: tokens
        marked `yaw_free` (B, N) have their yaw third zeroed."""
        if not self.spatial:
            return apply_1d_rope(x, pos.unsqueeze(1), self.freqs)
        if yaw_free is not None:
            x = x.masked_fill(yaw_free[:, None, :, None] & self.yaw_dims, 0.0)
        return apply_3d_rope(x, pos.unsqueeze(1), self.freqs_xz, self.freqs_yaw)


# ---------------------------------------------------------------- blocks
class Attention(nn.Module):
    """Multi-head attention with RoPE on queries and keys (no positional embedding when `rope`
    is None): self-attention over x, or cross-attention from x to `context` when given."""

    def __init__(self, dim: int, num_heads: int, rope: Optional[nn.Module], cross: bool):
        super().__init__()
        assert dim % num_heads == 0, f"dim {dim} must divide into {num_heads} heads"
        self.num_heads, self.head_dim, self.rope = num_heads, dim // num_heads, rope
        if cross:
            self.q = nn.Linear(dim, dim)
            self.kv = nn.Linear(dim, 2 * dim)
        else:
            self.qkv = nn.Linear(dim, 3 * dim)
        self.out = nn.Linear(dim, dim)

    def forward(self, x, pos, mask, yaw_free=None, context=None, context_pos=None):
        """x (B, T, D) at pos; `yaw_free` marks the key tokens (in self-attention also the
        queries) whose heading RoPE ignores."""
        B, T, D = x.shape
        if context is None:
            q, k, v = self.qkv(x).reshape(B, T, 3, self.num_heads, self.head_dim).permute(2, 0, 3, 1, 4)
            if self.rope is not None:
                q, k = self.rope(q, pos, yaw_free), self.rope(k, pos, yaw_free)
        else:
            N = context.shape[1]
            q = self.q(x).reshape(B, T, self.num_heads, self.head_dim).transpose(1, 2)
            k, v = self.kv(context).reshape(B, N, 2, self.num_heads, self.head_dim).permute(2, 0, 3, 1, 4)
            if self.rope is not None:
                q, k = self.rope(q, pos), self.rope(k, context_pos, yaw_free)
        o = F.scaled_dot_product_attention(q, k, v, attn_mask=mask)
        return self.out(o.transpose(1, 2).reshape(B, T, D))


class Block(nn.Module):
    """Pre-LN transformer block; `zero_init` starts both residual branches at zero."""

    def __init__(self, dim: int, num_heads: int, ffn_ratio: float, rope: Optional[nn.Module],
                 cross: bool, zero_init: bool):
        super().__init__()
        self.n1 = nn.LayerNorm(dim)
        self.attn = Attention(dim, num_heads, rope, cross)
        self.n2 = nn.LayerNorm(dim)
        hidden = int(dim * ffn_ratio)
        self.ffn = nn.Sequential(nn.Linear(dim, hidden), nn.GELU(), nn.Linear(hidden, dim))
        if zero_init:
            for layer in (self.attn.out, self.ffn[-1]):
                nn.init.zeros_(layer.weight)
                nn.init.zeros_(layer.bias)

    def forward(self, x, *attn_args):
        x = x + self.attn(self.n1(x), *attn_args)
        return x + self.ffn(self.n2(x))


class Summarizer(nn.Module):
    """One keyframe's patch tokens -> one summary token: a learned query attention-pools
    the patches at width `d`, then an MLP."""

    def __init__(self, d: int):
        super().__init__()
        self.in_proj = nn.Linear(DEFM_TOKEN_DIM, d) if d != DEFM_TOKEN_DIM else nn.Identity()
        self.query = nn.Parameter(torch.randn(1, 1, d) * 0.02)
        self.ln_q = nn.LayerNorm(d)
        self.ln_kv = nn.LayerNorm(d)
        self.attn = nn.MultiheadAttention(d, max(1, d // 64), batch_first=True)
        self.ln_mlp = nn.LayerNorm(d)
        self.mlp = nn.Sequential(nn.Linear(d, 2 * d), nn.GELU(), nn.Linear(2 * d, d))

    def forward(self, patches: torch.Tensor) -> torch.Tensor:
        kv = self.ln_kv(self.in_proj(patches))
        q = self.ln_q(self.query.expand(patches.shape[0], -1, -1))
        pooled, _ = self.attn(q, kv, kv, need_weights=False)
        x = self.query.expand_as(pooled) + pooled
        x = x + self.mlp(self.ln_mlp(x))
        return x[:, 0]


def scatter_packed(packed: torch.Tensor, valid: torch.Tensor) -> torch.Tensor:
    """(n_valid, D) rows in the row-major order of `valid`'s True entries -> (B, K, D),
    zeros at padded slots."""
    out = packed.new_zeros(*valid.shape, packed.shape[-1])
    out[valid] = packed
    return out


class ChunkDecoder(nn.Module):
    """Autoregressive action chunk: a GRU whose state starts at the query feature and whose
    input at slot k is the action at slot k-1 (BOS at slot 0)."""

    def __init__(self, dim: int, action_dim: int, chunk: int):
        super().__init__()
        self.chunk = chunk
        self.embed = nn.Embedding(N_ACTIONS + 1, action_dim)       # index N_ACTIONS = BOS
        self.gru = nn.GRUCell(action_dim, dim)
        self.out = nn.Linear(dim, N_ACTIONS)
        nn.init.orthogonal_(self.out.weight, gain=0.01)
        nn.init.zeros_(self.out.bias)

    def forward(self, h: torch.Tensor, teacher_actions: Optional[torch.Tensor] = None):
        """h (B, T, D) -> logits (B, T, chunk, A). Slot k conditions on `teacher_actions`
        (B, T, chunk) when given, else on its own argmax."""
        B, T, D = h.shape
        hidden = h.reshape(B * T, D)
        previous = hidden.new_full((B * T,), N_ACTIONS, dtype=torch.long)
        logits = []
        for k in range(self.chunk):
            hidden = self.gru(self.embed(previous), hidden)
            logits.append(self.out(hidden))
            if teacher_actions is not None:          # padded queries (PAD_TARGET) are loss-masked
                previous = teacher_actions[..., k].reshape(B * T).clamp(0, N_ACTIONS - 1)
            else:
                previous = logits[-1].argmax(-1)
        return torch.stack(logits, 1).reshape(B, T, self.chunk, N_ACTIONS)


# ---------------------------------------------------------------- pose features
def pose_content(pose: torch.Tensor) -> torch.Tensor:
    """(..., 3) pose -> (..., 4) content features: (x, z) / 10 m, sin yaw, cos yaw."""
    return torch.cat([pose[..., :2] / _CONTENT_METRES, pose[..., 2:].sin(), pose[..., 2:].cos()], -1)


def goal_features(pose: torch.Tensor, goal_position: torch.Tensor,
                  metres_per_unit: float = 1.0) -> torch.Tensor:
    """The goal seen from `pose`: (distance / metres_per_unit, cos, sin of its bearing in
    the pose's ego frame)."""
    rel = goal_position - pose[..., :2]
    distance = rel.norm(dim=-1, keepdim=True) / metres_per_unit
    bearing = (torch.atan2(-rel[..., 0], -rel[..., 1]) - pose[..., 2]).unsqueeze(-1)
    return torch.cat([distance, bearing.cos(), bearing.sin()], -1)


GOAL_FEATURES = 3


# ---------------------------------------------------------------- the planner
@dataclass
class PlannerConfig:
    summary_dim: int = arg(384, "width of the per-keyframe summary token")
    model_dim: int = arg(768, "map and reader width; head_dim must divide by 6 (spatial RoPE) "
                              "and 8 (fused SDPA)")
    num_heads: int = arg(8, "attention heads")
    map_layers: int = arg(6, "map self-attention blocks")
    read_layers: int = arg(4, "reader cross-attention blocks")
    ffn_ratio: float = arg(4.0, "MLP width over model width")
    spatial_base: float = arg(100.0, "span of the spatial RoPE frequencies: the longest wavelength, "
                                     "the shortest times base**((p-1)/p), must exceed the largest map")
    spatial_pos_scale: float = arg(0.25, "metres per spatial RoPE unit: the highest frequency turns "
                                         "1 rad per this many metres")
    chunk: int = arg(8, "actions decoded per query")
    act_emb_dim: int = arg(64, "action embedding width of the chunk decoder")
    pos_mode: str = arg("spatial", "spatial: RoPE on (x, z, yaw); temporal: 1-D RoPE on tour rank, "
                                   "poses as content; none (VNT-PF): no positional embedding, poses "
                                   "as content", choices=POS_MODES)
    zero_content: bool = arg(False, "content ablation: map entries are zero, the frozen tokens and "
                                    "summarizer unused")
    goal_entry: bool = arg(True, "goal token in the map; off (spatial only), the goal reaches the model "
                                 "through the reader queries alone")

    def __post_init__(self):
        assert self.pos_mode in POS_MODES, f"pos_mode {self.pos_mode!r} not in {POS_MODES}"
        assert self.goal_entry or self.pos_mode == "spatial", "goal_entry=False needs pos_mode spatial"


class Planner(nn.Module):
    def __init__(self, cfg: PlannerConfig = PlannerConfig()):
        super().__init__()
        self.cfg = cfg
        d = cfg.model_dim
        self.temporal = cfg.pos_mode == "temporal"
        self.pose_as_content = cfg.pos_mode != "spatial"
        rope = (None if cfg.pos_mode == "none" else
                RoPE(d // cfg.num_heads, not self.temporal, cfg.spatial_base, cfg.spatial_pos_scale))
        if not cfg.zero_content:
            self.summarizer = Summarizer(cfg.summary_dim)
            self.entry_proj = _linear_norm(cfg.summary_dim, d)
        self.map_blocks = nn.ModuleList(
            Block(d, cfg.num_heads, cfg.ffn_ratio, rope, cross=False, zero_init=True)
            for _ in range(cfg.map_layers))
        if cfg.goal_entry:
            self.goal_token = nn.Parameter(torch.randn(1, 1, d) * 0.02)
        self.query_token = nn.Parameter(torch.empty(1, 1, d))
        nn.init.normal_(self.query_token, std=0.02)
        self.query_goal_proj = nn.Linear(GOAL_FEATURES, d)
        if self.pose_as_content:
            self.entry_pose_content = _linear_norm(4, d)
            self.query_pose_content = _linear_norm(4, d)
            self.goal_position_content = _linear_norm(2, d)
        self.read_blocks = nn.ModuleList(
            Block(d, cfg.num_heads, cfg.ffn_ratio, rope, cross=True, zero_init=False)
            for _ in range(cfg.read_layers))
        self.map_out_norm = nn.LayerNorm(d)
        self.read_norm = nn.LayerNorm(d)
        self.decoder = ChunkDecoder(d, cfg.act_emb_dim, cfg.chunk)

    def summarize(self, kf_tokens: torch.Tensor, kf_valid: torch.Tensor) -> torch.Tensor:
        """PACKED frozen patch tokens (n_valid, P, Dt), the real keyframes in the row-major
        order of `kf_valid` -> (B, K, D) entries, zeros at padded slots."""
        if self.cfg.zero_content:
            return kf_valid.new_zeros(*kf_valid.shape, self.cfg.model_dim, dtype=torch.float32)
        return scatter_packed(self.entry_proj(self.summarizer(kf_tokens)), kf_valid)

    def _ranks(self, valid: torch.Tensor) -> torch.Tensor:
        """Temporal positions (B, K'): keyframes at their rank, the goal (last) at n_valid."""
        B, K = valid.shape
        t = torch.arange(K, device=valid.device, dtype=torch.float32).repeat(B, 1)
        t[:, -1] = valid[:, :-1].sum(1).float()
        return t

    def _entry_positions(self, pose: torch.Tensor, valid: torch.Tensor):
        """What the map RoPE reads -> (positions, yaw_free)."""
        if self.temporal:
            return self._ranks(valid), None
        if not self.cfg.goal_entry:
            return pose, None
        yaw_free = torch.zeros_like(valid)
        yaw_free[:, -1] = True
        return pose, yaw_free

    def encode_map(self, entries: torch.Tensor, kf_pose: torch.Tensor, kf_valid: torch.Tensor,
                   goal_position: torch.Tensor):
        """Entries (B, K, D) at kf_pose (B, K, 3) -> the mixed map, the goal entry appended when
        `goal_entry`: (tokens (B, K', D), poses (B, K', 3), valid (B, K'))."""
        B = entries.shape[0]
        if self.pose_as_content:
            entries = entries + self.entry_pose_content(pose_content(kf_pose))
        if self.cfg.goal_entry:
            goal = self.goal_token.expand(B, 1, -1)
            if self.pose_as_content:
                goal = goal + self.goal_position_content(goal_position / _CONTENT_METRES).unsqueeze(1)
            goal_pose = F.pad(goal_position, (0, 1)).unsqueeze(1)
            entries = torch.cat([entries, goal], 1)
            kf_pose = torch.cat([kf_pose, goal_pose], 1)
            kf_valid = torch.cat([kf_valid, kf_valid.new_ones(B, 1)], 1)
        K = entries.shape[1]
        # padded rows attend to themselves: no all-masked row
        mask = kf_valid.view(B, 1, 1, K) | torch.eye(K, dtype=torch.bool, device=entries.device)
        positions, yaw_free = self._entry_positions(kf_pose, kf_valid)
        for block in self.map_blocks:
            entries = block(entries, positions, mask, yaw_free)
        return self.map_out_norm(entries), kf_pose, kf_valid

    def read(self, memory, query_pose: torch.Tensor, goal_position: torch.Tensor,
             teacher_actions: Optional[torch.Tensor] = None) -> torch.Tensor:
        """Queries at query_pose (B, T, 3) toward goal_position (B, 2) read the `encode_map`
        memory -> logits (B, T, chunk, A)."""
        tokens, pose, valid = memory
        B, T = query_pose.shape[:2]
        N = tokens.shape[1]
        x = self.query_token + self.query_goal_proj(goal_features(query_pose, goal_position[:, None]))
        if self.pose_as_content:
            x = x + self.query_pose_content(pose_content(query_pose))
        entry_positions, yaw_free = self._entry_positions(pose, valid)
        query_positions = entry_positions[:, -1:].expand(B, T) if self.temporal else query_pose
        mask = valid.view(B, 1, 1, N).expand(B, 1, T, N)
        for block in self.read_blocks:
            x = block(x, query_positions, mask, yaw_free, tokens, entry_positions)
        return self.decoder(self.read_norm(x), teacher_actions)

    def forward(self, kf_tokens, kf_pose, kf_valid, query_pose, goal_position,
                teacher_actions: Optional[torch.Tensor] = None) -> torch.Tensor:
        """kf_tokens PACKED (n_valid, P, Dt); kf_pose (B, K, 3); kf_valid (B, K) bool;
        query_pose (B, T, 3); goal_position (B, 2) -> logits (B, T, chunk, A)."""
        memory = self.encode_map(self.summarize(kf_tokens, kf_valid), kf_pose, kf_valid,
                                 goal_position)
        return self.read(memory, query_pose, goal_position, teacher_actions)


def _linear_norm(d_in: int, d: int) -> nn.Sequential:
    return nn.Sequential(OrderedDict(linear=nn.Linear(d_in, d), norm=nn.LayerNorm(d)))
