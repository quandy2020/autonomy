#!/usr/bin/env python3
"""Learn SanD depth-condition tokens while keeping the actual DeFM frozen.

This replaces the upstream depth encoder, not its spline implementation, expert
labels, goal/turn fusion or diffusion loss. Callers must supply a strictly loaded
official backbone and its official metric preprocessing function. Untrained
projection weights are not a deployment model or a navigation safety verdict.
"""

import torch
from torch import nn
from torch.nn import functional as F


class FrozenDefmSandEncoder(nn.Module):
    """Encode metric B,T,1,H,W input as real frozen DeFM plus learned conditions."""

    def __init__(self, backbone, preprocess, feature_dim=256, history=4,
                 patch_grid=(16, 16), condition_grid=(8, 12), token_dim=384):
        super().__init__()
        if (not isinstance(backbone, nn.Module) or not callable(preprocess) or
                not 1 <= history <= 32 or not 1 <= feature_dim <= 1024 or
                not 1 <= token_dim <= 2048 or any(not 1 <= v <= 32 for v in patch_grid) or
                any(not 1 <= v <= 32 for v in condition_grid)):
            raise ValueError('invalid frozen backbone, preprocessing or condition dimensions')
        self.backbone = backbone.eval().requires_grad_(False)
        self.preprocess = preprocess
        self.history = history
        self.patch_grid = tuple(patch_grid)
        self.condition_grid = tuple(condition_grid)
        self.token_dim = token_dim
        self.feature_dim = feature_dim
        self.projection = nn.Sequential(nn.Linear(token_dim, feature_dim),
                                        nn.LayerNorm(feature_dim), nn.GELU())
        self.spatial_position = nn.Parameter(torch.zeros(1, condition_grid[0] * condition_grid[1], feature_dim))
        self.time_embed = nn.Embedding(history, feature_dim)
        nn.init.normal_(self.spatial_position, std=.02)
        nn.init.normal_(self.time_embed.weight, std=.02)

    def train(self, mode=True):
        """Train the projection/fusion inputs without changing backbone dropout or statistics."""
        super().train(mode)
        self.backbone.eval()
        return self

    def forward(self, depth_sequence):
        """Use official metric preprocessing; never reconstruct raw depth from normalized pixels."""
        if (depth_sequence.ndim != 5 or depth_sequence.shape[1] != self.history or
                depth_sequence.shape[2] != 1 or not depth_sequence.is_floating_point()):
            raise ValueError('expected metric depth with shape B,history,1,H,W')
        if any(parameter.requires_grad for parameter in self.backbone.parameters()):
            raise RuntimeError('external code unfroze the shared DeFM backbone')
        batch, history, _, height, width = depth_sequence.shape
        if batch < 1 or height < 1 or width < 1 or height * width > 1920 * 1080:
            raise ValueError('invalid or excessive camera raster')
        with torch.no_grad():
            prepared = self.preprocess(depth_sequence.reshape(batch * history, 1, height, width),
                target_size=(self.patch_grid[0] * 14, self.patch_grid[1] * 14),
                patch_size=14, device=depth_sequence.device)
            features = self.backbone.forward_features(prepared)
            tokens = features['x_norm_patchtokens']
            expected = (batch * history, self.patch_grid[0] * self.patch_grid[1], self.token_dim)
            if tuple(tokens.shape) != expected or not torch.isfinite(tokens).all():
                raise ValueError('actual DeFM token shape/values do not match the declared contract')
        return self.project_tokens(tokens.reshape(batch, history, self.patch_grid[0] * self.patch_grid[1], self.token_dim))

    def project_tokens(self, tokens):
        """Consume the same real per-frame DeFM tokens already shared by perception consumers."""
        if (tokens.ndim != 4 or tuple(tokens.shape[1:]) !=
                (self.history, self.patch_grid[0] * self.patch_grid[1], self.token_dim) or
                tokens.shape[0] < 1 or not tokens.is_floating_point() or not torch.isfinite(tokens).all()):
            raise ValueError('expected finite real DeFM tokens with shape B,history,N,D')
        batch, history = tokens.shape[:2]
        tokens = tokens.detach()
        spatial = tokens.reshape(batch * history, -1, self.token_dim).transpose(1, 2).reshape(
            batch * history, self.token_dim, *self.patch_grid)
        pooled = self.pool_spatial(spatial).flatten(2).transpose(1, 2)
        # Pooling preserves spatial order; the trainable projection changes the
        # representation. Existing ResNet condition weights are not relabeled.
        projected = self.projection(pooled) + self.spatial_position
        projected = projected.reshape(batch, history, -1, self.feature_dim)
        time = self.time_embed(torch.arange(history, device=tokens.device))
        projected = projected + time[None, :, None, :]
        return projected.flatten(1, 2), None

    def pool_spatial(self, spatial):
        """Original training pooling operation; export may decompose the fixed integer bins."""
        return F.adaptive_avg_pool2d(spatial, self.condition_grid)


def fixed_adaptive_average(spatial, output_size):
    """Exact fixed-size AdaptiveAvgPool2d bins expressed as exportable slice means.

    PyTorch's legacy ONNX exporter cannot express a 16-to-12 adaptive pooling
    ratio directly. Each output bin is the mean over floor(i*input/output) to
    ceil((i+1)*input/output), preserving the original trained operator's overlap.
    This function changes neither the grid nor the learned feature projection.
    """
    height, width = spatial.shape[-2:]
    out_height, out_width = output_size
    rows = []
    for row in range(out_height):
        begin_y = row*height//out_height
        end_y = ((row+1)*height+out_height-1)//out_height
        columns = []
        for col in range(out_width):
            begin_x = col*width//out_width
            end_x = ((col+1)*width+out_width-1)//out_width
            columns.append(spatial[..., begin_y:end_y, begin_x:end_x].mean(dim=(-2, -1)))
        rows.append(torch.stack(columns, dim=-1))
    return torch.stack(rows, dim=-2)


def validate_real_expert(sample):
    """Reject upstream dummy/unnormalized examples before they can enter optimization."""
    from pathlib import Path
    # Upstream original_run_id is a short run name, not an absolute directory.
    # The dataset wrapper supplies its actual canonical source path separately.
    run = sample.get('source_run_path', sample.get('original_run_id'))
    if (not sample.get('bspline_success') or sample.get('run_id') in (None, '', 'dummy') or
            not run or not Path(run).is_dir() or
            'control_points_norm' not in sample or 'end_relative_pose_norm' not in sample):
        raise RuntimeError('missing actual expert identity, successful fit or normalized targets')
    expected = {'depth_sequence': (4, 1, 480, 640), 'control_points_norm': (8, 3),
                'end_relative_pose_norm': (3,)}
    for field, shape in expected.items():
        value = sample.get(field)
        if not isinstance(value, torch.Tensor) or tuple(value.shape) != shape or not torch.isfinite(value).all():
            raise RuntimeError('invalid actual training input/target: ' + field)
    if any(torch.any(torch.abs(sample[field]) > 1 + 1e-6)
           for field in ('control_points_norm', 'end_relative_pose_norm')):
        raise RuntimeError('actual expert normalized targets exceed their declared range')
    if not torch.all(torch.any(sample['depth_sequence'] > 0, dim=(1, 2, 3))):
        raise RuntimeError('an actual depth history frame has no measurement')


class FrozenOnnxDefmSandEncoder(FrozenDefmSandEncoder):
    """Train against the exact frozen graph consumed by the native perception loop.

    The caller validates graph/source/checkpoint provenance before constructing
    the session. The strictly loaded Torch backbone remains frozen and is not
    executed here, avoiding a different numerical feature path in training.
    Export the learned token projection/head separately, not this Python session.
    """

    def __init__(self, backbone, preprocess, session, cache_directory=None, **kwargs):
        super().__init__(backbone, preprocess, **kwargs)
        inputs = session.get_inputs()
        outputs = {value.name: value for value in session.get_outputs()}
        if (len(inputs) != 1 or inputs[0].name != 'depth_metric' or len(inputs[0].shape) != 4 or
                inputs[0].shape[:2] != [1, 1] or inputs[0].type != 'tensor(float)' or
                'patch_tokens' not in outputs or outputs['patch_tokens'].shape !=
                [1, self.patch_grid[0] * self.patch_grid[1], self.token_dim]):
            raise ValueError('frozen ONNX graph does not match the verified native DeFM contract')
        self.session = session
        self.raster = tuple(inputs[0].shape[-2:])
        self.cache_directory = cache_directory
        if cache_directory is not None:
            from pathlib import Path
            self.cache_directory = Path(cache_directory)
            self.cache_directory.mkdir(parents=True, exist_ok=True)

    def _tokens(self, frame):
        """Cache only actual immutable graph outputs keyed by complete metric input bytes."""
        import hashlib
        import numpy as np
        import os
        import tempfile
        key = hashlib.sha256(frame.tobytes()).hexdigest()
        path = self.cache_directory / (key + '.npy') if self.cache_directory is not None else None
        if path is not None and path.is_file():
            tokens = np.load(path, allow_pickle=False)
        else:
            tokens = self.session.run(['patch_tokens'], {'depth_metric': frame})[0]
            if (tokens.shape != (1, self.patch_grid[0] * self.patch_grid[1], self.token_dim) or
                    tokens.dtype != np.float32 or not np.isfinite(tokens).all()):
                raise ValueError('actual frozen ONNX returned invalid DeFM tokens')
            if path is not None:
                fd, temporary = tempfile.mkstemp(prefix='.defm-tokens-', dir=self.cache_directory)
                try:
                    with os.fdopen(fd, 'wb') as stream:
                        np.save(stream, tokens, allow_pickle=False)
                    os.replace(temporary, path)
                finally:
                    if os.path.exists(temporary): os.unlink(temporary)
        if (tokens.shape != (1, self.patch_grid[0] * self.patch_grid[1], self.token_dim) or
                tokens.dtype != np.float32 or not np.isfinite(tokens).all()):
            raise ValueError('cached tokens do not match actual frozen DeFM output')
        return tokens

    def forward(self, depth_sequence):
        """Feed original metric pixels to the same graph, with no second preprocessing pipeline."""
        import numpy as np
        if (depth_sequence.ndim != 5 or depth_sequence.shape[1] != self.history or
                depth_sequence.shape[2] != 1 or tuple(depth_sequence.shape[-2:]) != self.raster or
                depth_sequence.shape[0] < 1 or not depth_sequence.is_floating_point()):
            raise ValueError('actual camera history differs from the fixed native DeFM raster')
        if any(parameter.requires_grad for parameter in self.backbone.parameters()):
            raise RuntimeError('external code unfroze the shared DeFM backbone')
        batch = depth_sequence.shape[0]
        frames = depth_sequence.detach().to(device='cpu', dtype=torch.float32).numpy().reshape(-1, 1, *self.raster)
        tokens = np.concatenate([self._tokens(np.ascontiguousarray(frame[None])) for frame in frames])
        shared = torch.from_numpy(tokens).to(device=depth_sequence.device)
        return self.project_tokens(shared.reshape(batch, self.history, self.patch_grid[0] * self.patch_grid[1], self.token_dim))
