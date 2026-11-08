"""Coverage keyframe gate: a frame is kept iff at least eta of its valid depth patches are unseen.

Each valid depth patch (one per DeFM token) samples surface over a footprint that grows with
depth, d tan(HFOV / GRID), about 0.12 m per metre. Its cell is taken at the smallest of
`levels` not below that footprint, with the world bearing of its ray in `bearing_bins`. A
patch is seen when its cell or a planar neighbour, at its own level or the next coarser one,
is claimed within +-`bearing_tol` bearing bins. A kept frame claims its patches' cells at
their own and every coarser level, except patches `claim_range_m` or farther from the
camera, which count toward novelty but never claim.

Cells take a frame's depth and its recorded camera-to-world; heights are absolute.
"""
from __future__ import annotations

import dataclasses
import json
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from thirdparty.pan.util.constants import DEFM_GRID, SENSOR_HFOV_DEG, SENSOR_RES
from thirdparty.pan.util.geometry import CX, CY, DEPTH_MIN, FX, FY

GRID = DEFM_GRID                    # depth patches per frame side, one per DeFM token; eta counts them
assert SENSOR_RES[0] % GRID == 0 and SENSOR_RES[1] % GRID == 0, (
    f"SENSOR_RES {SENSOR_RES} must tile the {GRID}x{GRID} patch grid")
CELL_H, CELL_W = SENSOR_RES[0] // GRID, SENSOR_RES[1] // GRID   # native px per cell
N_CELLS = GRID * GRID
TAN_PATCH = float(np.tan(np.radians(SENSOR_HFOV_DEG / GRID)))  # patch footprint per metre of depth


def _ray_table():
    """Per-cell (a, b): the ray (a, b, -1) at unit depth through each cell centre of the native
    frame, with the sensor's own intrinsics. Row-major, matching the ViT's token order."""
    j, i = np.meshgrid(np.arange(GRID), np.arange(GRID))
    u = CELL_W * j.ravel() + (CELL_W - 1) / 2.0
    v = CELL_H * i.ravel() + (CELL_H - 1) / 2.0
    return (u - CX) / FX, -(v - CY) / FY


A_RAY, B_RAY = _ray_table()


def patch_depth(depth: np.ndarray):
    """(H, W) metric depth -> (N_CELLS,) median over each cell's valid pixels, and
    (N_CELLS,) valid fraction. All-invalid cells report depth 0."""
    d = np.asarray(depth, np.float32)
    assert d.shape == tuple(SENSOR_RES), (
        f"coverage cells tile the sensor's own grid: expected {tuple(SENSOR_RES)}, "
        f"got {d.shape}")
    b = d.reshape(GRID, CELL_H, GRID, CELL_W).transpose(0, 2, 1, 3).reshape(N_CELLS, -1)
    valid = b > DEPTH_MIN
    frac = valid.mean(1).astype(np.float32)
    med = np.zeros(N_CELLS, np.float32)
    any_valid = valid.any(1)
    if any_valid.any():
        with np.errstate(invalid="ignore"):
            med[any_valid] = np.nanmedian(
                np.where(valid[any_valid], b[any_valid], np.nan), axis=1)
    return med, frac


def world_points(pdepth: np.ndarray, c2w: np.ndarray) -> np.ndarray:
    """(N_CELLS,) patch depth + (4, 4) camera-to-world -> (N_CELLS, 3) world points."""
    d = np.asarray(pdepth, dtype=np.float64)
    pcam = np.stack([A_RAY * d, B_RAY * d, -d], -1)
    c2w = np.asarray(c2w, dtype=np.float64)
    return pcam @ c2w[:3, :3].T + c2w[:3, 3]


# Cell key layout, high to low bits: level * _BEARING_SLOTS + bearing | ix | iz | iy. Cell
# indices are offset to be non-negative and clipped to their field.
_XZ_BITS, _Y_BITS, _BEARING_SLOTS = 14, 10, 16


@dataclass(frozen=True)
class CoverageSpec:
    levels: Tuple[float, ...] = (0.25, 0.5, 1.0, 2.0)   # cell sizes (m), fine to coarse
    bearing_bins: int = 8
    bearing_tol: int = 1                  # +- bins that count as the same aspect
    max_range_m: float = 10.0             # patch depth bound: beyond it a patch neither counts nor claims
    min_valid_frac: float = 0.5           # a mostly no-hit patch neither counts nor claims
    claim_range_m: Optional[float] = 6.0  # camera-to-point range bound for claiming; None = no bound

    def __post_init__(self):
        assert self.bearing_bins <= _BEARING_SLOTS, f"at most {_BEARING_SLOTS} bearing bins"

    def to_json(self) -> str:
        return json.dumps(dataclasses.asdict(self))


COVERAGE = CoverageSpec()                 # the rule every stored keep column is written with


def _cell_keys(level, ix, iy, iz, bearing) -> np.ndarray:
    xz_max, y_max = (1 << _XZ_BITS) - 1, (1 << _Y_BITS) - 1
    ix = np.clip(ix + (1 << (_XZ_BITS - 1)), 0, xz_max)
    iz = np.clip(iz + (1 << (_XZ_BITS - 1)), 0, xz_max)
    iy = np.clip(iy + (1 << (_Y_BITS - 1)), 0, y_max)
    return (((level * _BEARING_SLOTS + bearing) << (2 * _XZ_BITS + _Y_BITS))
            | (ix << (_XZ_BITS + _Y_BITS)) | (iz << _Y_BITS) | iy)


def frame_cells(depth: np.ndarray, c2w: np.ndarray, spec: CoverageSpec = COVERAGE):
    """One frame's coverage cells, from its depth and camera-to-world -> (lookup (P, M) keys
    through which each valid patch counts as seen, -1 = none; claim (Q,) keys the frame
    claims if kept)."""
    pd, pf = patch_depth(depth)
    ok = (pd > DEPTH_MIN) & (pd < spec.max_range_m) & (pf >= spec.min_valid_frac)
    if not ok.any():
        return np.zeros((0, 0), np.int64), np.zeros(0, np.int64)
    c2w = np.asarray(c2w, np.float64)
    points = world_points(pd, c2w)[ok]
    ray = points - c2w[:3, 3]
    n_bins = spec.bearing_bins
    width = 2 * np.pi / n_bins
    bearing = np.floor((np.arctan2(-ray[:, 0], -ray[:, 2]) + width / 2) / width).astype(np.int64) % n_bins
    sizes = np.asarray(spec.levels, np.float64)
    top = len(sizes) - 1
    level = np.searchsorted(sizes, pd[ok].astype(np.float64) * TAN_PATCH).clip(0, top)
    cell = np.floor(points[None] / sizes[:, None, None]).astype(np.int64)       # (levels, P, 3)
    offsets = np.array([(dx, dz, db) for dx in (-1, 0, 1) for dz in (-1, 0, 1)
                        for db in range(-spec.bearing_tol, spec.bearing_tol + 1)], np.int64)
    patches = np.arange(len(points))

    def neighbourhood(lv: np.ndarray) -> np.ndarray:
        c = cell[lv, patches]
        return _cell_keys(lv[:, None], c[:, 0:1] + offsets[:, 0], c[:, 1:2],
                          c[:, 2:3] + offsets[:, 1], (bearing[:, None] + offsets[:, 2]) % n_bins)

    coarser = neighbourhood(np.minimum(level + 1, top))
    coarser[level == top] = -1
    claims = (np.ones(len(points), bool) if spec.claim_range_m is None
              else np.linalg.norm(ray, axis=1) < spec.claim_range_m)
    claim = []
    for lv in range(top + 1):
        m = claims & (level <= lv)
        c = cell[lv, m]
        claim.append(_cell_keys(lv, c[:, 0], c[:, 1], c[:, 2], bearing[m]))
    return np.concatenate([neighbourhood(level), coarser], 1), np.unique(np.concatenate(claim))


class CoverageGate:
    """The cells claimed by one map's kept frames, on top of the python-int keys `claimed`
    given at construction (e.g. an inherited map's)."""

    def __init__(self, eta: int, claimed=()):
        self.eta = eta
        self.claimed = set(claimed)

    def novel(self, frame) -> np.ndarray:
        """(P,) whether each valid patch of the frame (`frame_cells`) is unseen."""
        lookup = frame[0]
        claimed = self.claimed
        seen = np.fromiter((k in claimed for k in lookup.ravel().tolist()), bool, lookup.size)
        return ~seen.reshape(lookup.shape).any(1)

    def offer(self, frame, force: bool = False) -> bool:
        """Keep, and claim, the frame iff at least eta of its patches are unseen; `force`
        keeps unconditionally (a map's first frame)."""
        keep = force or int(self.novel(frame).sum()) >= self.eta
        if keep:
            self.claimed.update(frame[1].tolist())
        return keep
