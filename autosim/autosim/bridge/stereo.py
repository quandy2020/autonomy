"""A calibrated two-camera head rig; independent of automotive surround rigs."""
from __future__ import annotations
import math
from typing import Mapping, Any


def stereo_cameras(sensors: Mapping[str, Any]) -> list[dict]:
    block = sensors.get('stereo') or {}
    if not isinstance(block, Mapping):
        raise ValueError('habitat.sensors.stereo must be a mapping')
    if not block.get('enabled', False):
        return []
    width, height = int(block.get('width', 640)), int(block.get('height', 480))
    fov, rate = float(block.get('hfov_deg', 60)), float(block.get('rate_hz', 15))
    baseline = float(block.get('baseline_m', 0.10))
    if width <= 0 or height <= 0 or not 0 < fov < 180 or not math.isfinite(fov):
        raise ValueError('invalid stereo resolution or field of view')
    if not math.isfinite(rate) or rate <= 0 or not math.isfinite(baseline) or baseline <= 0:
        raise ValueError('stereo rate and baseline must be finite and positive')
    cameras = []
    for side in ('left', 'right'):
        frame = str(block.get(f'{side}_frame') or f'camera_{side}_optical_frame')
        cameras.append(dict(side=side, uuid=f'stereo_{side}', frame=frame,
                            width=width, height=height, hfov_deg=fov, rate_hz=rate,
                            baseline_m=baseline,
                            rgb_key=f'stereo_{side}_rgb', info_key=f'stereo_{side}_info',
                            rgb_channel=str(block.get(f'{side}_channel') or f'/camera/{side}/image_raw'),
                            info_channel=str(block.get(f'{side}_info_channel') or f'/camera/{side}/camera_info')))
    return cameras
