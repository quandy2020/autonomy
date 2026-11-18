#!/usr/bin/env python3
"""Compare recorded automsgs OccupancyGrid messages outside the navigation loop.

Ground truth is read only by this offline evaluator. Unknown algorithm cells
remain unknown, including cells outside its map extent. Maps must already have
the same frame, resolution and grid alignment; this tool never fits a transform
to improve a score. Reported coverage does not certify exploration completion,
sensor provenance, controller safety, or physical collision avoidance.
"""

import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path

import numpy as np


MAXIMUM_CELLS = 2000000


def validate_grid(message, require_axis_aligned=True):
    """Check the native message before reshaping or interpreting occupancy."""
    if message.DESCRIPTOR.full_name != 'automsgs.msgs.map_msgs.OccupancyGrid':
        raise ValueError('expected the existing automsgs OccupancyGrid')
    info = message.info
    width, height = int(info.width), int(info.height)
    if width <= 0 or height <= 0 or width * height > MAXIMUM_CELLS:
        raise ValueError('invalid grid dimensions or cell budget')
    if len(message.data) != width * height:
        raise ValueError('OccupancyGrid data length differs from width*height')
    if not message.header.frame_id:
        raise ValueError('map frame_id is required')
    if not math.isfinite(info.resolution) or info.resolution <= 0:
        raise ValueError('map resolution must be finite and positive')
    origin = info.origin
    if not all(math.isfinite(v) for v in
               (origin.position.x, origin.position.y, origin.position.z)):
        raise ValueError('map origin must be finite')
    q = origin.orientation
    quaternion = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in quaternion) or \
            abs(sum(v * v for v in quaternion) - 1) > 1e-8:
        raise ValueError('expected a finite normalized grid quaternion')
    if require_axis_aligned and (max(abs(q.x), abs(q.y), abs(q.z)) > 1e-8 or
                                 abs(abs(q.w) - 1) > 1e-8):
        raise ValueError('only explicit axis-aligned grid origins are supported')
    values = np.asarray(message.data, dtype=np.int32).reshape(height, width)
    if np.any((values < -1) | (values > 100)):
        raise ValueError('occupancy values must be -1 or in [0,100]')
    return values


def align_algorithm(truth, algorithm, algorithm_values):
    """Copy only aligned overlapping cells; retain unknown outside the source."""
    if truth.header.frame_id != algorithm.header.frame_id:
        raise ValueError('map frames differ; an explicit upstream alignment is required')
    scale = float(truth.info.resolution)
    if not math.isclose(scale, algorithm.info.resolution, rel_tol=1e-7, abs_tol=1e-9):
        raise ValueError('map resolutions differ; resample explicitly before evaluation')
    a, b = truth.info.origin.position, algorithm.info.origin.position
    if not math.isclose(a.z, b.z, rel_tol=0, abs_tol=1e-6):
        raise ValueError('grid ground-plane origins differ')
    offsets = [(b.x - a.x) / scale, (b.y - a.y) / scale]
    dx, dy = [round(value) for value in offsets]
    if any(abs(value - rounded) > 1e-4 for value, rounded in zip(offsets, (dx, dy))):
        raise ValueError('map origins are not aligned on the same cell lattice')
    output = np.full((truth.info.height, truth.info.width), -1, dtype=np.int32)
    tx, ty, ax, ay = max(dx, 0), max(dy, 0), max(-dx, 0), max(-dy, 0)
    width = min(truth.info.width - tx, algorithm.info.width - ax)
    height = min(truth.info.height - ty, algorithm.info.height - ay)
    if width > 0 and height > 0:
        output[ty:ty + height, tx:tx + width] = \
            algorithm_values[ay:ay + height, ax:ax + width]
    return output, {'offset_cells': [dx, dy],
                    'overlap_cells': max(0, width) * max(0, height)}


def footprint_free(free, radius_cells):
    """Require a circular robot footprint clear of closed blocked-cell squares.

Unknown truth cells and the outside of the finite truth grid are blocked. For
each relative cell, its square intersects a footprint when the distance from
the queried center to that square is no greater than the configured radius.
This is a conservative static 2D domain model, not a 3D reachability guarantee.
"""
    if not math.isfinite(radius_cells) or not 0 <= radius_cells <= 100:
        raise ValueError('robot radius must span between zero and 100 grid cells')
    if radius_cells == 0:
        return free.copy()
    span = math.ceil(radius_cells + 0.5)
    if free.size * (2 * span + 1) ** 2 > 512000000:
        raise ValueError('footprint evaluation exceeds the bounded cell-offset budget')
    padded = np.pad(~free, span, constant_values=True)
    blocked = np.zeros_like(free)
    height, width = free.shape
    for dy in range(-span, span + 1):
        for dx in range(-span, span + 1):
            separation = math.hypot(max(abs(dx) - 0.5, 0), max(abs(dy) - 0.5, 0))
            if separation <= radius_cells:
                blocked |= padded[span + dy:span + dy + height,
                                  span + dx:span + dx + width]
    return free & ~blocked


def connected_component(mask, start):
    """Find the start-connected domain with eight neighbors and no corner cuts."""
    height, width = mask.shape
    sx, sy = start
    if not (0 <= sx < width and 0 <= sy < height) or not mask[sy, sx]:
        raise ValueError('start pose is outside footprint-clear truth free space')
    reached = np.zeros_like(mask)
    reached[sy, sx] = True
    pending = deque([(sx, sy)])
    while pending:
        x, y = pending.popleft()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if not (0 <= nx < width and 0 <= ny < height) or \
                        not mask[ny, nx] or reached[ny, nx]:
                    continue
                if dx and dy and (not mask[y, nx] or not mask[ny, x]):
                    continue
                reached[ny, nx] = True
                pending.append((nx, ny))
    return reached


def residual_components(mask):
    """Count four-connected residuals and retain their ten largest cell sets."""
    height, width = mask.shape
    visited = np.zeros_like(mask)
    largest = []
    count = 0
    for index in np.flatnonzero(mask):
        y, x = divmod(int(index), width)
        if visited[y, x]:
            continue
        visited[y, x] = True
        pending = deque([(x, y)])
        size = 0
        while pending:
            cx, cy = pending.popleft()
            size += 1
            for nx, ny in ((cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1)):
                if 0 <= nx < width and 0 <= ny < height and mask[ny, nx] and not visited[ny, nx]:
                    visited[ny, nx] = True
                    pending.append((nx, ny))
        count += 1
        largest.append({'cells': size, 'representative_cell': [x, y]})
        largest.sort(key=lambda item: item['cells'], reverse=True)
        del largest[10:]
    return {'count': count, 'largest': largest, 'connectivity': 4}


def compare_grids(truth, algorithm, start, robot_radius, free_max=20, occupied_min=65):
    """Measure coverage and classification errors on a fixed truth domain."""
    truth_values = validate_grid(truth)
    algorithm_values = validate_grid(algorithm)
    if not isinstance(free_max, int) or not isinstance(occupied_min, int) or \
            not 0 <= free_max < occupied_min <= 100:
        raise ValueError('thresholds must satisfy 0 <= free_max < occupied_min <= 100')
    if start.DESCRIPTOR.full_name != 'automsgs.msgs.geometry_msgs.PoseStamped' or \
            start.header.frame_id != truth.header.frame_id:
        raise ValueError('start must be an existing PoseStamped in the truth frame')
    if not math.isfinite(robot_radius) or robot_radius < 0:
        raise ValueError('robot radius must be finite and nonnegative')
    position = start.pose.position
    if not math.isfinite(position.x) or not math.isfinite(position.y):
        raise ValueError('start position must be finite')
    aligned, alignment = align_algorithm(truth, algorithm, algorithm_values)
    truth_free = (truth_values >= 0) & (truth_values <= free_max)
    truth_occupied = truth_values >= occupied_min
    algorithm_free = (aligned >= 0) & (aligned <= free_max)
    algorithm_occupied = aligned >= occupied_min
    algorithm_undecided = ~(algorithm_free | algorithm_occupied)
    scale = float(truth.info.resolution)
    origin = truth.info.origin.position
    start_cell = (math.floor((position.x - origin.x) / scale),
                  math.floor((position.y - origin.y) / scale))
    traversable = footprint_free(truth_free, robot_radius / scale)
    reachable = connected_component(traversable, start_cell)
    denominator = int(np.count_nonzero(reachable))
    unknown_count = int(np.count_nonzero(reachable & (aligned < 0)))
    uncertain_count = int(np.count_nonzero(reachable & (aligned >= 0) & algorithm_undecided))
    known_count = denominator - unknown_count
    correct_free_count = int(np.count_nonzero(reachable & algorithm_free))
    false_occupied_count = int(np.count_nonzero(reachable & algorithm_occupied))
    confusion = {}
    for name, actual in (('free', truth_free), ('occupied', truth_occupied)):
        confusion[name] = {
            predicted: int(np.count_nonzero(actual & mask)) for predicted, mask in
            (('free', algorithm_free), ('occupied', algorithm_occupied),
             ('unknown_or_uncertain', algorithm_undecided))}
    return {
        'coverage_comparison_performed': True, 'exploration_accepted': False,
        'scope': 'Offline map metrics only; independent run and map provenance are still required.',
        'frame_id': truth.header.frame_id, 'resolution': scale,
        'truth_grid_dimensions': [truth.info.width, truth.info.height],
        'alignment': alignment, 'start_cell': list(start_cell),
        'robot_radius_m': robot_radius,
        'thresholds': {'free_max': free_max, 'occupied_min': occupied_min},
        'domain': 'Start-connected static 2D truth free cells after circular footprint erosion.',
        'reachable_truth_cells': denominator, 'reachable_truth_area_m2': denominator * scale * scale,
        'reachable_unknown_cells': unknown_count, 'reachable_uncertain_cells': uncertain_count,
        'known_coverage_fraction': known_count / denominator,
        'correct_free_coverage_fraction': correct_free_count / denominator,
        'reachable_false_occupied_fraction': false_occupied_count / denominator,
        'global_confusion_cells': confusion,
        'residual_unknown_regions': residual_components(reachable & (aligned < 0)),
        'residual_not_confirmed_free_regions': residual_components(reachable & ~algorithm_free),
        'independent_map_provenance_verified': False,
    }


def load_message(path, message_type):
    """Read a bounded serialized native protobuf without fabricating a schema."""
    if path.stat().st_size > 32 * 1024 * 1024:
        raise ValueError('serialized input exceeds the 32 MiB budget')
    payload = path.read_bytes()
    message = message_type()
    message.ParseFromString(payload)
    return message, hashlib.sha256(payload).hexdigest()


def main():
    """Compare two independently recorded maps and save an immutable report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ground-truth', type=Path, required=True)
    parser.add_argument('--algorithm-map', type=Path, required=True)
    parser.add_argument('--start-pose', type=Path, required=True)
    parser.add_argument('--robot-radius', type=float, required=True)
    parser.add_argument('--free-max', type=int, default=20)
    parser.add_argument('--occupied-min', type=int, default=65)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    if args.report.exists():
        parser.error('report already exists')
    report = {'coverage_comparison_performed': False, 'exploration_accepted': False}
    try:
        from automsgs.msgs.map_msgs.occupancy_grid_pb2 import OccupancyGrid
        from automsgs.msgs.geometry_msgs.pose_stamped_pb2 import PoseStamped

        truth, truth_hash = load_message(args.ground_truth, OccupancyGrid)
        algorithm, algorithm_hash = load_message(args.algorithm_map, OccupancyGrid)
        start, start_hash = load_message(args.start_pose, PoseStamped)
        if args.ground_truth.resolve() == args.algorithm_map.resolve() or truth_hash == algorithm_hash:
            raise ValueError('truth and algorithm inputs are identical; independent recordings are required')
        report = compare_grids(truth, algorithm, start, args.robot_radius,
                               args.free_max, args.occupied_min)
        report['input_sha256'] = {'ground_truth': truth_hash, 'algorithm_map': algorithm_hash,
                                  'start_pose': start_hash}
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.report.open('x') as output:
        json.dump(report, output, indent=2, allow_nan=False)
        output.write('\n')
    print(json.dumps(report, allow_nan=False), flush=True)
    return 0 if report['coverage_comparison_performed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
