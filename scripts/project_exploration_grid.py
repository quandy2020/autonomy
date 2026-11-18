#!/usr/bin/env python3
"""Project an independently recorded algorithm grid onto an evaluation lattice.

This offline tool requires an explicit native TransformStamped; it never fits
map data or publishes anything to navigation. The target map supplies lattice
geometry only. Unknown source cells and space outside the source remain unknown.
An optical XZ grid can be mapped to XY using its original origin orientation.
Projection between parallel planes at different heights needs explicit opt-in
and remains a 2D evaluation convention, not measured ground or 3D clearance.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from compare_exploration_maps import load_message, validate_grid


MAXIMUM_WINDOW_CELLS = 256
MAXIMUM_CELL_VISITS = 50000000
CHUNK_CELLS = 16384


def rotation_matrix(quaternion):
    """Require a normalized native quaternion before constructing its rotation."""
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    if not all(math.isfinite(v) for v in (x, y, z, w)) or \
            abs(x*x + y*y + z*z + w*w - 1) > 1e-8:
        raise ValueError('transform needs a finite normalized quaternion')
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def project_grid(source, target, transform, allow_parallel_plane_projection=False):
    """Use conservative source-cell bounds for each complete target cell.

    Every source cell in the inverse-projected target square's bounding box must
    be known. Values use the largest probability in that box, so obstacle and
    uncertain evidence take precedence over free. Bounding boxes may include
    extra cells for a rotated lattice; this intentionally reduces free coverage.
    No target occupancy values participate in output classification.
    """
    values = validate_grid(source, require_axis_aligned=False)
    validate_grid(target)
    if transform.DESCRIPTOR.full_name != 'automsgs.msgs.geometry_msgs.TransformStamped':
        raise ValueError('expected the existing native TransformStamped')
    if transform.header.frame_id != target.header.frame_id or \
            transform.child_frame_id != source.header.frame_id:
        raise ValueError('transform parent/child must match target/source map frames')
    stamp = transform.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
        raise ValueError('invalid transform timestamp')
    translation = transform.transform.translation
    offset = np.array([translation.x, translation.y, translation.z], dtype=float)
    if not np.isfinite(offset).all():
        raise ValueError('transform translation must be finite')
    world_rotation = rotation_matrix(transform.transform.rotation)
    source_rotation = rotation_matrix(source.info.origin.orientation)
    origin = source.info.origin.position
    source_offset = np.array([origin.x, origin.y, origin.z], dtype=float)
    rotation = world_rotation @ source_rotation
    position = world_rotation @ source_offset + offset
    if not np.isfinite(position).all():
        raise ValueError('composed grid origin overflowed')
    # A tilted plane cannot be flattened silently into a navigation grid.
    if max(abs(rotation[2, 0]), abs(rotation[2, 1])) > 1e-7:
        raise ValueError('source grid plane is not parallel to target XY plane')
    height_offset = float(position[2] - target.info.origin.position.z)
    if abs(height_offset) > 1e-6 and not allow_parallel_plane_projection:
        raise ValueError('parallel planes differ in height; explicit projection permission is required')
    inverse = rotation[:2, :2].T
    if not np.allclose(inverse @ rotation[:2, :2], np.eye(2), atol=1e-7, rtol=0):
        raise ValueError('projected transform is not a planar isometry')
    source_scale = float(source.info.resolution)
    target_scale = float(target.info.resolution)
    half = np.abs(inverse) @ np.full(2, target_scale / (2 * source_scale))
    if not np.isfinite(half).all() or (half > MAXIMUM_WINDOW_CELLS / 2).any():
        raise ValueError('resolution ratio exceeds bounded source-cell work')
    bound = np.ceil(2 * half).astype(np.int64) + 2
    count = int(target.info.width) * int(target.info.height)
    if int(bound[0]) * int(bound[1]) > MAXIMUM_WINDOW_CELLS or \
            count * int(bound[0]) * int(bound[1]) > MAXIMUM_CELL_VISITS:
        raise ValueError('projection exceeds bounded source-cell work')
    output = np.full(count, -1, dtype=np.int32)
    visits = 0
    width, height = values.shape[1], values.shape[0]
    target_origin = target.info.origin.position
    for begin in range(0, count, CHUNK_CELLS):
        end = min(begin + CHUNK_CELLS, count)
        indexes = np.arange(begin, end, dtype=np.int64)
        coordinates = np.column_stack([
            target_origin.x + (indexes % target.info.width + .5) * target_scale,
            target_origin.y + (indexes // target.info.width + .5) * target_scale])
        centers = (coordinates - position[:2]) @ inverse.T / source_scale
        if not np.isfinite(coordinates).all() or not np.isfinite(centers).all():
            raise ValueError('projected coordinates overflowed')
        # Maps with huge or remote origins need no conversion to overflowing ints.
        relevant = np.all((centers + half > 0) &
                          (centers - half < np.array([width, height])), axis=1)
        if not relevant.any():
            continue
        selected = np.flatnonzero(relevant)
        centers = centers[relevant]
        lower = np.floor(centers - half + 1e-8).astype(np.int64)
        upper = np.ceil(centers + half - 1e-8).astype(np.int64) - 1
        span = upper - lower + 1
        if (span <= 0).any():
            raise ValueError('target cell vanished at numerical precision')
        known = np.ones(len(selected), dtype=bool)
        maximum = np.zeros(len(selected), dtype=np.int32)
        for dy in range(int(span[:, 1].max())):
            for dx in range(int(span[:, 0].max())):
                active = (dx < span[:, 0]) & (dy < span[:, 1])
                visits += int(active.sum())
                if visits > MAXIMUM_CELL_VISITS:
                    raise ValueError('projection exhausted source-cell visit budget')
                x, y = lower[:, 0] + dx, lower[:, 1] + dy
                inside = (x >= 0) & (x < width) & (y >= 0) & (y < height)
                sample = np.full(len(selected), -1, dtype=np.int32)
                valid = active & inside
                sample[valid] = values[y[valid], x[valid]]
                known &= ~active | (sample >= 0)
                np.maximum(maximum, sample, out=maximum, where=active)
        output[begin + selected[known]] = maximum[known]
    result = type(source)()
    result.header.CopyFrom(source.header)
    result.header.frame_id = target.header.frame_id
    result.info.CopyFrom(target.info)
    result.info.map_load_time.CopyFrom(source.info.map_load_time)
    result.data.extend(output.tolist())
    details = {
        'projected': False,
        'method': 'All source cells in the inverse target-cell bounding box; unknown veto, maximum probability.',
        'target_values_used_for_classification': False,
        'transform_fitted_from_maps': False,
        'source_frame': source.header.frame_id, 'target_frame': target.header.frame_id,
        'source_resolution': source_scale, 'target_resolution': target_scale,
        'parallel_plane_height_offset_m': height_offset,
        'parallel_plane_projection_explicitly_enabled': allow_parallel_plane_projection,
        'source_cell_visits': visits,
        'source_known_cells': int((values >= 0).sum()),
        'projected_known_cells': int((output >= 0).sum()),
        'projected_unknown_cells': int((output < 0).sum()),
        'coordinate_epoch_verified': False,
        'exploration_accepted': False,
        'scope': 'Offline 2D lattice projection; not ground, fusion, coverage completion or controller evidence.',
    }
    return result, details


def main():
    """Retain original map/transform hashes and write a new native evaluation grid."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-map', type=Path, required=True)
    parser.add_argument('--target-grid', type=Path, required=True)
    parser.add_argument('--transform', type=Path, required=True)
    parser.add_argument('--allow-parallel-plane-projection', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    inputs = [args.source_map.resolve(), args.target_grid.resolve(), args.transform.resolve()]
    if args.output.resolve() in inputs or args.report.resolve() in inputs or \
            args.output.resolve() == args.report.resolve() or args.output.exists() or args.report.exists():
        parser.error('outputs must be new distinct files separate from original inputs')
    report = {'projected': False, 'exploration_accepted': False}
    try:
        from automsgs.msgs.map_msgs.occupancy_grid_pb2 import OccupancyGrid
        from automsgs.msgs.geometry_msgs.transform_stamped_pb2 import TransformStamped
        source, source_hash = load_message(args.source_map, OccupancyGrid)
        target, target_hash = load_message(args.target_grid, OccupancyGrid)
        transform, transform_hash = load_message(args.transform, TransformStamped)
        if args.source_map.resolve() == args.target_grid.resolve() or source_hash == target_hash:
            raise ValueError('algorithm and target recordings must be independent')
        result, report = project_grid(source, target, transform, args.allow_parallel_plane_projection)
        report['input_sha256'] = {'source_map': source_hash, 'target_grid': target_hash,
                                 'transform': transform_hash}
        payload = result.SerializeToString()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('xb') as stream:
            stream.write(payload)
        report['output_sha256'] = hashlib.sha256(payload).hexdigest()
        report['projected'] = True
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.report.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(report, allow_nan=False), flush=True)
    return 0 if report['projected'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
