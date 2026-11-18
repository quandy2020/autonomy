#!/usr/bin/env python3
"""Run the real RGB-D Atlas application beside an isolated Habitat instance.

The camera YAML is derived from live CameraInfo and the actual static sensor
installation. Ground truth is recorded by this evaluator only. This check sends
no motion and never treats SLAM output as autonomous-exploration acceptance.
Run with AutoSim's Python environment and generated automsgs on PYTHONPATH.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time

import numpy as np


def sensor_height(transforms, root, sensor):
    """Read camera height from the published static installation, not a nominal prior."""
    graph = {}
    for stamped in transforms.transforms:
        transform = stamped.transform
        q = transform.rotation
        xyzw = np.array([q.x, q.y, q.z, q.w], dtype=float)
        xyz = np.array([transform.translation.x, transform.translation.y,
                        transform.translation.z], dtype=float)
        if not np.isfinite(xyzw).all() or not np.isfinite(xyz).all() or abs(np.linalg.norm(xyzw) - 1) > 1e-5:
            raise ValueError('invalid static installation transform')
        x, y, z, w = xyzw
        matrix = np.eye(4)
        matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
        matrix[:3, 3] = xyz
        graph.setdefault(stamped.header.frame_id, []).append((stamped.child_frame_id, matrix))
    pending = [(root, np.eye(4))]
    seen = set()
    while pending and len(seen) < 256:
        frame, accumulated = pending.pop()
        if frame in seen:
            continue
        seen.add(frame)
        if frame == sensor:
            height = float(accumulated[2, 3])
            if not math.isfinite(height) or height <= 0:
                raise ValueError('camera must have a measured positive ground-relative height')
            return height
        pending.extend((child, accumulated @ transform) for child, transform in graph.get(frame, []))
    raise ValueError('static TF has no root-to-camera installation chain')


def camera_settings(info, rgb, depth, fps, height):
    """Generate a pinhole RGB-D configuration from actual registered metric images."""
    k = list(info.k)
    if (len(k) != 9 or not all(math.isfinite(v) for v in k) or k[0] <= 0 or k[4] <= 0 or
            any(abs(v) > 1e-10 for v in info.d) or info.binning_x > 1 or info.binning_y > 1 or
            info.roi.width or info.roi.height or
            (rgb.width, rgb.height) != (info.width, info.height) or
            (depth.width, depth.height) != (info.width, info.height) or
            rgb.header.frame_id != info.header.frame_id or depth.header.frame_id != info.header.frame_id or
            rgb.encoding != 'rgb8' or depth.encoding != '32FC1' or
            not 0 < fps <= 120 or not float(fps).is_integer()):
        raise ValueError('requires registered rectified RGB8 and metric 32FC1 with live pinhole calibration')
    return '\n'.join([
        '%YAML:1.0', 'File.version: "1.0"', 'Camera.type: "PinHole"',
        f'Camera1.fx: {float(k[0])!r}', f'Camera1.fy: {float(k[4])!r}',
        f'Camera1.cx: {float(k[2])!r}', f'Camera1.cy: {float(k[5])!r}',
        'Camera1.k1: 0.0', 'Camera1.k2: 0.0', 'Camera1.p1: 0.0',
        'Camera1.p2: 0.0', 'Camera1.k3: 0.0',
        f'Camera.width: {info.width}', f'Camera.height: {info.height}',
        f'Camera.fps: {int(fps)}', 'Camera.RGB: 1',
        # RGB-D has no stereo pair. This nominal ORB close-depth baseline is an
        # algorithm setting; image depth remains measured metres at scale one.
        'Stereo.b: 0.1', 'Stereo.ThDepth: 40.0', 'RGBD.DepthMapFactor: 1.0',
        'ORBextractor.nFeatures: 1200', 'ORBextractor.scaleFactor: 1.2',
        'ORBextractor.nLevels: 8', 'ORBextractor.iniThFAST: 20', 'ORBextractor.minThFAST: 7',
        # Atlas parses these required settings even when its viewer is disabled.
        'Viewer.KeyFrameSize: 0.05', 'Viewer.KeyFrameLineWidth: 1.0',
        'Viewer.GraphLineWidth: 0.9', 'Viewer.PointSize: 2.0',
        'Viewer.CameraSize: 0.08', 'Viewer.CameraLineWidth: 3.0',
        'Viewer.ViewpointX: 0.0', 'Viewer.ViewpointY: -0.7',
        'Viewer.ViewpointZ: -1.8', 'Viewer.ViewpointF: 500.0',
        f'Atlas.Map.SensorHeight: {float(height)!r}', 'Atlas.Map.EstimateGroundPlane: 1',
        'Atlas.Map.GridResolution: 0.05', ''])


def main():
    """Retain native application logs and sensor/map evidence with explicit scope."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--localization', type=Path, required=True)
    parser.add_argument('--vocabulary', type=Path, required=True)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--steps', type=int, default=2500)
    args = parser.parse_args()
    if not args.prefix.startswith('/validation/exploration/') or not 500 <= args.steps <= 10000:
        parser.error('use an isolated validation prefix and 500..10000 steps')
    if not args.localization.is_file() or not args.vocabulary.is_file():
        parser.error('actual application and vocabulary are required')
    args.output.mkdir(parents=True, exist_ok=False)
    from autosim.runtime.settings import Config
    from autosim.runtime.runner import Runner
    from autosim.runtime.instance_lock import InstanceLock
    from automsgs.msgs.nav_msgs.path_pb2 import Path as NativePath
    from automsgs.msgs.diagnostic_msgs.diagnostic_array_pb2 import DiagnosticArray

    config = Config.load(args.config)
    channels = config.channel_map()
    habitat = config.data['habitat']
    if (habitat['backend'] != 'habitat' or
            any(not name.startswith(args.prefix + '/') for name in channels.values()) or
            not habitat['sensors']['camera']['enabled'] or
            any(habitat['sensors'][sensor]['enabled'] for sensor in ('imu', 'lidar_2d', 'lidar_3d'))):
        parser.error('requires real Habitat, isolated channels and RGB-D-only sensing')
    report = {'passed': False, 'scope': 'Real RGB-D localization and mapping only; no navigation or exploration acceptance.',
              'exploration_task_started': False, 'coverage_comparison_performed': False,
              'config_sha256': hashlib.sha256(args.config.read_bytes()).hexdigest(),
              'application_sha256': hashlib.sha256(args.localization.read_bytes()).hexdigest(),
              'vocabulary_sha256': hashlib.sha256(args.vocabulary.read_bytes()).hexdigest(),
              'channels': channels, 'maximum_steps': args.steps}
    data_lock = threading.Lock()
    changed = threading.Event()
    stopping = threading.Event()
    counts, first, last = {}, {}, {}
    tracked_poses, healthy_status, matched_status = {}, {}, set()
    status_instances, status_maps = set(), set()
    depth_stamps, matched_depth = set(), set()
    unhealthy_status_count = 0
    process_owner = []
    readers = []
    start = time.monotonic()

    def receive(message, name):
        nonlocal unhealthy_status_count
        if message is None:
            return
        with data_lock:
            counts[name] = counts.get(name, 0) + 1
            if name not in first:
                first[name] = message
            last[name] = message
            stamp = (message.header.stamp.sec, message.header.stamp.nanosec) if hasattr(message, 'header') else None
            if name == 'atlas_odometry':
                tracked_poses[stamp] = (message.header.frame_id, message.child_frame_id)
            elif name == 'depth':
                depth_stamps.add(stamp)
            elif name == 'atlas_localization_status':
                for status in message.status:
                    if status.name != 'atlas/localization':
                        continue
                    values = {value.key: value.value for value in status.values}
                    if values.get('healthy') != 'true':
                        unhealthy_status_count += 1
                        continue
                    required = ('pose_valid', 'map_id_valid', 'coordinate_frame_ready', 'matching_odometry')
                    if (status.level != 0 or len(values) != len(status.values) or not status.hardware_id or
                            values.get('schema') != 'autonomy.atlas.tracking.v1' or
                            values.get('sensor_mode') != 'rgbd' or
                            values.get('odometry_topic') != args.prefix + '/atlas/odometry' or
                            not all(values.get(key) == 'true' for key in required) or
                            not all(values.get(key, '').isdigit() for key in
                                    ('map_id', 'coordinate_epoch', 'destructive_epoch'))):
                        report['diagnostic_error'] = 'invalid healthy tracking diagnostic metadata'
                        continue
                    healthy_status[stamp] = (message.header.frame_id, values.get('camera_frame', ''))
                    status_instances.add(status.hardware_id)
                    status_maps.add((values['map_id'], values['coordinate_epoch'], values['destructive_epoch']))
            if stamp in tracked_poses and stamp in healthy_status:
                if tracked_poses[stamp] == healthy_status[stamp] and all(tracked_poses[stamp]):
                    matched_status.add(stamp)
                else:
                    report['diagnostic_error'] = 'tracking diagnostic and estimated pose frames disagree'
            if stamp in matched_status and stamp in depth_stamps:
                matched_depth.add(stamp)
            for cache in (tracked_poses, healthy_status):
                while len(cache) > 2048:
                    cache.pop(next(iter(cache)))
        changed.set()

    def launch():
        try:
            deadline = time.monotonic() + 30
            while not stopping.is_set() and time.monotonic() < deadline:
                with data_lock:
                    ready = all(name in first for name in ('camera_info', 'rgb', 'depth', 'tf_static'))
                    snapshot = dict(first) if ready else {}
                if ready:
                    info = snapshot['camera_info']
                    base = habitat['sensors']['odom']['child_frame']
                    height = sensor_height(snapshot['tf_static'], base, info.header.frame_id)
                    settings = args.output / 'atlas-camera.yaml'
                    settings.write_text(camera_settings(info, snapshot['rgb'], snapshot['depth'],
                        habitat['sensors']['camera']['rate_hz'], height))
                    command = [str(args.localization), '--localization_mode', 'atlas',
                        '--atlas_sensor', 'rgbd', '--atlas_config', str(settings),
                        '--atlas_vocabulary', str(args.vocabulary),
                        '--atlas_image_topic', channels['rgb'], '--atlas_depth_image_topic', channels['depth'],
                        '--atlas_imu_topic', args.prefix + '/disabled/imu',
                        '--atlas_right_image_topic', args.prefix + '/disabled/right',
                        '--atlas_tf_topic', channels['tf'], '--atlas_tf_static_topic', channels['tf_static'],
                        '--atlas_map_frame', 'atlas_optical_map', '--atlas_camera_frame', info.header.frame_id,
                        '--atlas_publish_tf', 'false', '--atlas_ground_truth_odometry_topic', '',
                        '--atlas_ground_truth_trajectory_topic', '',
                        '--atlas_localization_status_topic', args.prefix + '/atlas/localization_status']
                    for name in ('odometry', 'trajectory', 'optimized_trajectory', 'map_points',
                                 'dense_map_points', 'occupancy_grid', 'tracking_image', 'feature_matches', 'loop_closures'):
                        command.extend(['--atlas_' + name + '_topic', args.prefix + '/atlas/' + name])
                    environment = dict(os.environ)
                    environment['AUTONOMY_ATLAS_INSTANCE_LOCK_PATH'] = str(args.output / 'atlas.lock')
                    report['command'] = command
                    report['camera_height_from_static_tf'] = height
                    report['camera_settings_sha256'] = hashlib.sha256(settings.read_bytes()).hexdigest()
                    report['ground_truth_input_disabled'] = True
                    with (args.output / 'atlas.log').open('xb') as log:
                        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=environment)
                        process_owner.append(child)
                        while not stopping.wait(.1) and child.poll() is None:
                            pass
                        if child.poll() is None:
                            child.terminate()
                        try:
                            code = child.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            child.kill()
                            code = child.wait(timeout=5)
                            report['forced_application_stop'] = True
                        report['application_returncode'] = code
                    return
                changed.wait(.1)
                changed.clear()
            raise RuntimeError('live camera calibration/static installation was not available before timeout')
        except Exception as error:
            report['application_error'] = f'{type(error).__name__}: {error}'

    launcher = None
    try:
        with InstanceLock.from_environment(args.config):
            runner = Runner(config, max_steps=args.steps)
            types = runner.message_types()
            for name in ('rgb', 'depth', 'camera_info', 'map_grid', 'gt_pose', 'clock', 'tf_static'):
                readers.append(runner.node.create_reader(channels[name],
                    lambda message, key=name: receive(message, key), types[name]))
            for name, kind in [('odometry', types['odom']), ('occupancy_grid', types['map_grid']),
                               ('trajectory', NativePath), ('localization_status', DiagnosticArray)]:
                readers.append(runner.node.create_reader(args.prefix + '/atlas/' + name,
                    lambda message, key='atlas_' + name: receive(message, key), kind))
            launcher = threading.Thread(target=launch, name='atlas-validation-launcher')
            launcher.start()
            runner.run()
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        stopping.set()
        if launcher:
            launcher.join(timeout=16)
            if launcher.is_alive():
                report['application_error'] = 'application monitor did not finish'
        with data_lock:
            report['message_counts'] = dict(counts)
            payloads = {name + '_first': value.SerializeToString() for name, value in first.items()}
            payloads.update({name + '_last': value.SerializeToString() for name, value in last.items()})
            grid = last.get('atlas_occupancy_grid')
            report['algorithm_known_cells'] = sum(value >= 0 for value in grid.data) if grid else 0
            report['healthy_localization_pairs'] = len(matched_status)
            report['healthy_depth_localization_triples'] = len(matched_depth)
            report['unhealthy_localization_diagnostics'] = unhealthy_status_count
            report['localization_status_instances'] = sorted(status_instances)
            report['localization_status_map_identities'] = sorted(status_maps)
            report['tracking_diagnostics_checked'] = not report.get('diagnostic_error') and len(matched_depth) >= 5
        report['payload_sha256'] = {}
        for name, payload in payloads.items():
            (args.output / (name + '.pb')).write_bytes(payload)
            report['payload_sha256'][name] = hashlib.sha256(payload).hexdigest()
        log = args.output / 'atlas.log'
        if log.is_file():
            report['application_log_sha256'] = hashlib.sha256(log.read_bytes()).hexdigest()
        report['elapsed_seconds'] = time.monotonic() - start
        report['passed'] = bool(not report.get('error') and not report.get('application_error') and
            report['tracking_diagnostics_checked'] and
            not report.get('forced_application_stop') and report.get('application_returncode') == 0 and
            counts.get('atlas_odometry', 0) >= 5 and counts.get('atlas_occupancy_grid', 0) >= 2 and
            report['algorithm_known_cells'] > 0 and report.get('ground_truth_input_disabled'))
        with (args.output / 'report.json').open('x') as output:
            json.dump(report, output, indent=2)
            output.write('\n')
    print(json.dumps(report), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
