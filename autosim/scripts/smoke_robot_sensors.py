#!/usr/bin/env python3
"""Verify the sensors enabled by one robot profile over Autolink.

Runs the runtime for a few seconds and subscribes only to the sensors enabled
in the selected profile. The Google profile exercises centre RGB-D, stereo,
IMU, ToF and cliff beams; the Unitree profiles exercise RGB-D, IMU and 2D
lidar. Checks image size, encoding, frame ids, stereo baseline, and that range
or lidar returns are present in a furnished scene.
"""
from pathlib import Path
import argparse
import json
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
# Use the same protobuf compatibility bootstrap as the project's tests.
import tests.conftest
import autolink
import cv2
import numpy as np

from autosim.runtime import Config, Runner
from autosim.robot.urdf import UrdfModel
from autosim.bridge.range import range_sensors
from autosim.bridge.stereo import stereo_cameras
from automsgs.msgs.sensor_msgs.camera_info_pb2 import CameraInfo
from automsgs.msgs.sensor_msgs.image_pb2 import Image
from automsgs.msgs.sensor_msgs.imu_pb2 import Imu
from automsgs.msgs.sensor_msgs.laser_scan_pb2 import LaserScan
from automsgs.msgs.sensor_msgs.range_pb2 import Range

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--profile', default='google_robot')
parser.add_argument('--output', type=Path, default=Path('/tmp/autosim-sensors'))
parser.add_argument('--scene', default='', help='Optional Habitat scene GLB')
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)

data = Config.load(UrdfModel.package_root() / 'config/robots' / f'{args.profile}.yaml').data
if args.scene:
    data['habitat']['path'] = args.scene
prefix = f'/autosim/sensors/{args.profile}'


def prefixed(value):
    """Prefix a channel once, leaving already isolated channels untouched."""
    if isinstance(value, str) and value.startswith('/') and not value.startswith(prefix):
        return prefix + value
    return value


def isolate(block):
    for key, value in list(block.items()):
        if isinstance(value, dict):
            isolate(value)
        elif key.endswith(('channel', 'cmd_vel')):
            block[key] = prefixed(value)


isolate(data)
sensors = data['habitat']['sensors']
camera_cfg = sensors.get('camera') or {}
imu_cfg = sensors.get('imu') or {}
lidar_cfg = sensors.get('lidar_2d') or {}
camera_enabled = bool(camera_cfg.get('enabled', False))
imu_enabled = bool(imu_cfg.get('enabled', False))
lidar_enabled = bool(lidar_cfg.get('enabled', False))

# The stereo block carries its own channel keys; isolate them explicitly.
stereo_block = sensors.setdefault('stereo', {})
for key, default in (
    ('left_channel', '/camera/left/image_raw'),
    ('right_channel', '/camera/right/image_raw'),
    ('left_info_channel', '/camera/left/camera_info'),
    ('right_info_channel', '/camera/right/camera_info'),
):
    stereo_block[key] = prefixed(stereo_block.get(key, default))
cameras = stereo_cameras(sensors)

# Range specs are keyed by URDF frame, so ``isolate`` skips bare strings.
range_block = sensors.setdefault('range', {})
for frame, value in list((range_block.get('sensors') or {}).items()):
    if isinstance(value, str):
        range_block['sensors'][frame] = prefixed(value)
beams = range_sensors(sensors)

if not (camera_enabled or cameras or imu_enabled or beams or lidar_enabled):
    raise SystemExit(f'profile {args.profile!r} has no enabled sensors to verify')

received = {}
runner = None
try:
    runner = Runner(Config.load(data))
    runner.sensor_worker.start()
    observer = autolink.Node('autosim_sensor_verification')
    for camera in cameras:
        for key in (camera['rgb_key'], camera['info_key']):
            received[key] = None
        observer.create_reader(
            camera['rgb_channel'],
            lambda m, k=camera['rgb_key']: received.__setitem__(k, m),
            Image,
        )
        observer.create_reader(
            camera['info_channel'],
            lambda m, k=camera['info_key']: received.__setitem__(k, m),
            CameraInfo,
        )
    if camera_enabled:
        received['centre_rgb'] = None
        received['centre_depth'] = None
        observer.create_reader(
            camera_cfg['rgb_channel'],
            lambda m: received.__setitem__('centre_rgb', m),
            Image,
        )
        observer.create_reader(
            camera_cfg['depth_channel'],
            lambda m: received.__setitem__('centre_depth', m),
            Image,
        )
    if imu_enabled:
        received['imu'] = None
        observer.create_reader(
            imu_cfg['channel'],
            lambda m: received.__setitem__('imu', m),
            Imu,
        )
    for beam in beams:
        received[beam['frame']] = None
        observer.create_reader(
            beam['channel'],
            lambda m, k=beam['frame']: received.__setitem__(k, m),
            Range,
        )
    if lidar_enabled:
        received['scan'] = None
        observer.create_reader(
            lidar_cfg['channel'],
            lambda m: received.__setitem__('scan', m),
            LaserScan,
        )

    deadline = time.time() + 20.0
    while time.time() < deadline and any(value is None for value in received.values()):
        runner.cycle(0.02)
        time.sleep(0.005)
    missing = [key for key, value in received.items() if value is None]
    assert not missing, f'missing topics: {missing}'

    def decode(image):
        array = np.frombuffer(image.data, dtype=np.uint8)
        return array.reshape(image.height, image.width, -1)

    result = {'profile': args.profile, 'topics': {}}
    for camera in cameras:
        image = decode(received[camera['rgb_key']])
        info = received[camera['info_key']]
        assert (image.shape[1], image.shape[0]) == (camera['width'], camera['height']), image.shape
        assert info.header.frame_id == camera['frame'], info.header.frame_id
        assert info.width == camera['width'] and info.height == camera['height']
        result['topics'][camera['rgb_channel']] = {
            'frame': info.header.frame_id,
            'size': [image.shape[1], image.shape[0]],
            'mean': round(float(image.mean()), 2),
            'tx': round(float(info.p[3]), 4),
        }
        cv2.imwrite(str(args.output / f'{camera["side"]}.png'), image[..., ::-1])
    if cameras:
        left = decode(received[cameras[0]['rgb_key']]).astype(float)
        right = decode(received[cameras[1]['rgb_key']]).astype(float)
        parallax = float(np.abs(left - right).mean())
        if float(left.mean()) > 1.0:
            assert parallax > 1.0, f'stereo pair shows no parallax: {parallax}'
        fx = float(received[cameras[0]['info_key']].p[0])
        baseline = float(cameras[0]['baseline_m'])
        expected = -fx * baseline
        assert abs(float(received[cameras[1]['info_key']].p[3]) - expected) < 1e-3
        assert abs(float(received[cameras[0]['info_key']].p[3])) < 1e-6
        result['stereo'] = {
            'baseline_m': baseline,
            'fx': round(fx, 3),
            'right_tx': round(float(received[cameras[1]['info_key']].p[3]), 4),
            'parallax': round(parallax, 2),
        }
    if camera_enabled:
        centre = decode(received['centre_rgb'])
        depth = np.frombuffer(received['centre_depth'].data, dtype=np.float32)
        assert (centre.shape[1], centre.shape[0]) == (
            int(camera_cfg['width']),
            int(camera_cfg['height']),
        ), centre.shape
        assert received['centre_rgb'].header.frame_id == camera_cfg['frame']
        cv2.imwrite(str(args.output / 'centre.png'), centre[..., ::-1])
        result['centre'] = {
            'frame': received['centre_rgb'].header.frame_id,
            'size': [centre.shape[1], centre.shape[0]],
            'mean': round(float(centre.mean()), 2),
            'depth_mean': round(float(depth.mean()), 3),
        }
    if imu_enabled:
        imu = received['imu']
        assert imu.header.frame_id == imu_cfg['frame'], imu.header.frame_id
        result['imu'] = {
            'frame': imu.header.frame_id,
            'orientation_z': round(float(imu.orientation.z), 4),
            'orientation_w': round(float(imu.orientation.w), 4),
            'accel_z': round(float(imu.linear_acceleration.z), 4),
        }
    if beams:
        ranges = {}
        for beam in beams:
            message = received[beam['frame']]
            assert message.header.frame_id == beam['frame'], message.header.frame_id
            assert abs(float(message.max_range) - beam['max_range']) < 1e-6
            ranges[beam['channel']] = (
                None if not np.isfinite(message.range) else round(float(message.range), 3)
            )
        result['range'] = ranges
        if args.scene:
            hits = [value for value in ranges.values() if value is not None]
            assert hits, 'range ring saw no returns in a furnished scene'
    if lidar_enabled:
        scan = received['scan']
        assert scan.header.frame_id == lidar_cfg['frame'], scan.header.frame_id
        assert len(scan.ranges) == int(lidar_cfg['num_beams']), len(scan.ranges)
        finite = [float(value) for value in scan.ranges if np.isfinite(value)]
        if args.scene:
            assert finite, 'lidar saw no returns in a furnished scene'
        result['lidar'] = {
            'frame': scan.header.frame_id,
            'beams': len(scan.ranges),
            'finite': len(finite),
            'min': round(min(finite), 3) if finite else None,
            'max': round(max(finite), 3) if finite else None,
        }
    result['ok'] = True
    (args.output / f'{args.profile}.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)
finally:
    if runner is not None:
        runner.sensor_worker.stop()
        runner.simulator.close()
    autolink.shutdown()
