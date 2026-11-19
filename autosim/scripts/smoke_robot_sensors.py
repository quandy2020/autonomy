#!/usr/bin/env python3
"""Verify the Google Robot sensor suite publishes over Autolink.

Runs the runtime for a few seconds and subscribes to the centre RGB-D, the
stereo pair, the IMU and every ToF / cliff beam. Checks image size, encoding,
frame ids, the stereo baseline encoded in the right camera_info projection
matrix, and that the range ring reports finite distances on a real scene.
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
from automsgs.msgs.sensor_msgs.range_pb2 import Range

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--profile', default='google_robot')
parser.add_argument('--output', type=Path, default=Path('/tmp/autosim-sensors'))
parser.add_argument('--scene', default='', help='Optional Habitat scene GLB for the parallax check')
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)

data = Config.load(UrdfModel.package_root() / 'config/robots' / f'{args.profile}.yaml').data
if args.scene:
    data['habitat']['path'] = args.scene
prefix = f'/autosim/sensors/{args.profile}'


def isolate(block):
    for key, value in list(block.items()):
        if isinstance(value, dict):
            isolate(value)
        elif isinstance(value, str) and value.startswith('/') and key.endswith(('channel', 'cmd_vel')):
            block[key] = prefix + value


isolate(data)
# The stereo block carries its own channel keys, so isolate them explicitly.
stereo_block = data['habitat']['sensors'].setdefault('stereo', {})
stereo_block.setdefault('left_channel', '/camera/left/image_raw')
stereo_block.setdefault('right_channel', '/camera/right/image_raw')
stereo_block.setdefault('left_info_channel', '/camera/left/camera_info')
stereo_block.setdefault('right_info_channel', '/camera/right/camera_info')
for key in ('left_channel', 'right_channel', 'left_info_channel', 'right_info_channel'):
    stereo_block[key] = prefix + stereo_block[key]
cameras = stereo_cameras(data['habitat']['sensors'])
# Range specs are keyed by URDF frame, so ``isolate`` skips their channels.
# Dict-form entries were already prefixed above; only bare strings need it.
range_block = data['habitat']['sensors'].setdefault('range', {})
for frame, value in list((range_block.get('sensors') or {}).items()):
    if isinstance(value, str) and not value.startswith(prefix):
        range_block['sensors'][frame] = prefix + value
beams = range_sensors(data['habitat']['sensors'])
imu_channel = data['habitat']['sensors']['imu']['channel']
received = {}
runner = None
try:
    runner = Runner(Config.load(data))
    runner.sensor_worker.start()
    observer = autolink.Node('autosim_sensor_verification')
    for camera in cameras:
        for key in (camera['rgb_key'], camera['info_key']):
            received[key] = None
    received['centre_rgb'] = None
    received['centre_depth'] = None
    for camera in cameras:
        observer.create_reader(camera['rgb_channel'],
                               lambda m, k=camera['rgb_key']: received.__setitem__(k, m), Image)
        observer.create_reader(camera['info_channel'],
                               lambda m, k=camera['info_key']: received.__setitem__(k, m), CameraInfo)
    observer.create_reader(prefix + '/camera/rgb/image_raw',
                           lambda m: received.__setitem__('centre_rgb', m), Image)
    observer.create_reader(prefix + '/camera/depth/image_raw',
                           lambda m: received.__setitem__('centre_depth', m), Image)
    received['imu'] = None
    observer.create_reader(imu_channel, lambda m: received.__setitem__('imu', m), Imu)
    for beam in beams:
        received[beam['frame']] = None
        observer.create_reader(
            beam['channel'],
            lambda m, k=beam['frame']: received.__setitem__(k, m),
            Range,
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
            'frame': info.header.frame_id, 'size': [image.shape[1], image.shape[0]],
            'mean': round(float(image.mean()), 2), 'tx': round(float(info.p[3]), 4),
        }
        cv2.imwrite(str(args.output / f'{camera["side"]}.png'), image[..., ::-1])
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
    result['stereo'] = {'baseline_m': baseline, 'fx': round(fx, 3),
                        'right_tx': round(float(received[cameras[1]['info_key']].p[3]), 4),
                        'parallax': round(parallax, 2)}
    centre = decode(received['centre_rgb'])
    depth = np.frombuffer(received['centre_depth'].data, dtype=np.float32)
    result['centre'] = {'size': [centre.shape[1], centre.shape[0]],
                        'mean': round(float(centre.mean()), 2),
                        'depth_mean': round(float(depth.mean()), 3)}
    imu = received['imu']
    assert imu.header.frame_id == data['habitat']['sensors']['imu']['frame'], imu.header.frame_id
    result['imu'] = {
        'frame': imu.header.frame_id,
        'orientation_z': round(float(imu.orientation.z), 4),
        'orientation_w': round(float(imu.orientation.w), 4),
        'accel_z': round(float(imu.linear_acceleration.z), 4),
    }
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
    result['ok'] = True
    (args.output / f'{args.profile}.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)
finally:
    if runner is not None:
        runner.sensor_worker.stop()
        runner.simulator.close()
    autolink.shutdown()
