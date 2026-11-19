"""Habitat URDF viewer with browser sliders, orbit camera and PNG export."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from autosim.robot.profiles import PROFILES, apply_profile, shipped_robot_defaults
from autosim.robot.joints import load_joints
from autosim.robot.urdf import UrdfModel
from autosim.robot.plant import Robot
from autosim.runtime.settings import Config
from autosim.backends.habitat.simulator import Simulator


def viewer_settings(profile: str, scene: str = '', width: int = 960, height: int = 720) -> dict:
    robot = apply_profile({'profile': profile, 'instance': True, 'fixed_base': True,
                           'cmd_vel': '/cmd_vel',
                           'watchdog_sec': 0.5, 'control_hz': 50.0, 'map_frame': 'map'})
    robot.update(shipped_robot_defaults(profile))
    habitat = {
        'backend': 'habitat', 'path': scene, 'gpu': 0, 'spawn': [0,0,0], 'robot': robot,
        'sensors': {'camera': {'enabled': False, 'width': width, 'height': height,
                              'rgb_channel': '/camera/rgb/image_raw',
                              'depth_channel': '/camera/depth/image_raw',
                              'info_channel': '/camera/camera_info',
                              'frame': 'camera_link', 'rate_hz': 10.0},
                    'chase': {'enabled': True, 'width': width, 'height': height,
                              'clear_color': [0.3, 0.34, 0.4, 1.0],
                              'distance_m': 3.5, 'height_m': 2.2, 'look_at_height_m': 0.8}},
    }
    if not str(scene or '').strip():
        # A floating robot is hard to read: give the preview a lit studio floor.
        habitat['scenario'] = {
            'enabled': True, 'type': 'floor',
            'x_length': 24.0, 'y_length': 24.0, 'z_length': 0.2,
            'resolution': 0.1, 'spawn_radius': 1.0, 'spawn_height': 1.5,
        }
        habitat['render'] = {
            'enable_hbao': True,
            'clear_color': [0.55, 0.57, 0.60, 1.0],
        }
    return Config.load({'habitat': habitat}).data


def camera_pose(sim: Simulator, azimuth: float, elevation: float, distance: float, target_height: float) -> None:
    import magnum as mn
    sensor = sim.session.get_agent(0)._sensors['chase_rgb']
    node = getattr(sensor, '_sensor_object', sensor).node
    target = mn.Vector3(sim.x, sim.floor_y + target_height, -sim.y)
    eye = target + mn.Vector3(distance*math.cos(elevation)*math.cos(azimuth),
                              distance*math.sin(elevation),
                              distance*math.cos(elevation)*math.sin(azimuth))
    world = mn.Matrix4.look_at(eye, target, mn.Vector3(0,1,0))
    # Magnum look_at gives a camera-to-world transform.
    node.transformation = node.parent.absolute_transformation().inverted() @ world


def render(sim: Simulator, camera: dict) -> np.ndarray:
    camera_pose(sim, **camera)
    sim.set_articulated_visible(visible=True)
    try:
        rgb = sim.session.get_sensor_observations()['chase_rgb'][...,:3]
        return np.ascontiguousarray(rgb, dtype=np.uint8)
    finally:
        sim.set_articulated_visible(visible=False)


def frame_robot(sim: Simulator) -> dict:
    """Recompute mesh bounds before framing; Habitat initially uses a sentinel."""
    import magnum as mn
    import itertools

    # Habitat's AO root bounds can contain only the base; articulated arm
    # visuals are separate link nodes. Frame the union of every visual link.
    corners = []
    for link_id in [-1, *sim.articulated.get_link_ids()]:
        for node in sim.articulated.get_link_visual_nodes(link_id):
            node.compute_cumulative_bb()
            bb = node.cumulative_bb
            transform = node.absolute_transformation()
            corners.extend(tuple(transform.transform_point(mn.Vector3(*corner)))
                           for corner in itertools.product(*zip(bb.min, bb.max)))
    if not corners:
        raise RuntimeError('robot has no visual bounds')
    points = np.asarray(corners, dtype=float)
    low, high = points.min(axis=0), points.max(axis=0)
    size = max(float((high-low).max()), 0.5)
    return {'azimuth': math.radians(140), 'elevation': math.radians(15),
            'distance': size*1.8, 'target_height': float((low[1]+high[1])/2-sim.floor_y)}


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=sorted(PROFILES), default='google_robot')
    parser.add_argument('--scene', default='', help='Optional Habitat scene GLB')
    parser.add_argument('--list', action='store_true', help='Print movable joints without opening Habitat')
    parser.add_argument('--joint', action='append', default=[], metavar='NAME=POSITION')
    parser.add_argument('--output', type=Path, help='Render one PNG and exit')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--cmd-vel-channel', default='/cmd_vel',
                        help='Autolink TwistStamped channel; empty disables subscription')
    args = parser.parse_args(argv)
    cfg = viewer_settings(args.profile, args.scene)
    joints = load_joints(UrdfModel.resolve(cfg['habitat']['robot']['urdf']))
    if args.list:
        for joint in joints.values():
            print(f'{joint.name}\t{joint.kind}\t[{joint.lower}, {joint.upper}]'+
                  (f'\tmimics {joint.mimic}' if joint.mimic else ''))
        return
    updates = {}
    for assignment in args.joint:
        name,value = assignment.split('=',1)
        updates[name] = float(value)
    # Image encoding is optional and imported only for rendering.
    import cv2
    sim = Simulator.create(cfg)
    try:
        sim.set_joint_positions(updates)
        # Frame the complete mesh automatically (large B2 and small Go2 alike).
        camera = frame_robot(sim)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(args.output), render(sim,camera)[...,::-1]):
                raise RuntimeError(f'could not write {args.output}')
            print(args.output.resolve())
            return
        serve(sim, joints, camera, args.host, args.port, args.profile, cv2, args.cmd_vel_channel)
    finally:
        sim.close()


def home_pose(sim, joints: dict) -> dict:
    """Configured home pose, falling back to the URDF zero pose."""
    block = sim.settings['habitat']['robot'].get('joint_positions') or {}
    if block:
        return dict(block)
    return {n:j.home() for n,j in joints.items() if not j.mimic}


def serve(sim, joints, camera, host, port, profile, cv2, cmd_vel_channel='/cmd_vel'):
    from autosim.bridge.bridge import CommandReader
    from autosim.bridge.messages import Messages
    from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped

    config = sim.settings['habitat']['robot']
    plant = Robot(max_linear=config['max_linear'], max_angular=config['max_angular'],
                  watchdog_sec=config.get('watchdog_sec', 0.5),
                  x=sim.x, y=sim.y, yaw=sim.yaw)
    link = None
    reader = None
    if cmd_vel_channel:
        import autolink

        link = autolink
        link.init('autosim_robot_viewer')
        node = link.Node('autosim_robot_viewer')
        reader = CommandReader(node, cmd_vel_channel, TwistStamped)
    last_tick = time.monotonic()
    requests: queue.Queue = queue.Queue(maxsize=32)
    lock = threading.Lock()
    shared = {'image': b'', 'state': {}}
    page = Path(__file__).with_name('viewer.html').read_bytes()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass
        def reply(self, code, content, kind='application/json'):
            self.send_response(code)
            self.send_header('Content-Type',kind)
            self.send_header('Content-Length',str(len(content)))
            self.send_header('Cache-Control','no-store')
            self.end_headers()
            self.wfile.write(content)
        def do_GET(self):
            if self.path == '/':
                self.reply(200,page,'text/html; charset=utf-8')
                return
            with lock:
                if self.path.startswith('/frame'):
                    content,kind = shared['image'],'image/jpeg'
                elif self.path.split('?', 1)[0] in ('/state', '/api/robot-state'):
                    content,kind = json.dumps(shared['state']).encode(),'application/json'
                else:
                    self.reply(404,b'{}')
                    return
            self.reply(200,content,kind)
        def do_POST(self):
            if self.path != '/command':
                self.reply(404,b'{}')
                return
            try:
                length = int(self.headers.get('Content-Length','0'))
                if not 0 < length <= 65536:
                    raise ValueError('invalid command length')
                payload = json.loads(self.rfile.read(length))
                done, result = threading.Event(), {}
                requests.put_nowait((payload,done,result))
                if not done.wait(5):
                    raise ValueError('simulation command timed out')
                self.reply(400 if 'error' in result else 200,json.dumps(result).encode())
            except (ValueError, queue.Full) as exc:
                self.reply(400,json.dumps({'error':str(exc)}).encode())
    server = ThreadingHTTPServer((host,port),Handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f'Habitat viewer: http://{host}:{port} ({profile})',flush=True)
    try:
        while True:
            now = time.monotonic()
            dt = min(now - last_tick, 0.1)
            last_tick = now
            if reader is not None and reader.has_msg():
                try:
                    linear, angular = Messages.parse_command_velocity(reader.get_msg())
                    plant.set_twist(linear, angular, now)
                except ValueError as exc:
                    print(f'cmd_vel rejected: {exc}', flush=True)
            while not requests.empty():
                payload, done, result = requests.get_nowait()
                try:
                    if not isinstance(payload,dict):
                        raise ValueError('command must be an object')
                    if 'profile' in payload:
                        selected = str(payload['profile'])
                        if selected not in PROFILES:
                            raise ValueError(f'unknown profile: {selected}')
                        sim.switch_robot(selected)
                        profile = selected
                        joints = dict(sim.articulation.joints)
                        camera = frame_robot(sim)
                        settings = apply_profile({'profile': selected})
                        plant.max_linear = settings.get('max_linear', 0.5)
                        plant.max_angular = settings.get('max_angular', 1.0)
                        plant.set_twist(0.0, 0.0, now)
                        plant.linear = plant.angular = 0.0
                    if payload.get('reset'):
                        sim.set_joint_positions(home_pose(sim, joints))
                    if 'cmd_vel' in payload:
                        twist = payload['cmd_vel']
                        plant.set_twist(twist['linear_x'], twist['angular_z'], now)
                    if 'joints' in payload:
                        sim.set_joint_positions(payload['joints'])
                    if 'camera' in payload:
                        updates = payload['camera']
                        if not isinstance(updates,dict) or set(updates)-set(camera):
                            raise ValueError('invalid camera fields')
                        values = {k:float(v) for k,v in updates.items()}
                        if not all(math.isfinite(v) for v in values.values()):
                            raise ValueError('camera values must be finite')
                        if 'distance' in values and values['distance'] <= 0:
                            raise ValueError('camera distance must be positive')
                        if 'elevation' in values and abs(values['elevation']) >= math.pi/2:
                            raise ValueError('camera elevation must be within (-pi/2, pi/2)')
                        camera.update(values)
                    result['ok'] = True
                except (ValueError, TypeError, RuntimeError, FileNotFoundError) as exc:
                    result['error'] = str(exc)
                finally:
                    done.set()
            sx, sy, _ = sim.pose()
            x, y, yaw = plant.step(dt, now)
            x, y = sim.constrain_motion(sx, sy, x, y)
            plant.x, plant.y = x, y
            sim.set_pose(x, y, yaw)
            sim.step()
            rgb = render(sim,camera)
            ok,jpeg = cv2.imencode('.jpg',rgb[...,::-1])
            if not ok:
                raise RuntimeError('JPEG encoding failed')
            positions = sim.joint_states()
            state = {'profile':profile,
                'profiles':[n for n,p in PROFILES.items() if UrdfModel.resolve(p.urdf).is_file()],
                'camera':dict(camera),'joints':[
                {'name':n,'type':j.kind,'position':positions[n],
                 'lower':j.lower,'upper':j.upper,'mimic':j.mimic} for n,j in joints.items()]}
            state.update(pose=dict(zip(('x', 'y', 'yaw'), sim.pose())),
                         velocity=dict(zip(('linear_x', 'angular_z'), plant.velocity())),
                         cmd_vel_channel=cmd_vel_channel)
            with lock:
                shared.update(image=jpeg.tobytes(),state=state)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        if link is not None:
            link.shutdown()


if __name__ == '__main__':
    main()
