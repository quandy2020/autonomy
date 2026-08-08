#!/usr/bin/env python3
"""Home-service robot all-topic simulator for Autoviz debugging.

Publishes the common monitoring channels across sensors, localization,
navigation, perception, control, diagnostics, tasks, and debug overlays.

Custom / missing pkgs mapped to automsgs stand-ins (see README §18).

  /usr/bin/python3 examples/python/18_tutorial_robot_all_data_sim.py
  ./build/autonomy/bin/autoviz
"""

from __future__ import annotations

from _reexec import early_reexec_if_needed

early_reexec_if_needed()

import argparse
import json
import math
import struct
import time
from pathlib import Path

from _bootstrap import prepare_example_environment

prepare_example_environment()

import autolink
from automsgs.msgs.action_msgs import goal_status_array_pb2, goal_status_pb2
from automsgs.msgs.diagnostic_msgs import (
    diagnostic_array_pb2,
    diagnostic_status_pb2,
)
from automsgs.msgs.geometry_msgs import (
    pose_stamped_pb2,
    pose_with_covariance_stamped_pb2,
    twist_pb2,
    wrench_stamped_pb2,
)
from automsgs.msgs.map_msgs import occupancy_grid_pb2
from automsgs.msgs.nav_msgs import odometry_pb2, path_pb2
from automsgs.msgs.sensor_msgs import (
    battery_state_pb2,
    image_pb2,
    imu_pb2,
    joint_state_pb2,
    joy_pb2,
    laser_scan_pb2,
    nav_sat_fix_pb2,
    nav_sat_status_pb2,
    point_cloud2_pb2,
    point_field_pb2,
    range_pb2,
)
from automsgs.msgs.std_msgs import float64_pb2, string_pb2
from automsgs.msgs.tf2_msgs import tf_message_pb2
from automsgs.msgs.vision_msgs import detection3d_array_pb2
from automsgs.msgs.visualization_msgs import marker_array_pb2, marker_pb2

DS = diagnostic_status_pb2.DiagnosticStatus
GS = goal_status_pb2.GoalStatus
M = marker_pb2.Marker
URDF = Path(__file__).resolve().parent / 'urdf' / 'pr2_simple.urdf'
JOINTS = (
    'torso_lift_joint', 'head_pan_joint', 'head_tilt_joint',
    'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_elbow_flex_joint',
    'l_wrist_flex_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint',
    'r_elbow_flex_joint', 'r_wrist_flex_joint',
)
RW, RH, DW, DH = 160, 120, 80, 60
GW, GH, GRES = 40, 30, 0.1
TASKS = ('IDLE', 'NAVIGATING', 'GRASPING', 'SPEAKING', 'DONE')


def stamp(h, frame='map'):
    now = time.time_ns()
    h.frame_id = frame
    h.stamp.sec = now // 1_000_000_000
    h.stamp.nanosec = now % 1_000_000_000


def set_pose(pose, x, y, yaw, z=0.05):
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.z = math.sin(yaw * 0.5)
    pose.orientation.w = math.cos(yaw * 0.5)


def robot_xy(t):
    x, y = 2.0 * math.cos(t * 0.25), 1.5 * math.sin(t * 0.25)
    yaw = t * 0.25 + math.pi / 2
    return x, y, yaw


def make_rgb(phase, frame='camera'):
    msg = image_pb2.Image()
    stamp(msg.header, frame)
    msg.height, msg.width, msg.encoding, msg.step = RH, RW, 'rgb8', RW * 3
    buf = bytearray(RH * RW * 3)
    for y in range(RH):
        for x in range(RW):
            i = (y * RW + x) * 3
            buf[i] = (x + phase) % 256
            buf[i + 1] = (y + phase // 2) % 256
            buf[i + 2] = 80
    msg.data = bytes(buf)
    return msg


def make_depth():
    msg = image_pb2.Image()
    stamp(msg.header, 'camera')
    msg.height, msg.width, msg.encoding, msg.step = DH, DW, '16UC1', DW * 2
    cx, cy = DW // 2, DH // 2
    buf = bytearray()
    for y in range(DH):
        for x in range(DW):
            mm = 1200 if (x - cx) ** 2 + (y - cy) ** 2 < 80 else 2200
            buf += struct.pack('<H', mm)
    msg.data = bytes(buf)
    return msg


def make_scan(t):
    msg = laser_scan_pb2.LaserScan()
    stamp(msg.header, 'laser')
    n = 180
    msg.angle_min, msg.angle_max = -math.pi / 2, math.pi / 2
    msg.angle_increment = math.pi / (n - 1)
    msg.range_min, msg.range_max, msg.scan_time = 0.1, 10.0, 0.1
    for i in range(n):
        a = msg.angle_min + i * msg.angle_increment
        msg.ranges.append(2.0 + 0.4 * math.sin(a * 3 + t))
    return msg


def make_pc2(t):
    msg = point_cloud2_pb2.PointCloud2()
    stamp(msg.header, 'camera')
    n = 60
    msg.height, msg.width, msg.is_dense = 1, n, True
    msg.point_step, msg.row_step = 12, 12 * n
    for name, off in (('x', 0), ('y', 4), ('z', 8)):
        f = msg.fields.add()
        f.name, f.offset, f.count = name, off, 1
        f.datatype = point_field_pb2.PointField.FLOAT32
    buf = bytearray()
    for i in range(n):
        buf += struct.pack('<fff', 0.05 * i, math.sin(0.2 * i + t), 0.05 * (i % 5))
    msg.data = bytes(buf)
    return msg


def make_imu(t):
    msg = imu_pb2.Imu()
    stamp(msg.header, 'imu_link')
    yaw = 0.2 * math.sin(t)
    msg.orientation.z = math.sin(yaw * 0.5)
    msg.orientation.w = math.cos(yaw * 0.5)
    msg.angular_velocity.z = 0.25 * math.cos(t)
    msg.linear_acceleration.z = 9.81
    return msg


def make_gps(t):
    msg = nav_sat_fix_pb2.NavSatFix()
    stamp(msg.header, 'gps')
    msg.status.status = nav_sat_status_pb2.NavSatStatus.STATUS_FIX
    msg.status.service = nav_sat_status_pb2.NavSatStatus.SERVICE_GPS
    msg.latitude = 31.2304 + 1e-5 * math.sin(t * 0.1)
    msg.longitude = 121.4737 + 1e-5 * math.cos(t * 0.1)
    msg.altitude = 12.0
    msg.position_covariance.extend([1.0, 0, 0, 0, 1.0, 0, 0, 0, 4.0])
    msg.position_covariance_type = msg.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return msg


def make_battery(t):
    msg = battery_state_pb2.BatteryState()
    stamp(msg.header)
    pct = 0.75 + 0.1 * math.sin(t * 0.05)
    msg.voltage, msg.current, msg.percentage = 12.4, -1.2, pct
    msg.capacity, msg.design_capacity, msg.present = 5.0, 5.2, True
    msg.power_supply_status = msg.POWER_SUPPLY_STATUS_DISCHARGING
    msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_GOOD
    msg.power_supply_technology = msg.POWER_SUPPLY_TECHNOLOGY_LION
    msg.location, msg.serial_number = 'base', 'BAT-001'
    return msg


def make_range(t):
    msg = range_pb2.Range()
    stamp(msg.header, 'ultrasonic')
    msg.radiation_type = range_pb2.Range.ULTRASOUND
    msg.field_of_view = 0.3
    msg.min_range, msg.max_range = 0.05, 4.0
    msg.range = 1.2 + 0.4 * math.sin(t)
    return msg


def make_js(t):
    msg = joint_state_pb2.JointState()
    stamp(msg.header)
    msg.name.extend(JOINTS)
    msg.position.extend([0.2 * math.sin(t * 0.4 + i) for i in range(len(JOINTS))])
    msg.velocity.extend([0.1 * math.cos(t * 0.4 + i) for i in range(len(JOINTS))])
    msg.effort.extend([10 * math.sin(t * 0.5 + i * 0.3) for i in range(len(JOINTS))])
    return msg


def make_wrench(t):
    msg = wrench_stamped_pb2.WrenchStamped()
    stamp(msg.header, 'gripper')
    msg.wrench.force.z = 2.0 + math.sin(t)
    msg.wrench.torque.y = 0.5 * math.cos(t)
    return msg


def make_odom(t):
    x, y, yaw = robot_xy(t)
    msg = odometry_pb2.Odometry()
    stamp(msg.header)
    msg.child_frame_id = 'base_link'
    stamp(msg.pose.pose.header)
    set_pose(msg.pose.pose.pose, x, y, yaw)
    msg.twist.twist.linear.x = 0.4
    msg.twist.twist.angular.z = 0.25
    return msg


def make_tf(t):
    x, y, yaw = robot_xy(t)
    msg = tf_message_pb2.TFMessage()
    for parent, child, tx, ty, tz, ryaw in (
        ('map', 'odom', 0.05, 0.0, 0.0, 0.0),
        ('odom', 'base_link', x, y, 0.0, yaw),
        ('base_link', 'laser', 0.1, 0.0, 0.2, 0.0),
        ('base_link', 'camera', 0.15, 0.0, 0.4, 0.0),
        ('base_link', 'imu_link', 0.0, 0.0, 0.3, 0.0),
    ):
        tr = msg.transforms.add()
        stamp(tr.header, parent)
        tr.child_frame_id = child
        tr.transform.translation.x, tr.transform.translation.y = tx, ty
        tr.transform.translation.z = tz
        tr.transform.rotation.z = math.sin(ryaw * 0.5)
        tr.transform.rotation.w = math.cos(ryaw * 0.5)
    return msg


def make_grid(channel_tag=0):
    msg = occupancy_grid_pb2.OccupancyGrid()
    stamp(msg.header)
    msg.info.resolution, msg.info.width, msg.info.height = GRES, GW, GH
    msg.info.origin.position.x = -0.5 * GW * GRES
    msg.info.origin.position.y = -0.5 * GH * GRES
    msg.info.origin.orientation.w = 1.0
    data = []
    for y in range(GH):
        for x in range(GW):
            edge = x in (0, GW - 1) or y in (0, GH - 1)
            cost = 100 if edge else (50 if (x + y + channel_tag) % 11 == 0 else 0)
            data.append(cost)
    msg.data.extend(data)
    return msg


def make_amcl(t):
    x, y, yaw = robot_xy(t)
    msg = pose_with_covariance_stamped_pb2.PoseWithCovarianceStamped()
    stamp(msg.header)
    stamp(msg.pose.pose.header)
    set_pose(msg.pose.pose.pose, x + 0.02, y - 0.01, yaw)
    msg.pose.covariance.extend([0.05 if i % 7 == 0 else 0.0 for i in range(36)])
    return msg


def make_path(t, n=30, radius=2.0, local=False):
    msg = path_pb2.Path()
    stamp(msg.header)
    x0, y0, _ = robot_xy(t)
    for i in range(n):
        u = i / max(n - 1, 1)
        ps = msg.poses.add()
        stamp(ps.header)
        if local:
            set_pose(ps.pose, x0 + 0.4 * u, y0 + 0.15 * math.sin(u * 6 + t), 0.0)
        else:
            a = t * 0.25 + u * 1.5
            set_pose(ps.pose, radius * math.cos(a), 1.5 * math.sin(a), a + math.pi / 2)
    return msg


def make_cmd(t):
    msg = twist_pb2.Twist()
    msg.linear.x = 0.3 + 0.1 * math.sin(t)
    msg.angular.z = 0.2 * math.cos(t * 0.5)
    return msg


def make_nav_status(t):
    msg = goal_status_array_pb2.GoalStatusArray()
    st = msg.status_list.add()
    st.goal_info.goal_id.uuid = bytes([(i * 17) % 256 for i in range(16)])
    now = time.time_ns()
    st.goal_info.stamp.sec = now // 1_000_000_000
    st.goal_info.stamp.nanosec = now % 1_000_000_000
    phase = int(t) % 20
    st.status = (
        GS.STATUS_SUCCEEDED if phase > 16 else
        GS.STATUS_ABORTED if phase > 14 else
        GS.STATUS_EXECUTING)
    return msg


def make_waypoint(t):
    msg = pose_stamped_pb2.PoseStamped()
    stamp(msg.header)
    set_pose(msg.pose, 3.0 * math.cos(t * 0.05), 2.0 * math.sin(t * 0.05), 0.0, 0.0)
    return msg


def make_detections3d(t, people=False):
    msg = detection3d_array_pb2.Detection3DArray()
    stamp(msg.header)
    labels = ('person',) if people else ('cup', 'bottle', 'chair')
    for i, cls in enumerate(labels):
        a = t * 0.4 + i * 2.0
        det = msg.detections.add()
        stamp(det.header)
        det.id = f'{cls}_{i}'
        det.bbox.center.position.x = 1.5 * math.cos(a)
        det.bbox.center.position.y = 1.5 * math.sin(a)
        det.bbox.center.position.z = 0.6 if people else 0.3
        det.bbox.center.orientation.w = 1.0
        det.bbox.size.x = 0.4
        det.bbox.size.y = 0.4
        det.bbox.size.z = 1.6 if people else 0.4
        hyp = det.results.add()
        hyp.hypothesis.class_id = cls
        hyp.hypothesis.score = 0.8 + 0.05 * i
    return msg


def make_joy(t):
    msg = joy_pb2.Joy()
    stamp(msg.header)
    msg.axes.extend([math.sin(t), math.cos(t * 0.7), 0.0, 0.0])
    msg.buttons.extend([1 if math.sin(t) > 0 else 0, 0, 0, 0])
    return msg


def make_diagnostics(t):
    msg = diagnostic_array_pb2.DiagnosticArray()
    stamp(msg.header)
    rows = (
        (DS.OK, 'power:battery', f'{75 + 5 * math.sin(t * 0.05):.0f}%'),
        (DS.WARN if math.sin(t) > 0.85 else DS.OK, 'sensor:lidar', 'streaming'),
        (DS.OK, 'nav:controller', 'tracking'),
        (DS.ERROR if int(t) % 25 > 22 else DS.OK, 'network:wifi', 'ok'),
    )
    for level, name, message in rows:
        st = msg.status.add()
        st.level, st.name, st.message, st.hardware_id = level, name, message, name
    return msg


def make_markers(t, ns):
    arr = marker_array_pb2.MarkerArray()
    for i in range(4):
        m = arr.markers.add()
        m.ns, m.id, m.type, m.action = ns, i, M.SPHERE if ns == 'tf_debug' else M.ARROW, M.ADD
        stamp(m.header)
        a = t + i
        m.pose.position.x = math.cos(a)
        m.pose.position.y = math.sin(a)
        m.pose.position.z = 0.2 * i
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.15
        if m.type == M.ARROW:
            m.scale.x, m.scale.y, m.scale.z = 0.4, 0.05, 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2 * i, 0.8, 0.3, 0.9
    return arr


def make_log(t, i):
    level = (1, 2, 3, 4)[i % 4]
    now = time.time_ns()
    return json.dumps({
        'timestamp': {'sec': now // 1_000_000_000, 'nsec': now % 1_000_000_000},
        'level': level,
        'message': f'home_bot tick={i} t={t:.1f}',
        'name': ('nav', 'perception', 'control', 'system')[i % 4],
        'file': '18_tutorial_robot_all_data_sim.py',
        'line': 1,
    }, separators=(',', ':')).encode()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=10.0)
    args = ap.parse_args()

    desc = string_pb2.String(data=URDF.read_text(encoding='utf-8'))
    autolink.init('robot_all')
    n = autolink.Node('/autoviz/robot_all')

    w = {
        # sensors
        'rgb': n.create_writer('/camera/image_raw', image_pb2.Image, qos_depth=2),
        'depth': n.create_writer('/camera/depth/image_raw', image_pb2.Image, qos_depth=2),
        'scan': n.create_writer('/scan', laser_scan_pb2.LaserScan, qos_depth=2),
        'cloud': n.create_writer('/points', point_cloud2_pb2.PointCloud2, qos_depth=2),
        'imu': n.create_writer('/imu/data', imu_pb2.Imu, qos_depth=5),
        'gps': n.create_writer('/gps/fix', nav_sat_fix_pb2.NavSatFix, qos_depth=2),
        'batt': n.create_writer('/battery_state', battery_state_pb2.BatteryState, qos_depth=2),
        'us': n.create_writer('/ultrasonic/distance', range_pb2.Range, qos_depth=2),
        'js': n.create_writer('/joint_states', joint_state_pb2.JointState, qos_depth=5),
        'wrench': n.create_writer('/wrench', wrench_stamped_pb2.WrenchStamped, qos_depth=5),
        # localization
        'odom': n.create_writer('/odom', odometry_pb2.Odometry, qos_depth=5),
        'tf': n.create_writer('/tf', tf_message_pb2.TFMessage, qos_depth=10),
        'map': n.create_writer('/map', occupancy_grid_pb2.OccupancyGrid, qos_depth=1),
        'amcl': n.create_writer(
            '/amcl_pose', pose_with_covariance_stamped_pb2.PoseWithCovarianceStamped,
            qos_depth=2),
        'urdf': n.create_writer('/robot_description', string_pb2.String, qos_depth=1),
        # navigation
        'cmd': n.create_writer('/cmd_vel', twist_pb2.Twist, qos_depth=5),
        'plan': n.create_writer('/plan', path_pb2.Path, qos_depth=2),
        'local': n.create_writer('/local_plan', path_pb2.Path, qos_depth=2),
        'navst': n.create_writer(
            '/move_base/status', goal_status_array_pb2.GoalStatusArray, qos_depth=2),
        'wp': n.create_writer('/waypoint', pose_stamped_pb2.PoseStamped, qos_depth=2),
        # perception
        'det': n.create_writer(
            '/detected_objects', detection3d_array_pb2.Detection3DArray, qos_depth=2),
        'people': n.create_writer(
            '/people', detection3d_array_pb2.Detection3DArray, qos_depth=2),
        'sem': n.create_writer(
            '/semantic_map', occupancy_grid_pb2.OccupancyGrid, qos_depth=1),
        # control / interaction
        'joy': n.create_writer('/joy', joy_pb2.Joy, qos_depth=2),
        'asr': n.create_writer('/speech_recognition', string_pb2.String, qos_depth=2),
        'tts': n.create_writer('/tts', string_pb2.String, qos_depth=2),
        'grip': n.create_writer('/gripper/command', float64_pb2.Float64, qos_depth=2),
        # system
        'diag': n.create_writer(
            '/diagnostics', diagnostic_array_pb2.DiagnosticArray, qos_depth=2),
        'rstate': n.create_writer('/robot_state', string_pb2.String, qos_depth=2),
        'rosout': n.create_writer('/rosout', 'foxglove.Log', qos_depth=10),
        # task
        'task': n.create_writer('/task_status', string_pb2.String, qos_depth=2),
        'bt': n.create_writer('/behavior_tree/status', string_pb2.String, qos_depth=2),
        # debug
        'dbg_img': n.create_writer('/debug_image', image_pb2.Image, qos_depth=2),
        'cost': n.create_writer(
            '/costmap_debug', occupancy_grid_pb2.OccupancyGrid, qos_depth=1),
        'path_dbg': n.create_writer(
            '/path_debug', marker_array_pb2.MarkerArray, qos_depth=2),
        'tf_dbg': n.create_writer(
            '/tf_debug', marker_array_pb2.MarkerArray, qos_depth=2),
    }

    rate = autolink.Rate(args.rate)
    print(f'robot_all @ {args.rate} Hz — home-service topic set')

    t0, phase, i = time.time(), 0, 0
    try:
        while not autolink.is_shutdown():
            t = time.time() - t0
            task = TASKS[int(t / 4) % len(TASKS)]

            w['rgb'].write(make_rgb(phase))
            w['depth'].write(make_depth())
            w['scan'].write(make_scan(t))
            w['cloud'].write(make_pc2(t))
            w['imu'].write(make_imu(t))
            w['gps'].write(make_gps(t))
            w['batt'].write(make_battery(t))
            w['us'].write(make_range(t))
            w['js'].write(make_js(t))
            w['wrench'].write(make_wrench(t))

            w['odom'].write(make_odom(t))
            w['tf'].write(make_tf(t))
            w['amcl'].write(make_amcl(t))
            w['urdf'].write(desc)
            if i % 20 == 0:
                w['map'].write(make_grid(0))
                w['sem'].write(make_grid(3))
                w['cost'].write(make_grid(5))

            w['cmd'].write(make_cmd(t))
            w['plan'].write(make_path(t, local=False))
            w['local'].write(make_path(t, n=12, local=True))
            w['navst'].write(make_nav_status(t))
            w['wp'].write(make_waypoint(t))

            w['det'].write(make_detections3d(t, people=False))
            w['people'].write(make_detections3d(t, people=True))

            w['joy'].write(make_joy(t))
            w['asr'].write(string_pb2.String(data='go to kitchen' if i % 30 < 3 else ''))
            w['tts'].write(string_pb2.String(data=f'current task {task}'))
            w['grip'].write(float64_pb2.Float64(data=0.5 + 0.5 * math.sin(t)))

            w['diag'].write(make_diagnostics(t))
            w['rstate'].write(string_pb2.String(data=json.dumps({
                'cpu_temp_c': 45 + 5 * math.sin(t * 0.2),
                'uptime_s': int(t),
                'task': task,
            }, separators=(',', ':'))))
            w['rosout'].write(make_log(t, i))

            w['task'].write(string_pb2.String(data=task))
            w['bt'].write(string_pb2.String(
                data=f'NavigateToPose:{task}|CheckBattery:OK'))

            w['dbg_img'].write(make_rgb(phase + 40, 'camera'))
            w['path_dbg'].write(make_markers(t, 'path_debug'))
            w['tf_dbg'].write(make_markers(t, 'tf_debug'))

            phase = (phase + 3) % 256
            i += 1
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
