"""Synthetic BICMap example messages for autoviz live preview (mirrors C++ bicmap_message_fixtures)."""

from __future__ import annotations

import math
import struct
import time
from typing import Iterable

from automsgs.msgs.nav_msgs import occupancy_grid_pb2, path_pb2
from automsgs.msgs.sensor_msgs import point_cloud2_pb2, point_field_pb2
from automsgs.msgs.std_msgs import header_pb2
from automsgs.msgs.strata_msgs import (
    canvas_label_pb2,
    iot_bubble_pb2,
    label_bubble_pb2,
    poi_marker_pb2,
    robot_3d_layer_pb2,
    robot_marker_pb2,
    road_graph_pb2,
    semantic_zone_pb2,
)
from automsgs.msgs.visualization_msgs import marker_array_pb2, marker_pb2

FRAME_ID = "map"


def make_header(frame_id: str = FRAME_ID, stamp_ns: int | None = None) -> header_pb2.Header:
    header = header_pb2.Header()
    header.frame_id = frame_id
    if stamp_ns is None:
        stamp_ns = time.time_ns()
    header.stamp.sec = stamp_ns // 1_000_000_000
    header.stamp.nanosec = stamp_ns % 1_000_000_000
    return header


def make_occupancy_grid(
    width: int,
    height: int,
    resolution: float = 0.05,
    fill_value: int = 100,
) -> occupancy_grid_pb2.OccupancyGrid:
    grid = occupancy_grid_pb2.OccupancyGrid()
    grid.header.CopyFrom(make_header())
    grid.info.resolution = resolution
    grid.info.width = width
    grid.info.height = height
    grid.info.origin.position.x = -1.0
    grid.info.origin.position.y = -1.0
    grid.data.extend([fill_value] * (width * height))
    for y in range(height):
        for x in range(width):
            idx = y * width + x
            if x == 0 or y == 0 or x + 1 == width or y + 1 == height:
                grid.data[idx] = 100
            elif (x + y) % 7 == 0:
                grid.data[idx] = -1
            else:
                grid.data[idx] = 0
    return grid


def make_poi_markers(count: int, long_labels: bool = False) -> poi_marker_pb2.PoiMarkerArray:
    array = poi_marker_pb2.PoiMarkerArray()
    array.header.CopyFrom(make_header())
    for i in range(count):
        poi = array.markers.add()
        poi.header.CopyFrom(make_header())
        poi.id = f"poi_{i}"
        poi.lng_lat.x = 1.0 + 0.5 * i
        poi.lng_lat.y = 0.5 * i
        poi.rotation_deg = 45.0 * i
        poi.name = (
            f"高级标注点位-{i}-BICMap" if long_labels else f"POI-{i}"
        )
    return array


def make_robot_markers(count: int, status: str = "running") -> robot_marker_pb2.RobotMarkerArray:
    array = robot_marker_pb2.RobotMarkerArray()
    array.header.CopyFrom(make_header())
    for i in range(count):
        robot = array.robots.add()
        robot.header.CopyFrom(make_header())
        robot.id = f"robot_{i}"
        robot.lng_lat.x = 0.5 + 0.3 * i
        robot.lng_lat.y = 0.2 * i
        robot.rotation_deg = 90.0
        robot.name = f"Robot-{i}"
        robot.status = status
        robot.battery = 80.0 - 5.0 * i
    return array


def make_semantic_zones(zone_type: str = "forbidden") -> semantic_zone_pb2.SemanticZoneArray:
    array = semantic_zone_pb2.SemanticZoneArray()
    array.header.CopyFrom(make_header())
    zone = array.zones.add()
    zone.header.CopyFrom(make_header())
    zone.id = "zone_0"
    zone.zone_type = zone_type
    zone.label = f"{zone_type}_area"
    zone.fill_color.r = 0.2
    zone.fill_color.g = 0.6
    zone.fill_color.b = 1.0
    zone.fill_color.a = 0.35
    zone.outline_color.r = 0.1
    zone.outline_color.g = 0.4
    zone.outline_color.b = 0.9
    zone.outline_color.a = 1.0
    zone.fill_opacity = 0.35
    zone.outline_width = 0.03
    for x, y in ((0.0, 0.0), (2.0, 0.0), (2.0, 1.5), (0.0, 1.5)):
        pt = zone.polygon.add()
        pt.x = x
        pt.y = y
    return array


def make_path(point_count: int) -> path_pb2.Path:
    path = path_pb2.Path()
    path.header.CopyFrom(make_header())
    for i in range(point_count):
        pose = path.poses.add()
        pose.header.CopyFrom(make_header())
        pose.pose.position.x = 0.2 * i
        pose.pose.position.y = 0.1 * i
        pose.pose.position.z = 0.01
        pose.pose.orientation.w = 1.0
    return path


def make_point_cloud2(point_count: int) -> point_cloud2_pb2.PointCloud2:
    cloud = point_cloud2_pb2.PointCloud2()
    cloud.header.CopyFrom(make_header())
    cloud.height = 1
    cloud.width = point_count
    cloud.is_bigendian = False
    cloud.is_dense = True
    cloud.point_step = 12
    cloud.row_step = 12 * point_count
    for name, offset in (("x", 0), ("y", 4), ("z", 8)):
        field = cloud.fields.add()
        field.name = name
        field.offset = offset
        field.datatype = point_field_pb2.PointField.FLOAT32
        field.count = 1
    buf = bytearray()
    for i in range(point_count):
        coords = (
            float(i) * 0.05,
            math.sin(float(i) * 0.2),
            0.05 * float(i % 5),
        )
        buf.extend(struct.pack("<fff", *coords))
    cloud.data = bytes(buf)
    return cloud


def _make_marker(
    marker_type: int,
    marker_id: int,
    ns: str = "bicmap",
) -> marker_pb2.Marker:
    marker = marker_pb2.Marker()
    marker.header.CopyFrom(make_header())
    marker.ns = ns
    marker.id = marker_id
    marker.type = marker_type
    marker.action = marker_pb2.Marker.ADD
    marker.color.r = 0.2
    marker.color.g = 0.7
    marker.color.b = 0.9
    marker.color.a = 0.9
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.pose.position.x = 1.0
    marker.pose.position.y = 0.5
    marker.pose.orientation.w = 1.0
    return marker


def make_line_strip_marker(marker_id: int, point_count: int) -> marker_array_pb2.MarkerArray:
    array = marker_array_pb2.MarkerArray()
    marker = array.markers.add()
    marker.CopyFrom(_make_marker(marker_pb2.Marker.LINE_STRIP, marker_id))
    marker.scale.x = 0.03
    for i in range(point_count):
        pt = marker.points.add()
        pt.x = 0.2 * i
        pt.y = 0.1 * math.sin(0.5 * i)
    return array


def make_rectangle_marker(marker_id: int) -> marker_array_pb2.MarkerArray:
    array = marker_array_pb2.MarkerArray()
    marker = array.markers.add()
    marker.CopyFrom(_make_marker(marker_pb2.Marker.LINE_LIST, marker_id))
    marker.scale.x = 0.02
    corners = [(0, 0), (1, 0), (1, 0.8), (0, 0.8), (0, 0), (1, 0)]
    for i in range(len(corners) - 1):
        a = marker.points.add()
        a.x, a.y = corners[i]
        b = marker.points.add()
        b.x, b.y = corners[i + 1]
    return array


def make_circle_marker(marker_id: int, segments: int = 24) -> marker_array_pb2.MarkerArray:
    array = marker_array_pb2.MarkerArray()
    marker = array.markers.add()
    marker.CopyFrom(_make_marker(marker_pb2.Marker.LINE_STRIP, marker_id))
    marker.scale.x = 0.02
    radius = 0.6
    for i in range(segments + 1):
        theta = 2.0 * math.pi * i / segments
        pt = marker.points.add()
        pt.x = radius * math.cos(theta)
        pt.y = radius * math.sin(theta)
    return array


def make_cube_marker(marker_id: int) -> marker_array_pb2.MarkerArray:
    array = marker_array_pb2.MarkerArray()
    marker = array.markers.add()
    marker.CopyFrom(_make_marker(marker_pb2.Marker.CUBE, marker_id))
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.8
    return array


def make_building_markers() -> marker_array_pb2.MarkerArray:
    array = marker_array_pb2.MarkerArray()
    for i in range(3):
        marker = array.markers.add()
        marker.CopyFrom(_make_marker(marker_pb2.Marker.CUBE, i, "outdoor_building"))
        marker.pose.position.x = 2.0 * i
        marker.pose.position.y = 1.0
        marker.scale.x = 1.2
        marker.scale.y = 0.8
        marker.scale.z = 3.0 + i
    return array


def make_space_markers() -> marker_array_pb2.MarkerArray:
    array = marker_array_pb2.MarkerArray()
    marker = array.markers.add()
    marker.CopyFrom(_make_marker(marker_pb2.Marker.SPHERE_LIST, 0, "space"))
    marker.scale.x = 0.15
    marker.scale.y = 0.15
    marker.scale.z = 0.15
    for i in range(12):
        pt = marker.points.add()
        pt.x = 0.1 * (i % 4)
        pt.y = 0.1 * (i // 4)
        pt.z = 0.05 * (i % 3)
    return array


def make_robot_fov_marker() -> marker_array_pb2.MarkerArray:
    array = marker_array_pb2.MarkerArray()
    marker = array.markers.add()
    marker.CopyFrom(_make_marker(marker_pb2.Marker.LINE_STRIP, 0, "strata_robot_fov"))
    marker.color.r = 0.2
    marker.color.g = 0.8
    marker.color.b = 1.0
    marker.color.a = 0.35
    for x, y in ((0, 0), (1.2, 0.4), (1.2, -0.4), (0, 0)):
        pt = marker.points.add()
        pt.x = x
        pt.y = y
    return array


def make_road_graph() -> road_graph_pb2.RoadGraph:
    graph = road_graph_pb2.RoadGraph()
    graph.header.CopyFrom(make_header())
    nodes = [("n0", 0.0, 0.0), ("n1", 3.0, 1.0), ("n2", 5.0, -0.5)]
    for node_id, x, y in nodes:
        node = graph.nodes.add()
        node.id = node_id
        node.coordinates.x = x
        node.coordinates.y = y
    e0 = graph.edges.add()
    e0.id = "e0"
    setattr(e0, "from", "n0")
    e0.to = "n1"
    e0.weight = 1.0
    e1 = graph.edges.add()
    e1.id = "e1"
    setattr(e1, "from", "n1")
    e1.to = "n2"
    e1.weight = 1.5
    return graph


def make_graphic_drawing_markers() -> marker_array_pb2.MarkerArray:
    array = marker_array_pb2.MarkerArray()
    array.markers.extend(make_line_strip_marker(0, 4).markers)
    array.markers.extend(make_cube_marker(1).markers)
    text = array.markers.add()
    text.CopyFrom(_make_marker(marker_pb2.Marker.TEXT_VIEW_FACING, 2))
    text.text = "BICMap"
    return array


def make_canvas_labels(count: int = 2) -> canvas_label_pb2.CanvasLabelArray:
    array = canvas_label_pb2.CanvasLabelArray()
    array.header.CopyFrom(make_header())
    for i in range(count):
        label = array.labels.add()
        label.header.CopyFrom(make_header())
        label.id = f"canvas_{i}"
        label.position.x = 1.0 + 0.4 * i
        label.position.y = 0.5 * i
        label.label = f"Canvas-{i}"
        label.visible = True
    return array


def make_label_bubbles(count: int = 2) -> label_bubble_pb2.LabelBubbleArray:
    array = label_bubble_pb2.LabelBubbleArray()
    array.header.CopyFrom(make_header())
    for i in range(count):
        bubble = array.bubbles.add()
        bubble.header.CopyFrom(make_header())
        bubble.lng_lat.x = 2.0 + 0.3 * i
        bubble.lng_lat.y = 0.8 + 0.2 * i
        bubble.html = f"<b>Bubble-{i}</b>"
        bubble.offset_x = 12.0 * i
        bubble.offset_y = -8.0
        bubble.visible = True
    return array


def make_iot_bubbles(count: int = 2) -> iot_bubble_pb2.IotBubbleArray:
    array = iot_bubble_pb2.IotBubbleArray()
    array.header.CopyFrom(make_header())
    future_ms = 4_000_000_000_000
    for i in range(count):
        bubble = array.bubbles.add()
        bubble.header.CopyFrom(make_header())
        bubble.id = f"iot_{i}"
        bubble.event_type = "warning" if i == 0 else "door_open"
        bubble.position.x = 3.0 + 0.2 * i
        bubble.position.y = 1.0
        bubble.message = f"IoT event {i}"
        bubble.expire_at_ms = future_ms
        bubble.visible = True
    return array


def make_robot_3d_layers(count: int = 1) -> robot_3d_layer_pb2.Robot3DLayerArray:
    array = robot_3d_layer_pb2.Robot3DLayerArray()
    array.header.CopyFrom(make_header())
    for i in range(count):
        layer = array.layers.add()
        layer.header.CopyFrom(make_header())
        layer.id = f"r3d_{i}"
        layer.robot_id = f"robot_{i}"
        layer.model_url = "/assets/models/guide-bot.glb"
        layer.position.x = 0.5 + 0.4 * i
        layer.position.y = 0.3
        layer.heading_deg = 45.0
        layer.status = "running"
        layer.scale = 1.0
        layer.visible = True
    return array


def merge_marker_arrays(*arrays: marker_array_pb2.MarkerArray) -> marker_array_pb2.MarkerArray:
    merged = marker_array_pb2.MarkerArray()
    for array in arrays:
        merged.markers.extend(array.markers)
    return merged
