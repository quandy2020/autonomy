#!/usr/bin/env python3
"""Publish BICMap router examples to /strata/* for autoviz live preview.

Usage:
  publish_bicmap_example.py --list
  publish_bicmap_example.py --example Slam
  publish_bicmap_example.py --cycle --interval 8

Requires automsgs + autolink on PYTHONPATH (same as publish_test_sensors.py).
Load config/default.autoviz in autoviz with Fixed Frame=map.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

# Bootstrap autolink + automsgs (same pattern as autolink/examples/python).
def _setup_pythonpath() -> None:
    candidates = [
        Path("/usr/local/python"),
        Path("/usr/local/lib/autolink/python/internal"),
    ]
    here = SCRIPT_DIR
    for _ in range(6):
        candidates.extend(
            [
                here / "build" / "automsgs" / "proto" / "gen" / "python",
                here / "build" / "autonomy" / "automsgs" / "proto" / "gen" / "python",
                here / "build" / "python",
                here / "build" / "lib" / "autolink" / "python" / "internal",
            ]
        )
        if (here / "src" / "autonomy").exists():
            candidates.append(here / "src" / "autonomy")
            break
        if here.parent == here:
            break
        here = here.parent
    for path in candidates:
        if path.exists() and str(path) not in sys.path:
            sys.path.insert(0, str(path))


_setup_pythonpath()

try:
    from autolink_py3 import autolink
except ModuleNotFoundError:
    try:
        from autolink.python.autolink_py3 import autolink  # type: ignore
    except ModuleNotFoundError as exc:
        print("autolink Python bindings not found.", file=sys.stderr)
        raise SystemExit(1) from exc

try:
    from automsgs.msgs.nav_msgs import occupancy_grid_pb2, path_pb2
    from automsgs.msgs.sensor_msgs import point_cloud2_pb2
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
    from automsgs.msgs.visualization_msgs import marker_array_pb2
except ImportError as exc:
    print("Run from workspace with automsgs + autolink on PYTHONPATH.", file=sys.stderr)
    raise SystemExit(1) from exc

import bicmap_fixtures as fixtures


@dataclass
class ExampleScene:
    name: str
    route: str
    title: str
    map: occupancy_grid_pb2.OccupancyGrid | None = None
    poi: poi_marker_pb2.PoiMarkerArray | None = None
    robots: robot_marker_pb2.RobotMarkerArray | None = None
    zones: semantic_zone_pb2.SemanticZoneArray | None = None
    path: path_pb2.Path | None = None
    pointcloud: point_cloud2_pb2.PointCloud2 | None = None
    markers: marker_array_pb2.MarkerArray | None = None
    road_graph: road_graph_pb2.RoadGraph | None = None
    canvas_labels: canvas_label_pb2.CanvasLabelArray | None = None
    label_bubbles: label_bubble_pb2.LabelBubbleArray | None = None
    iot_bubbles: iot_bubble_pb2.IotBubbleArray | None = None
    robot_3d: robot_3d_layer_pb2.Robot3DLayerArray | None = None
    animate_location: bool = False


def _all_examples() -> dict[str, ExampleScene]:
    scenes: list[ExampleScene] = [
        ExampleScene("Slam", "/indoor/slam", "SLAM地图显示示例",
                     map=fixtures.make_occupancy_grid(32, 24)),
        ExampleScene("BuildMap", "/indoor/buildMap", "机器人建图",
                     map=fixtures.make_occupancy_grid(48, 48)),
        ExampleScene("RobotFollow", "/indoor/robotFollow", "机器人视角跟随",
                     map=fixtures.make_occupancy_grid(32, 24),
                     robots=fixtures.make_robot_markers(1),
                     markers=fixtures.make_robot_fov_marker()),
        ExampleScene("MapTools", "/base/mapTools", "地图工具",
                     map=fixtures.make_occupancy_grid(64, 64)),
        ExampleScene("POIMarkers", "/base/POIMarkers", "点位Marker",
                     map=fixtures.make_occupancy_grid(32, 24),
                     poi=fixtures.make_poi_markers(4)),
        ExampleScene("POIadvancedLabel", "/expand/POIadvancedLabel", "POI高级标注",
                     map=fixtures.make_occupancy_grid(32, 24),
                     poi=fixtures.make_poi_markers(3, long_labels=True),
                     label_bubbles=fixtures.make_label_bubbles(3),
                     canvas_labels=fixtures.make_canvas_labels(2)),
        ExampleScene("Relocate", "/indoor/relocate", "机器人重定位",
                     map=fixtures.make_occupancy_grid(32, 24),
                     robots=fixtures.make_robot_markers(1, status="idle")),
        ExampleScene("Location", "/indoor/location", "实时位置更新",
                     map=fixtures.make_occupancy_grid(32, 24),
                     robots=fixtures.make_robot_markers(1),
                     animate_location=True),
        ExampleScene("SingleNavigation", "/indoor/singleNavigation", "单点导航",
                     map=fixtures.make_occupancy_grid(32, 24),
                     path=fixtures.make_path(8),
                     robots=fixtures.make_robot_markers(1)),
        ExampleScene("load3D", "/indoor/load3D", "加载3D模型",
                     map=fixtures.make_occupancy_grid(32, 24),
                     markers=fixtures.make_cube_marker(0),
                     robot_3d=fixtures.make_robot_3d_layers(1)),
        ExampleScene("load3dControl", "/indoor/load3dControl", "3D模型控制",
                     map=fixtures.make_occupancy_grid(32, 24),
                     markers=fixtures.make_cube_marker(0),
                     robot_3d=fixtures.make_robot_3d_layers(1)),
        ExampleScene("PointCloud", "/indoor/pointCloud", "二三维点云示例",
                     map=fixtures.make_occupancy_grid(32, 24),
                     pointcloud=fixtures.make_point_cloud2(200)),
        ExampleScene("SemanticMap", "/indoor/semanticMap", "语义地图分割",
                     map=fixtures.make_occupancy_grid(32, 24),
                     zones=fixtures.make_semantic_zones("activity_area")),
        ExampleScene("Space", "/indoor/load3dMarker", "三维空间数据渲染",
                     map=fixtures.make_occupancy_grid(32, 24),
                     markers=fixtures.make_space_markers()),
        ExampleScene("RobotGuideTour", "/scene/robotGuideTour", "博物馆导览",
                     map=fixtures.make_occupancy_grid(40, 30),
                     poi=fixtures.make_poi_markers(5),
                     path=fixtures.make_path(10),
                     robots=fixtures.make_robot_markers(1),
                     canvas_labels=fixtures.make_canvas_labels(3)),
        ExampleScene("HotelDelivery", "/scene/hotelDelivery", "酒店配送",
                     map=fixtures.make_occupancy_grid(40, 30),
                     path=fixtures.make_path(6),
                     robots=fixtures.make_robot_markers(1),
                     iot_bubbles=fixtures.make_iot_bubbles(2)),
        ExampleScene("MallRobotMonitor", "/scene/mallRobotMonitor", "商场监控",
                     map=fixtures.make_occupancy_grid(40, 30),
                     robots=fixtures.make_robot_markers(2),
                     poi=fixtures.make_poi_markers(3),
                     markers=fixtures.make_robot_fov_marker()),
        ExampleScene("IndoorCleaning", "/scene/indoorCleaning", "扫地清扫",
                     map=fixtures.make_occupancy_grid(40, 30),
                     zones=fixtures.make_semantic_zones("waiting"),
                     path=fixtures.make_path(12),
                     robots=fixtures.make_robot_markers(1)),
        ExampleScene("CommunityInspect", "/scene/communityInspect", "社区巡检",
                     map=fixtures.make_occupancy_grid(40, 30),
                     robots=fixtures.make_robot_markers(1),
                     path=fixtures.make_path(7),
                     poi=fixtures.make_poi_markers(2)),
        ExampleScene("PolylineDrawing", "/base/PolylineDrawing", "折线绘制",
                     map=fixtures.make_occupancy_grid(32, 24),
                     markers=fixtures.make_line_strip_marker(0, 6)),
        ExampleScene("RectangleDraw", "/base/RectangleDraw", "矩形绘制",
                     map=fixtures.make_occupancy_grid(32, 24),
                     markers=fixtures.make_rectangle_marker(0)),
        ExampleScene("PolygonDraw", "/base/PolygonDraw", "多边形绘制",
                     map=fixtures.make_occupancy_grid(32, 24),
                     markers=fixtures.make_line_strip_marker(0, 5)),
        ExampleScene("CircleDraw", "/base/CircleDraw", "圆形绘制",
                     map=fixtures.make_occupancy_grid(32, 24),
                     markers=fixtures.make_circle_marker(0)),
        ExampleScene("PathReplay", "/base/PathReplay", "路径回放",
                     map=fixtures.make_occupancy_grid(32, 24),
                     path=fixtures.make_path(20)),
        ExampleScene("PathPlanning", "/base/PathPlanning", "路径规划",
                     map=fixtures.make_occupancy_grid(40, 30),
                     path=fixtures.make_path(9)),
        ExampleScene("MapEditor", "/expand/MapEditor", "地图编辑",
                     map=fixtures.make_occupancy_grid(20, 20)),
        ExampleScene("GraphicDrawing", "/expand/GraphicDrawing", "图形绘制",
                     map=fixtures.make_occupancy_grid(32, 24),
                     markers=fixtures.make_graphic_drawing_markers()),
        ExampleScene("PassableArea", "/base/passableArea", "可通行区域",
                     map=fixtures.make_occupancy_grid(32, 24),
                     zones=fixtures.make_semantic_zones("activity_area")),
        ExampleScene("EditPassableArea", "/base/editPassableArea", "可通行区域编辑",
                     map=fixtures.make_occupancy_grid(32, 24),
                     zones=fixtures.make_semantic_zones("speed_limit")),
        ExampleScene("OutdoorBuildings", "/outdoor/buildings", "室外建筑物",
                     map=fixtures.make_occupancy_grid(64, 64, resolution=0.1),
                     markers=fixtures.make_building_markers()),
        ExampleScene("OutdoorPointCloud", "/outdoor/pointCloud", "室外点云",
                     map=fixtures.make_occupancy_grid(64, 64, resolution=0.1),
                     pointcloud=fixtures.make_point_cloud2(800)),
        ExampleScene("TransportHub", "/scene/transportHub", "三站一场引导",
                     map=fixtures.make_occupancy_grid(48, 36),
                     road_graph=fixtures.make_road_graph(),
                     poi=fixtures.make_poi_markers(4, long_labels=True),
                     label_bubbles=fixtures.make_label_bubbles(2)),
        ExampleScene("PublicClean", "/scene/publicClean", "公共区域清洁",
                     map=fixtures.make_occupancy_grid(40, 30),
                     zones=fixtures.make_semantic_zones("forbidden"),
                     robots=fixtures.make_robot_markers(1)),
        ExampleScene("OutdoorMapTiles", "/outdoor/mapTiles", "地图瓦片加载",
                     map=fixtures.make_occupancy_grid(128, 128, resolution=0.1)),
        ExampleScene("OutdoorHdMap", "/outdoor/hdMap", "高精地图加载",
                     map=fixtures.make_occupancy_grid(64, 64, resolution=0.1),
                     road_graph=fixtures.make_road_graph()),
        ExampleScene("SecurityPatrol", "/scene/securityPatrol", "园区安防巡逻",
                     map=fixtures.make_occupancy_grid(48, 36),
                     zones=fixtures.make_semantic_zones("forbidden"),
                     path=fixtures.make_path(15),
                     robots=fixtures.make_robot_markers(1),
                     markers=fixtures.make_robot_fov_marker()),
    ]
    return {scene.name: scene for scene in scenes}


EXAMPLES = _all_examples()


class Publisher:
    def __init__(self, prefix: str = "/strata") -> None:
        autolink.init("bicmap_example_publisher")
        self._node = autolink.Node("bicmap_example_publisher")
        self._writers = {
            "map": self._node.create_writer(f"{prefix}/map", occupancy_grid_pb2.OccupancyGrid),
            "poi": self._node.create_writer(
                f"{prefix}/poi_markers", poi_marker_pb2.PoiMarkerArray
            ),
            "robots": self._node.create_writer(
                f"{prefix}/robot_markers", robot_marker_pb2.RobotMarkerArray
            ),
            "zones": self._node.create_writer(
                f"{prefix}/semantic_zones", semantic_zone_pb2.SemanticZoneArray
            ),
            "path": self._node.create_writer(f"{prefix}/path", path_pb2.Path),
            "pointcloud": self._node.create_writer(
                f"{prefix}/pointcloud", point_cloud2_pb2.PointCloud2
            ),
            "markers": self._node.create_writer(
                f"{prefix}/markers", marker_array_pb2.MarkerArray
            ),
            "road_graph": self._node.create_writer(
                f"{prefix}/road_graph", road_graph_pb2.RoadGraph
            ),
            "canvas_labels": self._node.create_writer(
                f"{prefix}/canvas_labels", canvas_label_pb2.CanvasLabelArray
            ),
            "label_bubbles": self._node.create_writer(
                f"{prefix}/label_bubbles", label_bubble_pb2.LabelBubbleArray
            ),
            "iot_bubbles": self._node.create_writer(
                f"{prefix}/iot_bubbles", iot_bubble_pb2.IotBubbleArray
            ),
            "robot_3d": self._node.create_writer(
                f"{prefix}/robot_3d_layers", robot_3d_layer_pb2.Robot3DLayerArray
            ),
        }

    def publish(self, scene: ExampleScene, tick: int = 0) -> None:
        if scene.map is not None:
            self._writers["map"].write(scene.map)
        if scene.poi is not None:
            self._writers["poi"].write(scene.poi)
        if scene.robots is not None:
            robots = robot_marker_pb2.RobotMarkerArray()
            robots.CopyFrom(scene.robots)
            if scene.animate_location and robots.robots:
                robots.robots[0].lng_lat.x = 0.5 + 0.2 * (tick % 10)
            self._writers["robots"].write(robots)
        if scene.zones is not None:
            self._writers["zones"].write(scene.zones)
        if scene.path is not None:
            self._writers["path"].write(scene.path)
        if scene.pointcloud is not None:
            self._writers["pointcloud"].write(scene.pointcloud)
        if scene.markers is not None:
            self._writers["markers"].write(scene.markers)
        if scene.road_graph is not None:
            self._writers["road_graph"].write(scene.road_graph)
        if scene.canvas_labels is not None:
            self._writers["canvas_labels"].write(scene.canvas_labels)
        if scene.label_bubbles is not None:
            self._writers["label_bubbles"].write(scene.label_bubbles)
        if scene.iot_bubbles is not None:
            self._writers["iot_bubbles"].write(scene.iot_bubbles)
        if scene.robot_3d is not None:
            self._writers["robot_3d"].write(scene.robot_3d)


def list_examples() -> None:
    print(f"BICMap examples ({len(EXAMPLES)}):")
    for name, scene in EXAMPLES.items():
        print(f"  {name:20s}  {scene.route:28s}  {scene.title}")


def resolve_example(name: str) -> ExampleScene:
    if name in EXAMPLES:
        return EXAMPLES[name]
    lower = name.lower()
    for key, scene in EXAMPLES.items():
        if key.lower() == lower:
            return scene
    matches = [key for key in EXAMPLES if lower in key.lower()]
    if len(matches) == 1:
        return EXAMPLES[matches[0]]
    raise KeyError(name)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="List all 36 BICMap examples.")
    parser.add_argument("--example", default="Slam", help="Example name (default: Slam).")
    parser.add_argument("--cycle", action="store_true", help="Rotate through all examples.")
    parser.add_argument("--interval", type=float, default=8.0, help="Seconds per example in cycle mode.")
    parser.add_argument("--rate", type=float, default=2.0, help="Publish rate in Hz.")
    parser.add_argument("--prefix", default="/strata", help="Topic prefix (default: /strata).")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.list:
        list_examples()
        return

    publisher = Publisher(prefix=args.prefix)
    sleep_s = max(0.05, 1.0 / args.rate)

    if args.cycle:
        names = list(EXAMPLES.keys())
        print(f"Cycling {len(names)} examples every {args.interval}s at {args.rate} Hz")
        print("autoviz: autoviz -c config/default.autoviz  (Fixed Frame=map)")
        index = 0
        tick = 0
        last_switch = time.monotonic()
        current = EXAMPLES[names[0]]
        print(f"-> {current.name} ({current.title})")
        while True:
            if time.monotonic() - last_switch >= args.interval:
                index = (index + 1) % len(names)
                current = EXAMPLES[names[index]]
                last_switch = time.monotonic()
                print(f"-> {current.name} ({current.title})")
            publisher.publish(current, tick=tick)
            tick += 1
            time.sleep(sleep_s)
    else:
        scene = resolve_example(args.example)
        print(f"Publishing BICMap example: {scene.name} — {scene.title}")
        print(f"Route: {scene.route}")
        print(f"Topics under {args.prefix}/* at {args.rate} Hz")
        print("autoviz: autoviz -c config/default.autoviz  (Fixed Frame=map)")
        tick = 0
        while True:
            publisher.publish(scene, tick=tick)
            tick += 1
            time.sleep(sleep_s)


if __name__ == "__main__":
    main()
