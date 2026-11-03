#!/usr/bin/env python3
"""Record honest full-run Atlas/AutoSim validation trajectories.

The primary sources are raw odometry streams, not cumulative Path snapshots:

* ``/atlas/odometry`` is the Atlas camera pose.
* ``/gt/odometry`` is the AutoSim base pose and is composed with a configurable
  base-to-camera rigid transform before being written.

Appending every odometry sample preserves Atlas epoch changes, timestamp
regressions, and spatial resets.  A latest Path snapshot is retained only as a
diagnostic because Atlas may clear or truncate Path when an epoch changes.

The default extrinsic matches the AutoSim TurtleBot URDF: base-to-camera-link
translation ``(0.03, 0, 0.20)`` metres followed by the REP-103 camera optical
rotation ``rpy=(-pi/2, 0, -pi/2)``, quaternion
``(-0.5, 0.5, -0.5, 0.5)`` in xyzw order.

Autolink and AutoMsgs imports are delayed so all conversion logic can be tested
with only the Python standard library.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import tempfile
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Sequence


NANOSECONDS_PER_SECOND = 1_000_000_000
ATLAS_ODOMETRY_TOPIC = "/atlas/odometry"
GROUND_TRUTH_ODOMETRY_TOPIC = "/gt/odometry"
ATLAS_PATH_TOPIC = "/atlas/trajectory"
GROUND_TRUTH_PATH_TOPIC = "/atlas/ground_truth_trajectory"
DENSE_MAP_TOPIC = "/atlas/dense_map_points"
OCCUPANCY_GRID_TOPIC = "/atlas/occupancy_grid"
DEFAULT_BASE_TO_CAMERA_XYZ = (0.03, 0.0, 0.20)
DEFAULT_BASE_TO_CAMERA_QUATERNION = (-0.5, 0.5, -0.5, 0.5)


@dataclass(frozen=True)
class TumPose:
    """One validated TUM pose with an exact integer timestamp."""

    timestamp_ns: int
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class RigidTransform:
    """Parent-to-child transform in parent coordinates."""

    translation: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


@dataclass
class OdometryTrajectory:
    """Every decoded odometry sample received during one run."""

    topic: str
    poses: list[TumPose] = field(default_factory=list)
    message_count: int = 0
    conversion_error_count: int = 0
    last_conversion_error: str | None = None
    parent_frame_id: str = ""
    child_frame_id: str = ""

    def append(
        self,
        message: Any,
        child_transform: RigidTransform | None = None,
    ) -> None:
        """Convert and append one message without hiding invalid samples."""
        self.message_count += 1
        self.parent_frame_id = str(
            getattr(getattr(message, "header", None), "frame_id", "")
        )
        self.child_frame_id = str(getattr(message, "child_frame_id", ""))
        try:
            self.poses.append(odometry_to_tum_pose(message, child_transform))
        except (AttributeError, TypeError, ValueError) as error:
            self.conversion_error_count += 1
            self.last_conversion_error = str(error)

    def summary(self, output_file: Path) -> dict[str, object]:
        timestamps = [pose.timestamp_ns for pose in self.poses]
        regressions = sum(
            current < previous
            for previous, current in zip(timestamps, timestamps[1:])
        )
        duplicates = len(timestamps) - len(set(timestamps))
        return {
            "topic": self.topic,
            "received": self.message_count > 0,
            "message_count": self.message_count,
            "pose_count": len(self.poses),
            "parent_frame_id": self.parent_frame_id,
            "child_frame_id": self.child_frame_id,
            "first_timestamp_ns": timestamps[0] if timestamps else None,
            "last_timestamp_ns": timestamps[-1] if timestamps else None,
            "timestamp_regression_count": regressions,
            "duplicate_timestamp_count": duplicates,
            "conversion_error_count": self.conversion_error_count,
            "last_conversion_error": self.last_conversion_error,
            "output_file": str(output_file),
            "valid": bool(self.poses) and self.conversion_error_count == 0,
        }


@dataclass
class PathSnapshot:
    """Latest cumulative Path retained only for diagnostics."""

    topic: str
    message_count: int = 0
    frame_id: str = ""
    pose_count: int = 0
    conversion_error: str | None = None

    def update(self, message: Any, messages_received: int = 1) -> None:
        self.message_count += messages_received
        try:
            pose_count = len(message.poses)
            frame_id = str(getattr(getattr(message, "header", None), "frame_id", ""))
        except (AttributeError, TypeError, ValueError) as error:
            self.conversion_error = str(error)
            return
        self.frame_id = frame_id
        self.pose_count = pose_count
        self.conversion_error = None

    def summary(self) -> dict[str, object]:
        return {
            "topic": self.topic,
            "received": self.message_count > 0,
            "message_count": self.message_count,
            "frame_id": self.frame_id,
            "latest_pose_count": self.pose_count,
            "conversion_error": self.conversion_error,
            "role": "diagnostic_only_not_used_for_evaluation",
        }


@dataclass(frozen=True)
class MessageTypes:
    """Runtime AutoMsgs classes used to bind readers."""

    odometry: Any
    path: Any
    point_cloud: Any
    occupancy_grid: Any


def stamp_to_nanoseconds(stamp: Any) -> int:
    """Convert an AutoMsgs stamp to exact integer nanoseconds."""
    try:
        seconds = int(stamp.sec)
        nanoseconds = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError) as error:
        raise ValueError("stamp must contain integer sec and nanosec") from error
    if nanoseconds < 0 or nanoseconds >= NANOSECONDS_PER_SECOND:
        raise ValueError("stamp nanosec is outside [0, 1000000000)")
    return seconds * NANOSECONDS_PER_SECOND + nanoseconds


def normalize_quaternion(values: Sequence[float]) -> tuple[float, float, float, float]:
    """Validate and normalize one xyzw quaternion."""
    if len(values) != 4:
        raise ValueError("quaternion must contain four values")
    quaternion = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in quaternion):
        raise ValueError("quaternion contains a non-finite value")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        raise ValueError("quaternion norm is zero")
    return tuple(value / norm for value in quaternion)


def quaternion_multiply(
    left: Sequence[float], right: Sequence[float]
) -> tuple[float, float, float, float]:
    """Return the xyzw Hamilton product ``left * right``."""
    lx, ly, lz, lw = normalize_quaternion(left)
    rx, ry, rz, rw = normalize_quaternion(right)
    return normalize_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def quaternion_rotate_vector(
    quaternion: Sequence[float], vector: Sequence[float]
) -> tuple[float, float, float]:
    """Rotate a 3-vector by an xyzw quaternion."""
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    vx, vy, vz = (float(value) for value in vector)
    if not all(math.isfinite(value) for value in (vx, vy, vz)):
        raise ValueError("translation contains a non-finite value")
    # R(q) * v, expanded to avoid constructing temporary quaternion objects.
    return (
        (1.0 - 2.0 * (qy * qy + qz * qz)) * vx
        + 2.0 * (qx * qy - qz * qw) * vy
        + 2.0 * (qx * qz + qy * qw) * vz,
        2.0 * (qx * qy + qz * qw) * vx
        + (1.0 - 2.0 * (qx * qx + qz * qz)) * vy
        + 2.0 * (qy * qz - qx * qw) * vz,
        2.0 * (qx * qz - qy * qw) * vx
        + 2.0 * (qy * qz + qx * qw) * vy
        + (1.0 - 2.0 * (qx * qx + qy * qy)) * vz,
    )


def compose_child_pose(
    parent_pose: TumPose, child_transform: RigidTransform
) -> TumPose:
    """Compose world-to-parent with parent-to-child at the same timestamp."""
    parent_quaternion = (
        parent_pose.qx,
        parent_pose.qy,
        parent_pose.qz,
        parent_pose.qw,
    )
    rotated_translation = quaternion_rotate_vector(
        parent_quaternion, child_transform.translation
    )
    child_quaternion = quaternion_multiply(
        parent_quaternion, child_transform.quaternion_xyzw
    )
    return TumPose(
        parent_pose.timestamp_ns,
        parent_pose.x + rotated_translation[0],
        parent_pose.y + rotated_translation[1],
        parent_pose.z + rotated_translation[2],
        *child_quaternion,
    )


def extract_pose(message: Any) -> Any:
    """Find Pose inside standard or AutoMsgs-stamped odometry nesting."""
    try:
        candidate = message.pose
    except AttributeError as error:
        raise ValueError("Odometry message has no pose field") from error
    for _ in range(3):
        if hasattr(candidate, "position") and hasattr(candidate, "orientation"):
            return candidate
        if not hasattr(candidate, "pose"):
            break
        candidate = candidate.pose
    raise ValueError("Odometry pose nesting does not contain a Pose")


def odometry_to_tum_pose(
    message: Any,
    child_transform: RigidTransform | None = None,
) -> TumPose:
    """Convert one Odometry and optionally compose its child extrinsic."""
    try:
        timestamp_ns = stamp_to_nanoseconds(message.header.stamp)
        pose = extract_pose(message)
        xyz = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        )
        quaternion = normalize_quaternion(
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        )
    except (AttributeError, TypeError, ValueError) as error:
        raise ValueError(f"invalid Odometry pose: {error}") from error
    if not all(math.isfinite(value) for value in xyz):
        raise ValueError("invalid Odometry pose: position is not finite")
    result = TumPose(timestamp_ns, *xyz, *quaternion)
    return compose_child_pose(result, child_transform) if child_transform else result


def path_to_tum_poses(message: Any) -> list[TumPose]:
    """Convert a Path-like protobuf while preserving pose order and stamps."""
    try:
        stamped_poses = message.poses
    except AttributeError as error:
        raise ValueError("Path message has no poses field") from error
    result = []
    for index, stamped in enumerate(stamped_poses):
        try:
            timestamp_ns = stamp_to_nanoseconds(stamped.header.stamp)
            pose = stamped.pose
            xyz = (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            )
            quaternion = normalize_quaternion(
                (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
            )
        except (AttributeError, TypeError, ValueError) as error:
            raise ValueError(f"Path pose {index} is invalid: {error}") from error
        if not all(math.isfinite(value) for value in xyz):
            raise ValueError(f"Path pose {index} position is not finite")
        result.append(TumPose(timestamp_ns, *xyz, *quaternion))
    return result


def format_timestamp_ns(timestamp_ns: int) -> str:
    """Format exact nanoseconds as decimal seconds for TUM."""
    sign = "-" if timestamp_ns < 0 else ""
    seconds, nanoseconds = divmod(abs(int(timestamp_ns)), NANOSECONDS_PER_SECOND)
    return f"{sign}{seconds}.{nanoseconds:09d}"


def tum_text(
    poses: Sequence[TumPose], topic: str, comments: Sequence[str] = ()
) -> str:
    """Serialize poses as a documented TUM trajectory."""
    lines = [
        "# TUM trajectory: timestamp tx ty tz qx qy qz qw\n",
        f"# Autolink topic: {topic}\n",
    ]
    lines.extend(f"# {comment}\n" for comment in comments)
    for pose in poses:
        lines.append(
            f"{format_timestamp_ns(pose.timestamp_ns)} "
            f"{pose.x:.12f} {pose.y:.12f} {pose.z:.12f} "
            f"{pose.qx:.12f} {pose.qy:.12f} {pose.qz:.12f} "
            f"{pose.qw:.12f}\n"
        )
    return "".join(lines)


def summarize_dense_map(message: Any | None) -> dict[str, object]:
    """Return structural diagnostics for the latest PointCloud2."""
    if message is None:
        return {
            "received": False,
            "width": 0,
            "height": 0,
            "point_count": 0,
            "point_step": 0,
            "data_byte_count": 0,
            "minimum_data_byte_count": 0,
            "valid": False,
        }
    width = max(0, int(getattr(message, "width", 0)))
    height = max(0, int(getattr(message, "height", 0)))
    point_step = max(0, int(getattr(message, "point_step", 0)))
    data_byte_count = len(getattr(message, "data", ()))
    point_count = width * height
    minimum_data_byte_count = point_count * point_step
    return {
        "received": True,
        "width": width,
        "height": height,
        "point_count": point_count,
        "point_step": point_step,
        "data_byte_count": data_byte_count,
        "minimum_data_byte_count": minimum_data_byte_count,
        "valid": (
            point_count > 0
            and point_step > 0
            and data_byte_count >= minimum_data_byte_count
        ),
    }


def summarize_occupancy_grid(message: Any | None) -> dict[str, object]:
    """Return dimensions and known/occupied-cell diagnostics for a grid."""
    if message is None:
        return {
            "received": False,
            "width": 0,
            "height": 0,
            "expected_cell_count": 0,
            "data_cell_count": 0,
            "known_cell_count": 0,
            "occupied_cell_count": 0,
            "unknown_cell_count": 0,
            "valid": False,
        }
    info = getattr(message, "info", None)
    width = max(0, int(getattr(info, "width", 0)))
    height = max(0, int(getattr(info, "height", 0)))
    data = tuple(int(value) for value in getattr(message, "data", ()))
    expected_cell_count = width * height
    known_cell_count = sum(value >= 0 for value in data)
    occupied_cell_count = sum(value > 0 for value in data)
    unknown_cell_count = sum(value < 0 for value in data)
    dimensions_and_data_match = (
        expected_cell_count > 0 and len(data) == expected_cell_count
    )
    return {
        "received": True,
        "width": width,
        "height": height,
        "expected_cell_count": expected_cell_count,
        "data_cell_count": len(data),
        "known_cell_count": known_cell_count,
        "occupied_cell_count": occupied_cell_count,
        "unknown_cell_count": unknown_cell_count,
        "valid": dimensions_and_data_match and known_cell_count > 0,
    }


def atomic_write_text(path: Path, contents: str) -> None:
    """Atomically replace one UTF-8 text file in its destination directory."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary_path = Path(stream.name)
            stream.write(contents)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    except BaseException:
        if temporary_path is not None:
            try:
                temporary_path.unlink(missing_ok=True)
            except OSError:
                pass
        raise


class AutosimRecorder:
    """Collect typed Autolink callbacks behind one snapshot lock."""

    def __init__(
        self,
        link: Any,
        message_types: MessageTypes,
        *,
        node_name: str,
        atlas_odometry_topic: str,
        ground_truth_odometry_topic: str,
        atlas_path_topic: str,
        ground_truth_path_topic: str,
        dense_map_topic: str,
        occupancy_grid_topic: str,
        base_to_camera: RigidTransform,
    ) -> None:
        self.link = link
        self.base_to_camera = base_to_camera
        self.atlas_odometry = OdometryTrajectory(atlas_odometry_topic)
        self.ground_truth_odometry = OdometryTrajectory(
            ground_truth_odometry_topic
        )
        self.atlas_path = PathSnapshot(atlas_path_topic)
        self.ground_truth_path = PathSnapshot(ground_truth_path_topic)
        self.dense_map_topic = dense_map_topic
        self.occupancy_grid_topic = occupancy_grid_topic
        self.latest_dense_map: Any | None = None
        self.latest_occupancy_grid: Any | None = None
        self.dense_map_message_count = 0
        self.occupancy_grid_message_count = 0
        self._lock = threading.Lock()
        self._accept_messages = True

        self.link.init(node_name)
        self.node = self.link.Node(node_name)

        def on_atlas_odometry(message: Any) -> None:
            with self._lock:
                if self._accept_messages:
                    self.atlas_odometry.append(message)

        def on_ground_truth_odometry(message: Any) -> None:
            with self._lock:
                if self._accept_messages:
                    self.ground_truth_odometry.append(
                        message, self.base_to_camera
                    )

        def on_atlas_path(message: Any) -> None:
            with self._lock:
                if self._accept_messages:
                    self.atlas_path.update(message)

        def on_ground_truth_path(message: Any) -> None:
            with self._lock:
                if self._accept_messages:
                    self.ground_truth_path.update(message)

        def on_dense_map(message: Any) -> None:
            with self._lock:
                if self._accept_messages:
                    self.latest_dense_map = message
                    self.dense_map_message_count += 1

        def on_occupancy_grid(message: Any) -> None:
            with self._lock:
                if self._accept_messages:
                    self.latest_occupancy_grid = message
                    self.occupancy_grid_message_count += 1

        # Keep reader handles alive for the lifetime of this recorder.  The
        # Python Autolink API is callback based: channel, callback, type.
        self.readers = [
            self.node.create_reader(
                atlas_odometry_topic,
                on_atlas_odometry,
                message_types.odometry,
            ),
            self.node.create_reader(
                ground_truth_odometry_topic,
                on_ground_truth_odometry,
                message_types.odometry,
            ),
            self.node.create_reader(
                atlas_path_topic, on_atlas_path, message_types.path
            ),
            self.node.create_reader(
                ground_truth_path_topic,
                on_ground_truth_path,
                message_types.path,
            ),
            self.node.create_reader(
                dense_map_topic, on_dense_map, message_types.point_cloud
            ),
            self.node.create_reader(
                occupancy_grid_topic,
                on_occupancy_grid,
                message_types.occupancy_grid,
            ),
        ]

    def poll_once(self) -> None:
        """Compatibility hook; Autolink callbacks perform all collection."""

    def freeze(self) -> None:
        """Stop accepting callbacks and wait for an in-flight callback."""
        with self._lock:
            self._accept_messages = False

    def shutdown(self) -> None:
        """Release the node and stop the process-local Autolink runtime."""
        try:
            if hasattr(self.node, "shutdown"):
                self.node.shutdown()
        finally:
            self.link.shutdown()


def load_runtime() -> tuple[Any, MessageTypes]:
    """Import Autolink and AutoMsgs only inside the autonomy environment."""
    try:
        import autolink
        from automsgs.msgs.map_msgs.occupancy_grid_pb2 import OccupancyGrid
        from automsgs.msgs.nav_msgs.odometry_pb2 import Odometry
        from automsgs.msgs.nav_msgs.path_pb2 import Path as PathMessage
        from automsgs.msgs.sensor_msgs.point_cloud2_pb2 import PointCloud2
    except ImportError as error:
        raise RuntimeError(
            "Autolink/AutoMsgs Python modules are unavailable; source "
            "install/setup.bash and run with the autonomy runtime Python"
        ) from error
    return autolink, MessageTypes(
        Odometry, PathMessage, PointCloud2, OccupancyGrid
    )


def transform_summary(transform: RigidTransform) -> dict[str, object]:
    return {
        "translation_m": list(transform.translation),
        "quaternion_xyzw": list(transform.quaternion_xyzw),
        "meaning": "base_to_camera_optical",
    }


def build_summary(
    recorder: AutosimRecorder,
    *,
    output_dir: Path,
    require_maps: bool,
    requested_duration_sec: float,
    elapsed_sec: float,
    exit_reason: str,
) -> dict[str, object]:
    """Build the final validation summary and requirement failures."""
    atlas_output = output_dir / "atlas_trajectory.tum"
    ground_truth_output = output_dir / "ground_truth_trajectory.tum"
    atlas_summary = recorder.atlas_odometry.summary(atlas_output)
    ground_truth_summary = recorder.ground_truth_odometry.summary(
        ground_truth_output
    )
    dense_summary = summarize_dense_map(recorder.latest_dense_map)
    dense_summary["topic"] = recorder.dense_map_topic
    dense_summary["message_count"] = recorder.dense_map_message_count
    occupancy_summary = summarize_occupancy_grid(
        recorder.latest_occupancy_grid
    )
    occupancy_summary["topic"] = recorder.occupancy_grid_topic
    occupancy_summary["message_count"] = recorder.occupancy_grid_message_count

    errors = []
    if not atlas_summary["valid"]:
        errors.append("missing, empty, or invalid Atlas odometry stream")
    if not ground_truth_summary["valid"]:
        errors.append("missing, empty, or invalid ground-truth odometry stream")
    if require_maps and not dense_summary["valid"]:
        errors.append("missing or empty dense map PointCloud2")
    if require_maps and not occupancy_summary["valid"]:
        errors.append("missing or invalid occupancy grid")

    return {
        "success": not errors,
        "recording_source": "raw_odometry_full_run",
        "exit_reason": exit_reason,
        "requested_duration_sec": requested_duration_sec,
        "elapsed_sec": elapsed_sec,
        "requirements": {
            "odometry_streams_required": True,
            "maps_required": require_maps,
        },
        "ground_truth_base_to_camera": transform_summary(
            recorder.base_to_camera
        ),
        "trajectories": {
            "atlas_camera": atlas_summary,
            "ground_truth_camera": ground_truth_summary,
        },
        "path_diagnostics": {
            "atlas": recorder.atlas_path.summary(),
            "ground_truth": recorder.ground_truth_path.summary(),
        },
        "maps": {
            "dense": dense_summary,
            "occupancy_grid": occupancy_summary,
        },
        "errors": errors,
    }


def record_until_stopped(
    recorder: AutosimRecorder,
    *,
    duration_sec: float,
    stop_event: threading.Event,
    monotonic: Callable[[], float] = time.monotonic,
    sleeper: Callable[[float], None] = time.sleep,
) -> tuple[float, str]:
    """Poll until duration, signal request, or Autolink shutdown."""
    started = monotonic()
    deadline = started + duration_sec if duration_sec > 0.0 else None
    exit_reason = "stop_requested"
    while True:
        recorder.poll_once()
        now = monotonic()
        if stop_event.is_set():
            exit_reason = "signal"
            break
        if deadline is not None and now >= deadline:
            exit_reason = "duration"
            break
        if hasattr(recorder.link, "is_shutdown") and recorder.link.is_shutdown():
            exit_reason = "autolink_shutdown"
            break
        sleeper(0.02)
    recorder.poll_once()
    return max(0.0, monotonic() - started), exit_reason


def finite_values(values: Sequence[float], name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite values")
    return result


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Record full-run Atlas camera odometry and extrinsic-corrected "
            "AutoSim ground truth as TUM trajectories."
        )
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=0.0,
        help="recording duration; 0 waits for SIGINT (default: 0)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="directory for TUM files and validation_summary.json",
    )
    parser.add_argument(
        "--require-maps",
        action="store_true",
        help="fail unless dense point cloud and occupancy grid are valid",
    )
    parser.add_argument(
        "--base-to-camera-xyz",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=DEFAULT_BASE_TO_CAMERA_XYZ,
        help="base-to-camera translation in metres (default: 0.03 0 0.20)",
    )
    parser.add_argument(
        "--base-to-camera-quaternion",
        type=float,
        nargs=4,
        metavar=("QX", "QY", "QZ", "QW"),
        default=DEFAULT_BASE_TO_CAMERA_QUATERNION,
        help=(
            "base-to-camera xyzw rotation; default is the standard optical "
            "rotation -0.5 0.5 -0.5 0.5"
        ),
    )
    parser.add_argument("--node-name", default="atlas_autosim_recorder")
    parser.add_argument("--atlas-odometry-topic", default=ATLAS_ODOMETRY_TOPIC)
    parser.add_argument(
        "--ground-truth-odometry-topic",
        default=GROUND_TRUTH_ODOMETRY_TOPIC,
    )
    parser.add_argument("--atlas-path-topic", default=ATLAS_PATH_TOPIC)
    parser.add_argument(
        "--ground-truth-path-topic", default=GROUND_TRUTH_PATH_TOPIC
    )
    parser.add_argument("--dense-map-topic", default=DENSE_MAP_TOPIC)
    parser.add_argument(
        "--occupancy-grid-topic", default=OCCUPANCY_GRID_TOPIC
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        if not math.isfinite(args.duration_sec) or args.duration_sec < 0.0:
            raise ValueError("--duration-sec must be finite and nonnegative")
        translation = finite_values(
            args.base_to_camera_xyz, "--base-to-camera-xyz"
        )
        quaternion = normalize_quaternion(args.base_to_camera_quaternion)
        base_to_camera = RigidTransform(translation, quaternion)
    except ValueError as error:
        print(f"record_autosim_trajectories: error: {error}", file=sys.stderr)
        return 2

    link = None
    recorder = None
    stop_event = threading.Event()
    previous_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, lambda _signum, _frame: stop_event.set())
    try:
        link, message_types = load_runtime()
        recorder = AutosimRecorder(
            link,
            message_types,
            node_name=args.node_name,
            atlas_odometry_topic=args.atlas_odometry_topic,
            ground_truth_odometry_topic=args.ground_truth_odometry_topic,
            atlas_path_topic=args.atlas_path_topic,
            ground_truth_path_topic=args.ground_truth_path_topic,
            dense_map_topic=args.dense_map_topic,
            occupancy_grid_topic=args.occupancy_grid_topic,
            base_to_camera=base_to_camera,
        )
        elapsed_sec, exit_reason = record_until_stopped(
            recorder,
            duration_sec=args.duration_sec,
            stop_event=stop_event,
        )
        recorder.freeze()
        summary = build_summary(
            recorder,
            output_dir=args.output_dir,
            require_maps=args.require_maps,
            requested_duration_sec=args.duration_sec,
            elapsed_sec=elapsed_sec,
            exit_reason=exit_reason,
        )
        atomic_write_text(
            args.output_dir / "atlas_trajectory.tum",
            tum_text(
                recorder.atlas_odometry.poses,
                args.atlas_odometry_topic,
                ("Atlas camera pose; every received odometry sample",),
            ),
        )
        extrinsic = transform_summary(base_to_camera)
        atomic_write_text(
            args.output_dir / "ground_truth_trajectory.tum",
            tum_text(
                recorder.ground_truth_odometry.poses,
                args.ground_truth_odometry_topic,
                (
                    "AutoSim base pose composed with base-to-camera extrinsic",
                    "base_to_camera=" + json.dumps(extrinsic, sort_keys=True),
                ),
            ),
        )
        encoded_summary = json.dumps(
            summary, indent=2, sort_keys=True, allow_nan=False
        )
        atomic_write_text(
            args.output_dir / "validation_summary.json",
            encoded_summary + "\n",
        )
        print(encoded_summary)
        if summary["success"]:
            return 0
        print(
            "record_autosim_trajectories: validation failed: "
            + "; ".join(summary["errors"]),
            file=sys.stderr,
        )
        return 2
    except (OSError, RuntimeError, ValueError) as error:
        print(f"record_autosim_trajectories: error: {error}", file=sys.stderr)
        return 2
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        if recorder is not None:
            try:
                recorder.shutdown()
            except Exception as error:
                print(
                    f"record_autosim_trajectories: shutdown warning: {error}",
                    file=sys.stderr,
                )
        elif link is not None:
            try:
                link.shutdown()
            except Exception as error:
                print(
                    f"record_autosim_trajectories: shutdown warning: {error}",
                    file=sys.stderr,
                )


if __name__ == "__main__":
    raise SystemExit(main())
