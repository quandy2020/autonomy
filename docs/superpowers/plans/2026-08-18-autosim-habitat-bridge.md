# autosim Habitat Bridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `autosim/` 落地 Habitat-Sim 传感器–执行器桥，经 autolink + automsgs 发布 LiDAR/RGB-D/IMU/Odom（可选真值），订阅 `/cmd_vel`，供现有 SLAM/导航栈联调。

**Architecture:** 单进程分层：`world`/`agent`（Habitat）→ `drive`/传感器/`truth`（IO）→ `codec`（proto）→ `bridge`/`runner`（autolink）。无 Habitat 时用 `MockWorld` 跑通 CI。

**Tech Stack:** Python 3.10+、Habitat-Sim（可选运行时）、autolink Python、automsgs `*_pb2`、PyYAML、NumPy、pytest

**Spec:** `docs/superpowers/specs/2026-08-18-autosim-habitat-bridge-design.md`

---

## File Map

| 路径 | 职责 |
|------|------|
| `autosim/pyproject.toml` | 包元数据与依赖 |
| `autosim/README.md` | 安装、运行、通道说明 |
| `autosim/autosim/__init__.py` | 包版本 |
| `autosim/autosim/__main__.py` | `python -m autosim` |
| `autosim/autosim/config.py` | YAML 加载与校验 |
| `autosim/autosim/clock.py` | 仿真时间 |
| `autosim/autosim/drive.py` | Twist 限速 / 积分 / watchdog |
| `autosim/autosim/codec.py` | NumPy → automsgs proto |
| `autosim/autosim/odometry.py` | 轮式里程计状态 |
| `autosim/autosim/imu.py` | 位姿差分 IMU |
| `autosim/autosim/truth.py` | 真值位姿采样 |
| `autosim/autosim/lidar.py` | 深度环 → ranges |
| `autosim/autosim/rgbd.py` | RGB/Depth 观测 |
| `autosim/autosim/agent.py` | Agent 状态与外参 |
| `autosim/autosim/world.py` | Habitat 世界；含 `MockWorld` |
| `autosim/autosim/bridge.py` | Writer/Reader 封装 |
| `autosim/autosim/runner.py` | 主循环编排 |
| `autosim/config/default.yaml` | 默认配置 |
| `autosim/assets/scenes/minimal/README.md` | 轻量场景说明 |
| `autosim/examples/run_bridge.py` | 示例入口 |
| `autosim/tests/test_config.py` | 配置测试 |
| `autosim/tests/test_clock.py` | 时钟测试 |
| `autosim/tests/test_drive.py` | 驱动测试 |
| `autosim/tests/test_codec.py` | codec 测试 |
| `autosim/tests/test_odometry.py` | 里程计测试 |
| `autosim/tests/test_imu.py` | IMU 测试 |
| `autosim/tests/test_runner_smoke.py` | MockWorld 烟雾测试 |

**Proto 导入约定（与 `automsgs/python/test/basic_TEST.py` 一致）：**

```python
from automsgs.msgs.sensor_msgs.laser_scan_pb2 import LaserScan
from automsgs.msgs.sensor_msgs.image_pb2 import Image
from automsgs.msgs.sensor_msgs.camera_info_pb2 import CameraInfo
from automsgs.msgs.sensor_msgs.imu_pb2 import Imu
from automsgs.msgs.nav_msgs.odometry_pb2 import Odometry
from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped
from automsgs.msgs.geometry_msgs.twist_pb2 import Twist
from automsgs.msgs.geometry_msgs.pose_stamped_pb2 import PoseStamped
```

**前提：** 本机已编译安装 automsgs Python 绑定，且 `PYTHONPATH` 含 install 的 `lib/python`；autolink Python 同理。测试中对 `autolink` 使用 monkeypatch/fake，避免强依赖运行中的 discovery。

---

### Task 1: 包脚手架

**Files:**
- Create: `autosim/pyproject.toml`
- Create: `autosim/README.md`
- Create: `autosim/autosim/__init__.py`
- Create: `autosim/tests/__init__.py`

- [ ] **Step 1: 创建 pyproject.toml**

```toml
[build-system]
requires = ["setuptools>=68", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "autosim"
version = "0.1.0"
description = "Habitat-Sim sensor-actuator bridge for autonomy via autolink/automsgs"
readme = "README.md"
requires-python = ">=3.10"
dependencies = [
  "numpy>=1.24",
  "PyYAML>=6.0",
  "protobuf>=4.21",
]

[project.optional-dependencies]
habitat = ["habitat-sim>=0.3.0"]
dev = ["pytest>=7.0"]

[project.scripts]
autosim = "autosim.runner:main"

[tool.setuptools.packages.find]
where = ["."]
include = ["autosim*"]

[tool.pytest.ini_options]
testpaths = ["tests"]
pythonpath = ["."]
```

- [ ] **Step 2: 创建包初始化与 README**

`autosim/autosim/__init__.py`:

```python
"""Habitat-Sim bridge for autonomy (autolink + automsgs)."""

__version__ = "0.1.0"
```

`autosim/tests/__init__.py`: 空文件。

`autosim/README.md` 至少包含：职责边界（仅桥）、依赖、`pip install -e ".[dev]"`、`python -m autosim --config config/default.yaml`、默认通道表（与 spec §4 一致）。

- [ ] **Step 3: 验证包可导入**

Run:

```bash
cd autosim && pip install -e ".[dev]" && python -c "import autosim; print(autosim.__version__)"
```

Expected: `0.1.0`

- [ ] **Step 4: Commit**

```bash
git add autosim/pyproject.toml autosim/README.md autosim/autosim/__init__.py autosim/tests/__init__.py
git commit -m "chore(autosim): scaffold Python package"
```

---

### Task 2: config 加载与校验

**Files:**
- Create: `autosim/autosim/config.py`
- Create: `autosim/config/default.yaml`
- Create: `autosim/tests/test_config.py`

- [ ] **Step 1: 写失败测试**

```python
# autosim/tests/test_config.py
from pathlib import Path

import pytest

from autosim.config import load_config, ConfigError


ROOT = Path(__file__).resolve().parents[1]


def test_load_default_yaml():
    cfg = load_config(ROOT / "config" / "default.yaml")
    assert cfg["channels"]["cmd_vel"] == "/cmd_vel"
    assert cfg["channels"]["scan"] == "/scan"
    assert cfg["truth"]["enabled"] is False
    assert cfg["scene"]["backend"] == "minimal"


def test_reject_empty_channel():
    bad = {
        "scene": {"backend": "minimal", "path": "", "spawn": [0.0, 0.0, 0.0]},
        "robot": {
            "max_linear": 0.5,
            "max_angular": 1.0,
            "wheel_separation": 0.3,
        },
        "rates": {"control_hz": 50.0, "scan_hz": 10.0, "rgb_hz": 15.0, "imu_hz": 100.0},
        "channels": {
            "cmd_vel": "",
            "scan": "/scan",
            "rgb": "/camera/rgb/image_raw",
            "depth": "/camera/depth/image_raw",
            "camera_info": "/camera/camera_info",
            "imu": "/imu",
            "odom": "/odom",
            "gt_pose": "/gt/pose",
        },
        "noise": {"odom": 0.0, "imu": 0.0, "lidar": 0.0},
        "truth": {"enabled": False},
        "habitat": {"gpu": 0, "width": 640, "height": 480, "headless": True},
        "watchdog_sec": 0.5,
    }
    with pytest.raises(ConfigError):
        load_config(bad)


def test_reject_duplicate_channels():
    bad = {
        "scene": {"backend": "minimal", "path": "", "spawn": [0.0, 0.0, 0.0]},
        "robot": {"max_linear": 0.5, "max_angular": 1.0, "wheel_separation": 0.3},
        "rates": {"control_hz": 50.0, "scan_hz": 10.0, "rgb_hz": 15.0, "imu_hz": 100.0},
        "channels": {
            "cmd_vel": "/scan",
            "scan": "/scan",
            "rgb": "/camera/rgb/image_raw",
            "depth": "/camera/depth/image_raw",
            "camera_info": "/camera/camera_info",
            "imu": "/imu",
            "odom": "/odom",
            "gt_pose": "/gt/pose",
        },
        "noise": {"odom": 0.0, "imu": 0.0, "lidar": 0.0},
        "truth": {"enabled": False},
        "habitat": {"gpu": 0, "width": 640, "height": 480, "headless": True},
        "watchdog_sec": 0.5,
    }
    with pytest.raises(ConfigError):
        load_config(bad)
```

- [ ] **Step 2: 运行确认失败**

Run: `cd autosim && pytest tests/test_config.py -v`

Expected: FAIL（`autosim.config` 不存在）

- [ ] **Step 3: 写 default.yaml 与 config.py**

`autosim/config/default.yaml`:

```yaml
scene:
  backend: minimal   # minimal | habitat
  path: ""           # habitat 时填官方数据集场景路径
  spawn: [0.0, 0.0, 0.0]  # x, y, yaw

robot:
  max_linear: 0.5
  max_angular: 1.0
  wheel_separation: 0.30
  frames:
    base: base_link
    laser: laser_link
    camera: camera_link
    imu: imu_link
    odom: odom
    map: map

rates:
  control_hz: 50.0
  scan_hz: 10.0
  rgb_hz: 15.0
  imu_hz: 100.0

channels:
  cmd_vel: /cmd_vel
  scan: /scan
  rgb: /camera/rgb/image_raw
  depth: /camera/depth/image_raw
  camera_info: /camera/camera_info
  imu: /imu
  odom: /odom
  gt_pose: /gt/pose

noise:
  odom: 0.0
  imu: 0.0
  lidar: 0.0

truth:
  enabled: false

habitat:
  gpu: 0
  width: 640
  height: 480
  headless: true

watchdog_sec: 0.5

lidar:
  angle_min: -3.14159
  angle_max: 3.14159
  range_min: 0.1
  range_max: 30.0
  num_beams: 360
```

`autosim/autosim/config.py`:

```python
from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping, MutableMapping, Union

import yaml

ConfigDict = MutableMapping[str, Any]
ConfigSource = Union[str, Path, Mapping[str, Any]]

_REQUIRED_CHANNELS = (
    "cmd_vel",
    "scan",
    "rgb",
    "depth",
    "camera_info",
    "imu",
    "odom",
    "gt_pose",
)


class ConfigError(ValueError):
    """Invalid autosim configuration."""


def load_config(source: ConfigSource) -> ConfigDict:
    if isinstance(source, (str, Path)):
        path = Path(source)
        with path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    elif isinstance(source, Mapping):
        data = dict(source)
    else:
        raise ConfigError(f"unsupported config source: {type(source)!r}")

    if not isinstance(data, dict):
        raise ConfigError("config root must be a mapping")
    _validate(data)
    return data


def _validate(cfg: Mapping[str, Any]) -> None:
    channels = cfg.get("channels")
    if not isinstance(channels, Mapping):
        raise ConfigError("channels must be a mapping")
    names: list[str] = []
    for key in _REQUIRED_CHANNELS:
        if key not in channels:
            raise ConfigError(f"missing channel: {key}")
        name = channels[key]
        if not isinstance(name, str) or not name.strip():
            raise ConfigError(f"empty channel name: {key}")
        names.append(name)
    if len(names) != len(set(names)):
        raise ConfigError("duplicate channel names")
    backend = cfg.get("scene", {}).get("backend")
    if backend not in ("minimal", "habitat"):
        raise ConfigError("scene.backend must be 'minimal' or 'habitat'")
    if "truth" not in cfg or "enabled" not in cfg["truth"]:
        raise ConfigError("truth.enabled required")
```

- [ ] **Step 4: 运行确认通过**

Run: `cd autosim && pytest tests/test_config.py -v`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add autosim/autosim/config.py autosim/config/default.yaml autosim/tests/test_config.py
git commit -m "feat(autosim): add config load and validation"
```

---

### Task 3: clock

**Files:**
- Create: `autosim/autosim/clock.py`
- Create: `autosim/tests/test_clock.py`

- [ ] **Step 1: 写失败测试**

```python
# autosim/tests/test_clock.py
from autosim.clock import SimClock


def test_tick_advances_and_stamp():
    clock = SimClock(start_sec=0.0)
    clock.tick(0.02)
    sec, nanosec = clock.stamp()
    assert sec == 0
    assert nanosec == 20_000_000
    assert abs(clock.now() - 0.02) < 1e-9


def test_stamp_rolls_seconds():
    clock = SimClock(start_sec=0.0)
    clock.tick(1.5)
    sec, nanosec = clock.stamp()
    assert sec == 1
    assert nanosec == 500_000_000
```

- [ ] **Step 2: 运行确认失败**

Run: `cd autosim && pytest tests/test_clock.py -v`

Expected: FAIL

- [ ] **Step 3: 实现 clock.py**

```python
from __future__ import annotations


class SimClock:
    """Monotonic simulation clock used for message headers."""

    def __init__(self, start_sec: float = 0.0) -> None:
        self._t = float(start_sec)

    def tick(self, dt: float) -> None:
        if dt < 0.0:
            raise ValueError("dt must be non-negative")
        self._t += dt

    def now(self) -> float:
        return self._t

    def stamp(self) -> tuple[int, int]:
        sec = int(self._t)
        nanosec = int(round((self._t - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return sec, nanosec
```

- [ ] **Step 4: 运行确认通过**

Run: `cd autosim && pytest tests/test_clock.py -v`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add autosim/autosim/clock.py autosim/tests/test_clock.py
git commit -m "feat(autosim): add simulation clock"
```

---

### Task 4: drive（限速 / 积分 / watchdog）

**Files:**
- Create: `autosim/autosim/drive.py`
- Create: `autosim/tests/test_drive.py`

- [ ] **Step 1: 写失败测试**

```python
# autosim/tests/test_drive.py
import math

from autosim.drive import Drive


def test_clamp_and_integrate():
    drive = Drive(max_linear=0.5, max_angular=1.0, watchdog_sec=0.5)
    drive.set_twist(2.0, 3.0, t=0.0)
    x, y, yaw = drive.step(dt=0.1, t=0.1)
    assert abs(x - 0.05) < 1e-9  # 0.5 * 0.1
    assert abs(y) < 1e-9
    assert abs(yaw - 0.1) < 1e-9  # 1.0 * 0.1


def test_watchdog_zeros_velocity():
    drive = Drive(max_linear=0.5, max_angular=1.0, watchdog_sec=0.2)
    drive.set_twist(0.5, 0.0, t=0.0)
    drive.step(dt=0.1, t=0.1)
    x0, _, _ = drive.pose()
    drive.step(dt=0.1, t=0.35)  # timeout
    x1, _, _ = drive.pose()
    assert abs(x1 - x0) < 1e-9


def test_yaw_wrap():
    drive = Drive(max_linear=0.0, max_angular=2.0, watchdog_sec=10.0)
    drive.set_twist(0.0, 2.0, t=0.0)
    drive.step(dt=math.pi, t=0.1)  # delta yaw ~ 2*pi
    _, _, yaw = drive.pose()
    assert abs(yaw) < 1e-6 or abs(abs(yaw) - math.pi) < 1e-6 or abs(yaw) <= math.pi
```

- [ ] **Step 2: 运行确认失败**

Run: `cd autosim && pytest tests/test_drive.py -v`

Expected: FAIL

- [ ] **Step 3: 实现 drive.py**

```python
from __future__ import annotations

import math
from typing import Tuple


def _wrap_yaw(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


class Drive:
    """Differential-drive command filter and planar integrator."""

    def __init__(
        self,
        max_linear: float,
        max_angular: float,
        watchdog_sec: float,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
    ) -> None:
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.watchdog_sec = float(watchdog_sec)
        self._x = float(x)
        self._y = float(y)
        self._yaw = float(yaw)
        self._v = 0.0
        self._w = 0.0
        self._last_cmd_t = 0.0

    def set_twist(self, linear_x: float, angular_z: float, t: float) -> None:
        self._v = max(-self.max_linear, min(self.max_linear, float(linear_x)))
        self._w = max(-self.max_angular, min(self.max_angular, float(angular_z)))
        self._last_cmd_t = float(t)

    def step(self, dt: float, t: float) -> Tuple[float, float, float]:
        if t - self._last_cmd_t > self.watchdog_sec:
            self._v = 0.0
            self._w = 0.0
        self._x += self._v * math.cos(self._yaw) * dt
        self._y += self._v * math.sin(self._yaw) * dt
        self._yaw = _wrap_yaw(self._yaw + self._w * dt)
        return self.pose()

    def pose(self) -> Tuple[float, float, float]:
        return self._x, self._y, self._yaw

    def velocity(self) -> Tuple[float, float]:
        return self._v, self._w
```

- [ ] **Step 4: 运行确认通过**

Run: `cd autosim && pytest tests/test_drive.py -v`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add autosim/autosim/drive.py autosim/tests/test_drive.py
git commit -m "feat(autosim): add differential drive integrator"
```

---

### Task 5: codec（proto 编码）

**Files:**
- Create: `autosim/autosim/codec.py`
- Create: `autosim/tests/test_codec.py`

> 若环境未安装 automsgs Python，本 Task 的测试会 fail on import；执行前确认 `PYTHONPATH` 含 automsgs 安装目录（见 `automsgs/python/README.md`）。

- [ ] **Step 1: 写失败测试**

```python
# autosim/tests/test_codec.py
import numpy as np

from autosim.codec import (
    encode_laser_scan,
    encode_image,
    encode_camera_info,
    encode_imu,
    encode_odometry,
    encode_pose_stamped,
    parse_cmd_vel,
)


def test_encode_laser_scan_fields():
    ranges = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    msg = encode_laser_scan(
        ranges=ranges,
        stamp=(1, 500000000),
        frame_id="laser_link",
        angle_min=-1.0,
        angle_max=1.0,
        angle_increment=1.0,
        range_min=0.1,
        range_max=30.0,
        scan_time=0.1,
    )
    assert msg.header.frame_id == "laser_link"
    assert msg.header.stamp.sec == 1
    assert list(msg.ranges) == [1.0, 2.0, 3.0]


def test_encode_image_rgb8():
    img = np.zeros((2, 3, 3), dtype=np.uint8)
    img[0, 0] = [1, 2, 3]
    msg = encode_image(img, stamp=(0, 0), frame_id="camera_link", encoding="rgb8")
    assert msg.height == 2
    assert msg.width == 3
    assert msg.encoding == "rgb8"
    assert msg.step == 9
    assert msg.data[0:3] == bytes([1, 2, 3])


def test_parse_cmd_vel_twist_and_stamped():
    from automsgs.msgs.geometry_msgs.twist_pb2 import Twist
    from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped

    t = Twist()
    t.linear.x = 0.2
    t.angular.z = -0.1
    assert parse_cmd_vel(t) == (0.2, -0.1)

    ts = TwistStamped()
    ts.twist.linear.x = 0.3
    ts.twist.angular.z = 0.4
    assert parse_cmd_vel(ts) == (0.3, 0.4)


def test_encode_odometry_frames():
    msg = encode_odometry(
        x=1.0,
        y=2.0,
        yaw=0.0,
        v=0.1,
        w=0.0,
        stamp=(0, 0),
        frame_id="odom",
        child_frame_id="base_link",
    )
    assert msg.header.frame_id == "odom"
    assert msg.child_frame_id == "base_link"
    assert abs(msg.pose.pose.pose.position.x - 1.0) < 1e-9
```

- [ ] **Step 2: 运行确认失败**

Run: `cd autosim && pytest tests/test_codec.py -v`

Expected: FAIL

- [ ] **Step 3: 实现 codec.py**

```python
from __future__ import annotations

import math
from typing import Sequence, Tuple, Union

import numpy as np

from automsgs.msgs.geometry_msgs.pose_stamped_pb2 import PoseStamped
from automsgs.msgs.geometry_msgs.twist_pb2 import Twist
from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped
from automsgs.msgs.nav_msgs.odometry_pb2 import Odometry
from automsgs.msgs.sensor_msgs.camera_info_pb2 import CameraInfo
from automsgs.msgs.sensor_msgs.image_pb2 import Image
from automsgs.msgs.sensor_msgs.imu_pb2 import Imu
from automsgs.msgs.sensor_msgs.laser_scan_pb2 import LaserScan

Stamp = Tuple[int, int]


def _set_header(header, stamp: Stamp, frame_id: str) -> None:
    header.stamp.sec = int(stamp[0])
    header.stamp.nanosec = int(stamp[1])
    header.frame_id = frame_id


def _yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))  # x,y,z,w


def encode_laser_scan(
    ranges: np.ndarray,
    stamp: Stamp,
    frame_id: str,
    angle_min: float,
    angle_max: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    scan_time: float,
    intensities: np.ndarray | None = None,
) -> LaserScan:
    msg = LaserScan()
    _set_header(msg.header, stamp, frame_id)
    msg.angle_min = float(angle_min)
    msg.angle_max = float(angle_max)
    msg.angle_increment = float(angle_increment)
    msg.time_increment = 0.0
    msg.scan_time = float(scan_time)
    msg.range_min = float(range_min)
    msg.range_max = float(range_max)
    msg.ranges.extend(float(r) for r in ranges.reshape(-1))
    if intensities is not None:
        msg.intensities.extend(float(i) for i in intensities.reshape(-1))
    return msg


def encode_image(
    image: np.ndarray,
    stamp: Stamp,
    frame_id: str,
    encoding: str,
) -> Image:
    msg = Image()
    _set_header(msg.header, stamp, frame_id)
    if image.ndim == 2:
        height, width = image.shape
        channels = 1
    else:
        height, width, channels = image.shape
    msg.height = int(height)
    msg.width = int(width)
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = int(width * channels * image.dtype.itemsize)
    msg.data = np.ascontiguousarray(image).tobytes()
    return msg


def encode_camera_info(
    stamp: Stamp,
    frame_id: str,
    width: int,
    height: int,
    k: Sequence[float],
    p: Sequence[float] | None = None,
) -> CameraInfo:
    msg = CameraInfo()
    _set_header(msg.header, stamp, frame_id)
    msg.width = int(width)
    msg.height = int(height)
    msg.distortion_model = "plumb_bob"
    msg.d.extend([0.0, 0.0, 0.0, 0.0, 0.0])
    k9 = [float(v) for v in k]
    if len(k9) != 9:
        raise ValueError("camera matrix K must have length 9")
    msg.k.extend(k9)
    msg.r.extend([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    if p is None:
        fx, fy, cx, cy = k9[0], k9[4], k9[2], k9[5]
        p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    msg.p.extend(float(v) for v in p)
    return msg


def encode_imu(
    stamp: Stamp,
    frame_id: str,
    orient_xyzw: Tuple[float, float, float, float],
    gyro_xyz: Tuple[float, float, float],
    accel_xyz: Tuple[float, float, float],
) -> Imu:
    msg = Imu()
    _set_header(msg.header, stamp, frame_id)
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = orient_xyzw
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro_xyz
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel_xyz
    msg.orientation_covariance.extend([-1.0] + [0.0] * 8)
    msg.angular_velocity_covariance.extend([0.0] * 9)
    msg.linear_acceleration_covariance.extend([0.0] * 9)
    return msg


def encode_odometry(
    x: float,
    y: float,
    yaw: float,
    v: float,
    w: float,
    stamp: Stamp,
    frame_id: str,
    child_frame_id: str,
) -> Odometry:
    msg = Odometry()
    _set_header(msg.header, stamp, frame_id)
    msg.child_frame_id = child_frame_id
    qx, qy, qz, qw = _yaw_to_quat(yaw)
    pose = msg.pose.pose
    _set_header(pose.header, stamp, frame_id)
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    msg.pose.covariance.extend([0.0] * 36)
    msg.twist.twist.linear.x = float(v)
    msg.twist.twist.angular.z = float(w)
    msg.twist.covariance.extend([0.0] * 36)
    return msg


def encode_pose_stamped(
    x: float,
    y: float,
    yaw: float,
    stamp: Stamp,
    frame_id: str,
) -> PoseStamped:
    msg = PoseStamped()
    _set_header(msg.header, stamp, frame_id)
    qx, qy, qz, qw = _yaw_to_quat(yaw)
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


def parse_cmd_vel(msg: Union[Twist, TwistStamped]) -> Tuple[float, float]:
    if isinstance(msg, TwistStamped):
        twist = msg.twist
    elif isinstance(msg, Twist):
        twist = msg
    else:
        # duck-typing for raw protobuf of either shape
        twist = getattr(msg, "twist", msg)
    return float(twist.linear.x), float(twist.angular.z)
```

- [ ] **Step 4: 运行确认通过**

Run: `cd autosim && pytest tests/test_codec.py -v`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add autosim/autosim/codec.py autosim/tests/test_codec.py
git commit -m "feat(autosim): add automsgs codec helpers"
```

---

### Task 6: odometry / imu / truth（纯状态模块）

**Files:**
- Create: `autosim/autosim/odometry.py`
- Create: `autosim/autosim/imu.py`
- Create: `autosim/autosim/truth.py`
- Create: `autosim/tests/test_odometry.py`
- Create: `autosim/tests/test_imu.py`

- [ ] **Step 1: 写失败测试**

```python
# autosim/tests/test_odometry.py
from autosim.odometry import WheelOdometry


def test_odometry_tracks_drive_pose_with_zero_noise():
    odom = WheelOdometry(noise_std=0.0, seed=0)
    x, y, yaw = odom.update(gt_x=1.0, gt_y=0.0, gt_yaw=0.1)
    assert (x, y, yaw) == (1.0, 0.0, 0.1)


# autosim/tests/test_imu.py
from autosim.imu import ImuEstimator


def test_imu_finite_difference():
    imu = ImuEstimator()
    imu.reset(yaw=0.0, t=0.0)
    gyro_z, ax, ay = imu.update(yaw=0.1, v=1.0, t=0.1)
    assert abs(gyro_z - 1.0) < 1e-6
    assert abs(ax) < 1e-6  # constant speed
```

- [ ] **Step 2: 运行确认失败**

Run: `cd autosim && pytest tests/test_odometry.py tests/test_imu.py -v`

Expected: FAIL

- [ ] **Step 3: 实现三模块**

```python
# autosim/autosim/odometry.py
from __future__ import annotations

from typing import Tuple

import numpy as np


class WheelOdometry:
    """Wheel odometry pose; optional isotropic XY noise on updates."""

    def __init__(self, noise_std: float = 0.0, seed: int = 0) -> None:
        self.noise_std = float(noise_std)
        self._rng = np.random.default_rng(seed)
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

    def update(self, gt_x: float, gt_y: float, gt_yaw: float) -> Tuple[float, float, float]:
        self._x = float(gt_x)
        self._y = float(gt_y)
        self._yaw = float(gt_yaw)
        if self.noise_std > 0.0:
            self._x += float(self._rng.normal(0.0, self.noise_std))
            self._y += float(self._rng.normal(0.0, self.noise_std))
        return self._x, self._y, self._yaw

    def pose(self) -> Tuple[float, float, float]:
        return self._x, self._y, self._yaw
```

```python
# autosim/autosim/imu.py
from __future__ import annotations

from typing import Optional, Tuple


class ImuEstimator:
    """Finite-difference yaw rate and body accel from planar motion."""

    def __init__(self) -> None:
        self._yaw: Optional[float] = None
        self._v: Optional[float] = None
        self._t: Optional[float] = None

    def reset(self, yaw: float, t: float, v: float = 0.0) -> None:
        self._yaw = float(yaw)
        self._v = float(v)
        self._t = float(t)

    def update(self, yaw: float, v: float, t: float) -> Tuple[float, float, float]:
        if self._t is None or self._yaw is None or self._v is None:
            self.reset(yaw, t, v)
            return 0.0, 0.0, 0.0
        dt = t - self._t
        if dt <= 0.0:
            return 0.0, 0.0, 0.0
        gyro_z = (yaw - self._yaw) / dt
        ax = (v - self._v) / dt
        ay = 0.0
        self._yaw = float(yaw)
        self._v = float(v)
        self._t = float(t)
        return float(gyro_z), float(ax), float(ay)
```

```python
# autosim/autosim/truth.py
from __future__ import annotations

from typing import Tuple


class GroundTruth:
    """Pass-through ground-truth pose sampler."""

    def sample(self, x: float, y: float, yaw: float) -> Tuple[float, float, float]:
        return float(x), float(y), float(yaw)
```

- [ ] **Step 4: 运行确认通过**

Run: `cd autosim && pytest tests/test_odometry.py tests/test_imu.py -v`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add autosim/autosim/odometry.py autosim/autosim/imu.py autosim/autosim/truth.py \
  autosim/tests/test_odometry.py autosim/tests/test_imu.py
git commit -m "feat(autosim): add odometry imu and truth modules"
```

---

### Task 7: World 协议、MockWorld、lidar/rgbd/agent

**Files:**
- Create: `autosim/autosim/agent.py`
- Create: `autosim/autosim/world.py`
- Create: `autosim/autosim/lidar.py`
- Create: `autosim/autosim/rgbd.py`
- Create: `autosim/tests/test_mock_world.py`

- [ ] **Step 1: 写失败测试**

```python
# autosim/tests/test_mock_world.py
import numpy as np

from autosim.world import MockWorld
from autosim.lidar import Lidar
from autosim.rgbd import Rgbd


def test_mock_world_step_and_sensors():
    world = MockWorld(width=64, height=48)
    world.reset(x=0.0, y=0.0, yaw=0.0)
    world.set_pose(0.1, 0.0, 0.0)
    world.step()
    lidar = Lidar(angle_min=-np.pi, angle_max=np.pi, num_beams=36, range_min=0.1, range_max=10.0)
    ranges = lidar.sample(world)
    assert ranges.shape == (36,)
    assert np.all(ranges >= 0.1)
    rgbd = Rgbd()
    rgb, depth = rgbd.sample(world)
    assert rgb.shape[2] == 3
    assert depth.shape == rgb.shape[:2]
```

- [ ] **Step 2: 运行确认失败**

Run: `cd autosim && pytest tests/test_mock_world.py -v`

Expected: FAIL

- [ ] **Step 3: 实现协议与 Mock 传感器**

```python
# autosim/autosim/agent.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass
class AgentState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def as_tuple(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.yaw
```

```python
# autosim/autosim/world.py
from __future__ import annotations

from typing import Protocol, Tuple

import numpy as np

from autosim.agent import AgentState


class World(Protocol):
    def reset(self, x: float, y: float, yaw: float) -> None: ...
    def set_pose(self, x: float, y: float, yaw: float) -> None: ...
    def step(self) -> None: ...
    def pose(self) -> Tuple[float, float, float]: ...
    def depth_ring(self, num_beams: int, range_max: float) -> np.ndarray: ...
    def rgb_depth(self) -> Tuple[np.ndarray, np.ndarray]: ...
    def close(self) -> None: ...


class MockWorld:
    """Deterministic stand-in for CI without habitat-sim."""

    def __init__(self, width: int = 64, height: int = 48) -> None:
        self.width = int(width)
        self.height = int(height)
        self._agent = AgentState()

    def reset(self, x: float, y: float, yaw: float) -> None:
        self._agent = AgentState(x=x, y=y, yaw=yaw)

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        self._agent = AgentState(x=x, y=y, yaw=yaw)

    def step(self) -> None:
        return None

    def pose(self) -> Tuple[float, float, float]:
        return self._agent.as_tuple()

    def depth_ring(self, num_beams: int, range_max: float) -> np.ndarray:
        # Constant mid-range scan for pipeline tests.
        return np.full((num_beams,), 0.5 * float(range_max), dtype=np.float32)

    def rgb_depth(self) -> Tuple[np.ndarray, np.ndarray]:
        rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        rgb[..., 1] = 128
        depth = np.full((self.height, self.width), 2.0, dtype=np.float32)
        return rgb, depth

    def close(self) -> None:
        return None


def create_world(cfg: dict) -> World:
    backend = cfg["scene"]["backend"]
    w = int(cfg["habitat"]["width"])
    h = int(cfg["habitat"]["height"])
    if backend == "minimal":
        # Prefer Habitat empty/minimal stage when available; else MockWorld.
        try:
            return HabitatWorld(cfg)
        except Exception:
            return MockWorld(width=w, height=h)
    if backend == "habitat":
        return HabitatWorld(cfg)
    raise ValueError(f"unknown backend: {backend}")
```

> 注意：`HabitatWorld` 在同文件 Task 8 实现；本 Task 先放占位类，使 `create_world` 在无 habitat 时回退 Mock。

同文件先加占位：

```python
class HabitatWorld:
    def __init__(self, cfg: dict) -> None:
        raise ImportError(
            "habitat-sim is required for HabitatWorld; pip install habitat-sim "
            f"or set scene path. requested path={cfg.get('scene', {}).get('path')!r}"
        )
```

```python
# autosim/autosim/lidar.py
from __future__ import annotations

import numpy as np

from autosim.world import World


class Lidar:
    def __init__(
        self,
        angle_min: float,
        angle_max: float,
        num_beams: int,
        range_min: float,
        range_max: float,
    ) -> None:
        self.angle_min = float(angle_min)
        self.angle_max = float(angle_max)
        self.num_beams = int(num_beams)
        self.range_min = float(range_min)
        self.range_max = float(range_max)

    @property
    def angle_increment(self) -> float:
        if self.num_beams <= 1:
            return 0.0
        return (self.angle_max - self.angle_min) / float(self.num_beams - 1)

    def sample(self, world: World) -> np.ndarray:
        ranges = world.depth_ring(self.num_beams, self.range_max).astype(np.float32)
        ranges = np.clip(ranges, self.range_min, self.range_max)
        return ranges
```

```python
# autosim/autosim/rgbd.py
from __future__ import annotations

from typing import Tuple

import numpy as np

from autosim.world import World


class Rgbd:
    def sample(self, world: World) -> Tuple[np.ndarray, np.ndarray]:
        return world.rgb_depth()
```

- [ ] **Step 4: 运行确认通过**

Run: `cd autosim && pytest tests/test_mock_world.py -v`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add autosim/autosim/agent.py autosim/autosim/world.py autosim/autosim/lidar.py \
  autosim/autosim/rgbd.py autosim/tests/test_mock_world.py
git commit -m "feat(autosim): add mock world and sensor adapters"
```

---

### Task 8: HabitatWorld

**Files:**
- Modify: `autosim/autosim/world.py`
- Create: `autosim/assets/scenes/minimal/README.md`

- [ ] **Step 1: 实现 HabitatWorld（替换占位）**

在 `world.py` 用真实实现替换 `HabitatWorld`：

```python
class HabitatWorld:
    """Habitat-Sim backed world. Requires optional dependency habitat-sim."""

    def __init__(self, cfg: dict) -> None:
        try:
            import habitat_sim
        except ImportError as exc:
            raise ImportError(
                "habitat-sim is not installed. Install with: pip install habitat-sim"
            ) from exc

        self._habitat_sim = habitat_sim
        self._cfg = cfg
        self._agent = AgentState()
        backend = habitat_sim.SimulatorConfiguration()
        backend.gpu_device_id = int(cfg["habitat"]["gpu"])
        scene_path = cfg["scene"].get("path") or ""
        if cfg["scene"]["backend"] == "minimal" and not scene_path:
            # Empty stage: pipeline bring-up without external datasets.
            backend.scene_id = habitat_sim.utils.settings.default_sim_settings.get(
                "scene", "NONE"
            )
            if hasattr(habitat_sim, "STAGE_EMPTY_SCENE"):
                backend.scene_id = habitat_sim.STAGE_EMPTY_SCENE
            else:
                backend.scene_id = "NONE"
        else:
            if not scene_path:
                raise FileNotFoundError(
                    "scene.path is required for backend=habitat; "
                    "set path to an HM3D/Replica/MP3D scene file"
                )
            path = scene_path
            from pathlib import Path

            if not Path(path).exists():
                raise FileNotFoundError(f"scene path does not exist: {path}")
            backend.scene_id = path

        agent_cfg = habitat_sim.agent.AgentConfiguration()
        rgb_sensor = habitat_sim.CameraSensorSpec()
        rgb_sensor.uuid = "rgb"
        rgb_sensor.sensor_type = habitat_sim.SensorType.COLOR
        rgb_sensor.resolution = [int(cfg["habitat"]["height"]), int(cfg["habitat"]["width"])]
        depth_sensor = habitat_sim.CameraSensorSpec()
        depth_sensor.uuid = "depth"
        depth_sensor.sensor_type = habitat_sim.SensorType.DEPTH
        depth_sensor.resolution = [int(cfg["habitat"]["height"]), int(cfg["habitat"]["width"])]
        agent_cfg.sensor_specifications = [rgb_sensor, depth_sensor]

        self._sim = habitat_sim.Simulator(habitat_sim.Configuration(backend, [agent_cfg]))
        spawn = cfg["scene"].get("spawn", [0.0, 0.0, 0.0])
        self.reset(float(spawn[0]), float(spawn[1]), float(spawn[2]))

    def reset(self, x: float, y: float, yaw: float) -> None:
        self.set_pose(x, y, yaw)

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        import magnum as mn

        self._agent = AgentState(x=x, y=y, yaw=yaw)
        agent = self._sim.get_agent(0)
        state = agent.get_state()
        state.position = mn.Vector3(float(x), 0.0, float(y))  # habitat Y-up: map y→z
        state.rotation = mn.Quaternion.rotation(mn.Rad(float(yaw)), mn.Vector3(0, 1, 0))
        agent.set_state(state, infer_sensor_states=True)

    def step(self) -> None:
        # Pose is set explicitly from Drive each cycle; no discrete action needed.
        return None

    def pose(self) -> Tuple[float, float, float]:
        return self._agent.as_tuple()

    def depth_ring(self, num_beams: int, range_max: float) -> np.ndarray:
        obs = self._sim.get_sensor_observations()
        depth = np.asarray(obs["depth"], dtype=np.float32)
        # Approximate 2D lidar from center row of depth image.
        row = depth[depth.shape[0] // 2, :]
        idx = np.linspace(0, row.shape[0] - 1, num=num_beams).astype(np.int32)
        ranges = row[idx]
        ranges[~np.isfinite(ranges)] = float(range_max)
        return ranges.astype(np.float32)

    def rgb_depth(self) -> Tuple[np.ndarray, np.ndarray]:
        obs = self._sim.get_sensor_observations()
        rgb = np.asarray(obs["rgb"])[..., :3].astype(np.uint8)
        depth = np.asarray(obs["depth"], dtype=np.float32)
        return rgb, depth

    def close(self) -> None:
        self._sim.close()
```

修正 `create_world`：`minimal` 在 ImportError / Habitat 初始化失败时回退 `MockWorld` 并打日志；`habitat` 后端失败则抛出（不静默回退）。

- [ ] **Step 2: 写 minimal 场景 README**

`autosim/assets/scenes/minimal/README.md`:

```markdown
# minimal scene

默认 `scene.backend: minimal` 使用 Habitat 空舞台（或 CI 下 `MockWorld`），无需下载数据集。

切换真实场景：

```yaml
scene:
  backend: habitat
  path: /data/hm3d/scene.basis.glb
```

支持 HM3D / Replica / MP3D 等 Habitat 可加载资产。
```

- [ ] **Step 3: 无 Habitat 时 Mock 测试仍通过**

Run: `cd autosim && pytest tests/test_mock_world.py tests/test_config.py -v`

Expected: PASS

- [ ] **Step 4: Commit**

```bash
git add autosim/autosim/world.py autosim/assets/scenes/minimal/README.md
git commit -m "feat(autosim): add HabitatWorld backend"
```

---

### Task 9: bridge + runner + 烟雾测试

**Files:**
- Create: `autosim/autosim/bridge.py`
- Create: `autosim/autosim/runner.py`
- Create: `autosim/autosim/__main__.py`
- Create: `autosim/examples/run_bridge.py`
- Create: `autosim/tests/test_runner_smoke.py`

- [ ] **Step 1: 写烟雾测试（fake autolink）**

```python
# autosim/tests/test_runner_smoke.py
from pathlib import Path

from autosim.config import load_config
from autosim.runner import BridgeRunner


class FakeWriter:
    def __init__(self):
        self.msgs = []

    def write(self, msg):
        self.msgs.append(msg)


class FakeReader:
    def __init__(self):
        self._msg = None
        self._has = False

    def set(self, msg):
        self._msg = msg
        self._has = True

    def has_msg(self):
        return self._has

    def get_msg(self):
        self._has = False
        return self._msg


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.writers = {}
        self.readers = {}

    def create_writer(self, channel, dtype, qos_depth=1):
        w = FakeWriter()
        self.writers[channel] = w
        return w

    def create_reader(self, channel, dtype, qos_depth=1):
        r = FakeReader()
        self.readers[channel] = r
        return r


def test_runner_publishes_with_mock_world(monkeypatch):
    import autosim.runner as runner_mod

    monkeypatch.setattr(runner_mod, "autolink", type("AL", (), {
        "init": staticmethod(lambda name: None),
        "shutdown": staticmethod(lambda: None),
        "is_shutdown": staticmethod(lambda: False),
        "Node": FakeNode,
    }))

    root = Path(__file__).resolve().parents[1]
    cfg = load_config(root / "config" / "default.yaml")
    cfg["scene"]["backend"] = "minimal"
    # Force mock path: monkeypatch create_world
    from autosim.world import MockWorld

    monkeypatch.setattr(runner_mod, "create_world", lambda c: MockWorld(64, 48))

    runner = BridgeRunner(cfg, max_steps=5)
    runner.run()
    assert "/scan" in runner.node.writers
    assert len(runner.node.writers["/scan"].msgs) >= 1
    assert len(runner.node.writers["/odom"].msgs) >= 1
```

- [ ] **Step 2: 运行确认失败**

Run: `cd autosim && pytest tests/test_runner_smoke.py -v`

Expected: FAIL

- [ ] **Step 3: 实现 bridge.py 与 runner.py**

```python
# autosim/autosim/bridge.py
from __future__ import annotations

from typing import Any, Dict


class Bridge:
    def __init__(self, node: Any, channels: Dict[str, str], types: Dict[str, Any]) -> None:
        self.node = node
        self.channels = channels
        self.writers = {
            key: node.create_writer(channels[key], types[key], qos_depth=10)
            for key in ("scan", "rgb", "depth", "camera_info", "imu", "odom", "gt_pose")
            if key in types
        }
        self.cmd_reader = node.create_reader(channels["cmd_vel"], types["cmd_vel"], qos_depth=10)

    def publish(self, key: str, msg: Any) -> None:
        if key in self.writers:
            self.writers[key].write(msg)
```

```python
# autosim/autosim/runner.py
from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np

try:
    import autolink
except ImportError:  # pragma: no cover
    autolink = None

from autosim.bridge import Bridge
from autosim.clock import SimClock
from autosim.codec import (
    encode_camera_info,
    encode_image,
    encode_imu,
    encode_laser_scan,
    encode_odometry,
    encode_pose_stamped,
    parse_cmd_vel,
)
from autosim.config import load_config
from autosim.drive import Drive
from autosim.imu import ImuEstimator
from autosim.lidar import Lidar
from autosim.odometry import WheelOdometry
from autosim.rgbd import Rgbd
from autosim.truth import GroundTruth
from autosim.world import create_world

from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped
from automsgs.msgs.geometry_msgs.pose_stamped_pb2 import PoseStamped
from automsgs.msgs.nav_msgs.odometry_pb2 import Odometry
from automsgs.msgs.sensor_msgs.camera_info_pb2 import CameraInfo
from automsgs.msgs.sensor_msgs.image_pb2 import Image
from automsgs.msgs.sensor_msgs.imu_pb2 import Imu
from automsgs.msgs.sensor_msgs.laser_scan_pb2 import LaserScan


class BridgeRunner:
    def __init__(self, cfg: dict, max_steps: Optional[int] = None) -> None:
        if autolink is None:
            raise ImportError("autolink Python package is required")
        self.cfg = cfg
        self.max_steps = max_steps
        self.clock = SimClock()
        spawn = cfg["scene"]["spawn"]
        self.drive = Drive(
            max_linear=cfg["robot"]["max_linear"],
            max_angular=cfg["robot"]["max_angular"],
            watchdog_sec=cfg["watchdog_sec"],
            x=float(spawn[0]),
            y=float(spawn[1]),
            yaw=float(spawn[2]),
        )
        self.world = create_world(cfg)
        self.world.reset(*self.drive.pose())
        self.lidar = Lidar(
            angle_min=cfg["lidar"]["angle_min"],
            angle_max=cfg["lidar"]["angle_max"],
            num_beams=cfg["lidar"]["num_beams"],
            range_min=cfg["lidar"]["range_min"],
            range_max=cfg["lidar"]["range_max"],
        )
        self.rgbd = Rgbd()
        self.odom = WheelOdometry(noise_std=float(cfg["noise"]["odom"]))
        self.imu_est = ImuEstimator()
        self.imu_est.reset(yaw=self.drive.pose()[2], t=0.0)
        self.truth = GroundTruth()
        autolink.init("autosim")
        self.node = autolink.Node("autosim_bridge")
        types = {
            "cmd_vel": TwistStamped,
            "scan": LaserScan,
            "rgb": Image,
            "depth": Image,
            "camera_info": CameraInfo,
            "imu": Imu,
            "odom": Odometry,
            "gt_pose": PoseStamped,
        }
        if not cfg["truth"]["enabled"]:
            types = {k: v for k, v in types.items() if k != "gt_pose"}
        self.bridge = Bridge(self.node, cfg["channels"], types)
        self._acc = {"scan": 0.0, "rgb": 0.0, "imu": 0.0}

    def run(self) -> None:
        dt = 1.0 / float(self.cfg["rates"]["control_hz"])
        steps = 0
        try:
            while not autolink.is_shutdown():
                self._cycle(dt)
                steps += 1
                if self.max_steps is not None and steps >= self.max_steps:
                    break
                time.sleep(dt)
        finally:
            self.world.close()
            autolink.shutdown()

    def _cycle(self, dt: float) -> None:
        t = self.clock.now()
        reader = self.bridge.cmd_reader
        if hasattr(reader, "has_msg") and reader.has_msg():
            msg = reader.get_msg()
            v, w = parse_cmd_vel(msg)
            self.drive.set_twist(v, w, t=t)
        x, y, yaw = self.drive.step(dt=dt, t=t + dt)
        self.world.set_pose(x, y, yaw)
        self.world.step()
        self.clock.tick(dt)
        stamp = self.clock.stamp()
        frames = self.cfg["robot"]["frames"]
        v, w = self.drive.velocity()

        ox, oy, oyaw = self.odom.update(x, y, yaw)
        self.bridge.publish(
            "odom",
            encode_odometry(
                ox, oy, oyaw, v, w, stamp, frames["odom"], frames["base"]
            ),
        )

        self._acc["scan"] += dt
        self._acc["rgb"] += dt
        self._acc["imu"] += dt

        if self._acc["scan"] >= 1.0 / self.cfg["rates"]["scan_hz"]:
            self._acc["scan"] = 0.0
            ranges = self.lidar.sample(self.world)
            self.bridge.publish(
                "scan",
                encode_laser_scan(
                    ranges=ranges,
                    stamp=stamp,
                    frame_id=frames["laser"],
                    angle_min=self.lidar.angle_min,
                    angle_max=self.lidar.angle_max,
                    angle_increment=self.lidar.angle_increment,
                    range_min=self.lidar.range_min,
                    range_max=self.lidar.range_max,
                    scan_time=1.0 / self.cfg["rates"]["scan_hz"],
                ),
            )

        if self._acc["rgb"] >= 1.0 / self.cfg["rates"]["rgb_hz"]:
            self._acc["rgb"] = 0.0
            rgb, depth = self.rgbd.sample(self.world)
            self.bridge.publish(
                "rgb",
                encode_image(rgb, stamp, frames["camera"], "rgb8"),
            )
            # 16UC1-ish: publish float depth as 32FC1 bytes
            self.bridge.publish(
                "depth",
                encode_image(depth.astype(np.float32), stamp, frames["camera"], "32FC1"),
            )
            h, w_img = rgb.shape[:2]
            fx = fy = 0.5 * w_img
            cx, cy = 0.5 * w_img, 0.5 * h
            k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            self.bridge.publish(
                "camera_info",
                encode_camera_info(stamp, frames["camera"], w_img, h, k),
            )

        if self._acc["imu"] >= 1.0 / self.cfg["rates"]["imu_hz"]:
            self._acc["imu"] = 0.0
            gyro_z, ax, ay = self.imu_est.update(yaw, v, self.clock.now())
            half = 0.5 * yaw
            quat = (0.0, 0.0, math.sin(half), math.cos(half))
            self.bridge.publish(
                "imu",
                encode_imu(stamp, frames["imu"], quat, (0.0, 0.0, gyro_z), (ax, ay, 9.81)),
            )

        if self.cfg["truth"]["enabled"]:
            tx, ty, tyaw = self.truth.sample(x, y, yaw)
            self.bridge.publish(
                "gt_pose",
                encode_pose_stamped(tx, ty, tyaw, stamp, frames["map"]),
            )


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(prog="autosim")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "default.yaml",
    )
    args = parser.parse_args(argv)
    cfg = load_config(args.config)
    BridgeRunner(cfg).run()


if __name__ == "__main__":
    main()
```

```python
# autosim/autosim/__main__.py
from autosim.runner import main

if __name__ == "__main__":
    main()
```

```python
# autosim/examples/run_bridge.py
#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from autosim.runner import main

if __name__ == "__main__":
    main(["--config", str(ROOT / "config" / "default.yaml")])
```

- [ ] **Step 4: 修烟雾测试中的 create_world 注入点**

确保 `runner.py` 顶层 `from autosim.world import create_world`，测试里 `monkeypatch.setattr(runner_mod, "create_world", ...)` 生效。

Run: `cd autosim && pytest tests/ -v`

Expected: 全部 PASS（codec 需 automsgs）

- [ ] **Step 5: Commit**

```bash
git add autosim/autosim/bridge.py autosim/autosim/runner.py autosim/autosim/__main__.py \
  autosim/examples/run_bridge.py autosim/tests/test_runner_smoke.py
git commit -m "feat(autosim): add autolink bridge runner"
```

---

### Task 10: 文档收尾与验收

**Files:**
- Modify: `autosim/README.md`
- Optional: `config/simulation/simulation.lua` 仅加注释指针（非必须）

- [ ] **Step 1: 更新 README** 补充：通道表、真值开关、`backend: habitat` 示例、测试命令 `pytest tests -v`、与 Cartographer/Atlas 联调前提（先起 autosim 再起定位节点）

- [ ] **Step 2: 全量测试**

Run: `cd autosim && pytest tests/ -v`

Expected: PASS

- [ ] **Step 3: 对照 spec 成功标准自检**

1. `python -m autosim` 可启动（Mock 或 Habitat）
2. `/cmd_vel` → 运动
3. 发布 scan/camera/imu/odom
4. `truth.enabled` 控制 `/gt/pose`
5. 无 Habitat 时单元测试通过

- [ ] **Step 4: Commit**

```bash
git add autosim/README.md
git commit -m "docs(autosim): finalize README and usage"
```

---

## Spec Coverage Checklist

| Spec 项 | Task |
|---------|------|
| 分层桥模块结构 | 1–9 |
| 通道与 proto | 5, 9 |
| 默认/可覆盖配置 | 2 |
| 真值可选 | 6, 9 |
| cmd_vel watchdog | 4, 9 |
| minimal + habitat 场景 | 7, 8 |
| Mock CI / 无 Habitat 测试 | 7, 9 |
| 非目标（不做 RPC 导航等） | 未实现（正确） |

## Self-Review Notes

- `/cmd_vel` 类型以 `TwistStamped` 注册；`parse_cmd_vel` 兼容 `Twist`
- `Odometry.pose` 为 `PoseWithCovariance`，其内 `pose` 为 `PoseStamped`（`msg.pose.pose.pose.position`）
- Habitat 坐标：平面 `y` 映射到 Habitat `z`（Y-up）；实现时保持 Drive 平面约定不变
- `create_world`：`habitat` 失败必抛；`minimal` 可回退 Mock
