# autosim 2D/3D LiDAR (cast_ray) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split `habitat.sensors.lidar` into parallel `lidar_2d` / `lidar_3d`, sample via Habitat `cast_ray` (mock fallback), publish `LaserScan` and `PointCloud2`.

**Architecture:** Config drives two independent sensor blocks; `Simulator.laser_ranges` / `lidar_points` cast rays in agent frame; `Sensors` wraps sampling + clip/noise; `Messages.encode_point_cloud2` packs xyz; `Runner` gates publish on `enabled` and per-sensor rate.

**Tech Stack:** Python 3.10+, NumPy, Habitat-Sim (optional), automsgs `*_pb2` (local `.deps` stubs), pytest

**Spec:** `docs/superpowers/specs/2026-08-18-autosim-lidar-2d-3d-design.md`

---

## File Map

| Path | Responsibility |
|------|----------------|
| `autosim/config/default.yaml` | `lidar_2d` / `lidar_3d` blocks |
| `autosim/autosim/config.py` | Validate keys, channels, geometry; `channel_map` `scan`/`points` |
| `autosim/autosim/simulator.py` | `laser_ranges`, `lidar_points`; remove public `depth_ring` |
| `autosim/autosim/sensors.py` | Hold 2D/3D params; `sample_laser` / `sample_points` |
| `autosim/autosim/messages.py` | `encode_point_cloud2` |
| `autosim/autosim/bridge.py` | Writer key `points` |
| `autosim/autosim/runner.py` | Dual lidar rates + `enabled` gating |
| `autosim/.deps/python/.../point_field_pb2.py` | Stub if missing |
| `autosim/.deps/python/.../point_cloud2_pb2.py` | Stub if missing |
| `autosim/tests/test_config.py` | YAML + validation |
| `autosim/tests/test_simulator.py` | Mock 2D/3D shapes |
| `autosim/tests/test_messages.py` | PointCloud2 packing |
| `autosim/tests/test_runner_smoke.py` | Enabled publish paths |
| `autosim/README.md` | Channel table |

---

### Task 1: PointCloud2 test stubs

**Files:**
- Create: `autosim/.deps/python/automsgs/msgs/sensor_msgs/point_field_pb2.py`
- Create: `autosim/.deps/python/automsgs/msgs/sensor_msgs/point_cloud2_pb2.py`

- [ ] **Step 1: Generate stubs from protos**

```bash
cd /Users/quandy/Workspace/github/autonomy
python -c "from google.protobuf import descriptor_pb2"  # ensure protobuf installed
protoc -I automsgs/proto \
  --python_out=autosim/.deps/python \
  automsgs/proto/msgs/sensor_msgs/point_field.proto \
  automsgs/proto/msgs/sensor_msgs/point_cloud2.proto
```

If `protoc` missing, hand-write minimal `*_pb2` matching existing laser_scan stub style (serialized DESCRIPTOR).

- [ ] **Step 2: Verify import**

```bash
cd autosim && PYTHONPATH=.deps/python python -c "from automsgs.msgs.sensor_msgs.point_cloud2_pb2 import PointCloud2; print(PointCloud2())"
```

Expected: empty message prints without error.

- [ ] **Step 3: Commit**

```bash
git add autosim/.deps/python/automsgs/msgs/sensor_msgs/point_*.py
git commit -m "test(autosim): add PointCloud2 pb2 stubs for lidar_3d"
```

---

### Task 2: Config — split lidar_2d / lidar_3d

**Files:**
- Modify: `autosim/config/default.yaml`
- Modify: `autosim/autosim/config.py`
- Modify: `autosim/tests/test_config.py`

- [ ] **Step 1: Failing test — default yaml loads new keys**

In `test_config.py`, update `minimal_config()` and assert:

```python
def test_load_default_yaml():
    settings = Config.load(ROOT / "config" / "default.yaml")
    assert settings.channel_map()["scan"] == "/scan"
    assert settings.channel_map()["points"] == "/points"
    assert "lidar" not in settings["habitat"]["sensors"]
    assert settings["habitat"]["sensors"]["lidar_2d"]["enabled"] is False
    assert settings["habitat"]["sensors"]["lidar_3d"]["vertical"]["num_rings"] == 16
```

- [ ] **Step 2: Run — expect FAIL (KeyError lidar_2d / missing points)**

```bash
cd autosim && pytest tests/test_config.py::test_load_default_yaml -v
```

- [ ] **Step 3: Update `default.yaml`** — replace `lidar:` with spec §2 blocks (both `enabled: false`).

- [ ] **Step 4: Update `Config`**

```python
REQUIRED_SENSOR_KEYS = ("lidar_2d", "lidar_3d", "camera", "imu", "odom")

def channel_map(self):
    ...
    "scan": sensors["lidar_2d"]["channel"],
    "points": sensors["lidar_3d"]["channel"],
    ...

# In validate: for each lidar_* require enabled bool; validate beams/rings/angles/ranges;
# channel list uses lidar_2d.channel and lidar_3d.channel
```

- [ ] **Step 5: Fix `minimal_config` helper + pass tests**

```bash
cd autosim && pytest tests/test_config.py -v
```

- [ ] **Step 6: Commit**

```bash
git commit -m "feat(autosim): split lidar config into lidar_2d and lidar_3d"
```

---

### Task 3: Messages.encode_point_cloud2

**Files:**
- Modify: `autosim/autosim/messages.py`
- Modify: `autosim/tests/test_messages.py`

- [ ] **Step 1: Failing test**

```python
def test_encode_point_cloud2_xyz():
    points = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
    msg = Messages.encode_point_cloud2(points, (1, 0), "lidar_link")
    assert msg.height == 1
    assert msg.width == 2
    assert msg.point_step == 12
    assert msg.row_step == 24
    assert msg.is_dense is False
    assert [f.name for f in msg.fields] == ["x", "y", "z"]
    data = np.frombuffer(msg.data, dtype=np.float32).reshape(2, 3)
    np.testing.assert_allclose(data, points)
```

- [ ] **Step 2: Implement**

```python
@classmethod
def encode_point_cloud2(cls, points, stamp, frame_id) -> PointCloud2:
    # points: Nx3 float32; empty N=0 allowed
    # fields x,y,z FLOAT32 offsets 0,4,8; point_step=12; height=1; width=N; is_dense=False
```

- [ ] **Step 3: pytest pass + commit**

```bash
git commit -m "feat(autosim): encode PointCloud2 xyz"
```

---

### Task 4: Simulator raycast APIs (mock first)

**Files:**
- Modify: `autosim/autosim/simulator.py`
- Modify: `autosim/tests/test_simulator.py`

- [ ] **Step 1: Failing tests**

```python
def test_mock_laser_ranges_shape():
    sim = Simulator(backend="minimal", width=64, height=48, use_mock=True,
                    settings={"habitat": {"path": "", "spawn": [0, 0, 0]}})
    ranges = sim.laser_ranges(angle_min=-np.pi, angle_max=np.pi, num_beams=36, range_max=10.0)
    assert ranges.shape == (36,)
    assert np.allclose(ranges, 5.0)

def test_mock_lidar_points_rings():
    sim = Simulator(..., use_mock=True)
    pts = sim.lidar_points(
        h_min=-np.pi, h_max=np.pi, h_beams=8,
        v_min=-0.2, v_max=0.2, v_rings=2, range_max=10.0,
    )
    assert pts.ndim == 2 and pts.shape[1] == 3
    assert pts.shape[0] == 8 * 2
```

- [ ] **Step 2: Implement mock + Habitat cast_ray path**

```python
def laser_ranges(self, angle_min, angle_max, num_beams, range_max) -> np.ndarray: ...
def lidar_points(self, h_min, h_max, h_beams, v_min, v_max, v_rings, range_max) -> np.ndarray: ...
# Habitat: habitat_sim.geo.Ray / session.cast_ray; map planar (x,y,yaw) to world
# Delete public depth_ring (update any callers)
```

Habitat hit handling (non-mock):
- origin = `(x, 0, y)` in Habitat
- 2D direction: rotate `(cos(yaw+θ), 0, sin(yaw+θ))` — match existing set_pose convention
- 3D: include pitch `ψ` in world direction; transform hit to sensor frame (x forward, y left, z up in bridge plane → convert consistently)
- miss → 2D `range_max`; 3D skip

- [ ] **Step 3: pytest + commit**

```bash
git commit -m "feat(autosim): cast_ray laser_ranges and lidar_points"
```

---

### Task 5: Sensors wrapper

**Files:**
- Modify: `autosim/autosim/sensors.py`
- Modify: `autosim/tests/test_simulator.py` (or `test_sensors` via existing)

- [ ] **Step 1: Extend Sensors** to store 2D params + optional 3D dict; `sample_laser` calls `laser_ranges` + clip + noise; `sample_points` calls `lidar_points` + noise.

```python
def sample_points(self, simulator) -> np.ndarray:
    return simulator.lidar_points(...)
```

- [ ] **Step 2: Update tests that used `depth_ring` / old Sensors ctor**

- [ ] **Step 3: commit**

```bash
git commit -m "feat(autosim): Sensors sample_laser/sample_points via cast_ray"
```

---

### Task 6: Bridge + Runner dual lidar

**Files:**
- Modify: `autosim/autosim/bridge.py` — add `"points"` to writer keys
- Modify: `autosim/autosim/runner.py`
- Modify: `autosim/tests/test_runner_smoke.py`
- Modify: `autosim/README.md`

- [ ] **Step 1: Runner behavior**

```python
# Only create LaserScan/PointCloud2 writers when corresponding enabled
# scan_elapsed / points_elapsed independent
# if not lidar_2d["enabled"]: skip sample+publish
# if not lidar_3d["enabled"]: skip
# odom still always published (unless later gated; out of scope)
```

Also gate camera/imu/odom on their `enabled` if present in yaml (camera/imu/odom already have `enabled` in default.yaml) — **only lidar_2d/lidar_3d required by this spec**; leave other sensors as today (always publish) unless already broken.

- [ ] **Step 2: Smoke test**

```python
def test_runner_publishes_scan_and_points_when_enabled(monkeypatch):
    settings = Config.load(...)
    settings.data["habitat"]["sensors"]["lidar_2d"]["enabled"] = True
    settings.data["habitat"]["sensors"]["lidar_3d"]["enabled"] = True
    # reduce beams for speed
    settings.data["habitat"]["sensors"]["lidar_2d"]["num_beams"] = 16
    settings.data["habitat"]["sensors"]["lidar_3d"]["horizontal"]["num_beams"] = 8
    settings.data["habitat"]["sensors"]["lidar_3d"]["vertical"]["num_rings"] = 2
    ...
    runner.run()
    assert "/scan" in writers and len(msgs) >= 1
    assert "/points" in writers and len(msgs) >= 1
```

- [ ] **Step 3: Full suite**

```bash
cd autosim && pytest tests/ -q
```

Expected: all green.

- [ ] **Step 4: Commit**

```bash
git commit -m "feat(autosim): publish parallel 2D LaserScan and 3D PointCloud2"
```

---

## Spec coverage checklist

| Spec item | Task |
|-----------|------|
| Parallel lidar_2d / lidar_3d config | 2 |
| channel_map scan/points | 2 |
| LaserScan / PointCloud2 | 3, 6 |
| cast_ray sampling | 4 |
| Mock shapes | 4 |
| enabled gating | 6 |
| PointCloud2 stub | 1 |
| README | 6 |

## Self-review

- No placeholders in steps
- `depth_ring` removal called out in Task 4–5
- Signatures `laser_ranges` / `lidar_points` consistent across Tasks 4–6
