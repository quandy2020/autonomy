# 02 · 架构

主循环与后端解耦：`Runner` 只依赖 `WorldBackend` + **capability**；引擎互不 import。数据采集走独立 `dataset/` 层（LeRobot）。

| 相关 | 链接 |
|------|------|
| 后端 | [04](04_backends.md) |
| 数据集 | [11](11_dataset.md) |
| 源码 | `autosim/runtime/` · `autosim/backend/` · `autosim/backends/` · `autosim/dataset/` |

---

## 1. 专业化包树

```text
autosim/
├── runtime/           # 编排：Runner · SensorWorker
├── backend/           # 契约：Protocol · registry · capabilities
├── backends/          # 引擎（互不耦合）
│   ├── habitat/       # Habitat-Sim Simulator
│   ├── fake/          # FakeWorld · map/sensor/robot
│   └── isaac/         # USD / PhysX / 相机
├── scenario/          # 纯几何：静态图 · Traffic · 障碍/行人
├── sensing/           # Sensors · cameras · stitch
├── io/                # Bridge · Messages · Clock · Teleop
├── dataset/           # LeRobot v2.1 采集
├── policy/            # 可选 ONNX Twist
├── robot/             # plant · profiles · urdf
├── map/               # 共享真值占据 / PLY
└── config/            # YAML 加载与校验
```

---

## 2. 分层与数据流

```text
teleop /cmd_vel / PolicyDriver
            │
            ▼
┌───────────────────────────────────────┐
│  runtime.Runner                       │
│  Robot · sensing.Sensors · Traffic    │
│  io.Bridge · dataset.Collector        │
└───────────────┬───────────────────────┘
                ▼
┌───────────────────────────────────────┐
│  WorldBackend + capability(...)       │
├──────────┬──────────┬─────────────────┤
│ habitat  │  fake    │  isaac          │
└──────────┴──────────┴─────────────────┘
                │
                ▼
         dataset → LeRobot 目录
```

| 层 | 职责 |
|----|------|
| `runtime` | 周期积分、传感发布、录制钩子 |
| `sensing` / `io` | 采样噪声与 Autolink 编解码 |
| `scenario` | 与引擎无关的占据 / 动态体 |
| `dataset` | Episode 缓冲 → parquet / video / meta |
| `backends.*` | 唯一接触 Habitat / Kit / 过程射线的代码 |

---

## 3. 一拍（nav）

1. 读 `/cmd_vel` 或 Policy → `Robot.step`  
2. `WorldBackend.set_pose` / `step`；`Traffic.step`  
3. 按 rate 采 scan / camera → Bridge 发布  
4. `DatasetCollector.tick`（若 `habitat.dataset.enabled`）  

---

## 4. 扩展后端

1. `backends/<name>/` 实现协议并声明 `capabilities`  
2. `register("name", factory)` — **不要**在 Runner 按名字分支  
3. YAML + [04](04_backends.md) + 测试  

---

## 5. 命名约定

| 类别 | 规则 | 示例 |
|------|------|------|
| 属性 / 变量 | 名词，无前缀 `_` | `last_scan`, `sensor_worker`, `episode` |
| 函数 / 方法 | 动词 + 名词 | `poll_command`, `flush_episode`, `build_feeds` |
| 常量 | 全大写 | `CAPABILITY_DEFAULTS`, `MAX_DEPTH_M` |
| 日志 | `logger` | `logger = logging.getLogger(__name__)` |

保留 Python 惯例：`__init__` / `__all__` 等 dunder；循环中故意忽略的 `_` 除外。
