# 02 · 架构

主循环与后端解耦：`Runner` 只依赖 `WorldBackend` + **capability**；引擎互不 import。数据采集走独立 `dataset/` 层（LeRobot）。

| 相关 | 链接 |
|------|------|
| 后端 | [04](04_backends.md) |
| 数据集 | [11](11_dataset.md) |
| 源码 | `autosim/runtime/` · `backends/` · `scene/` · `bridge/` · `robot/` · `dataset/` |

---

## 1. 包树

```text
autosim/
├── runtime/              # 编排：Runner · SensorWorker · Config · paths
├── backends/             # 契约 + 引擎（引擎子包互不 import）
│   ├── protocol.py       # WorldBackend
│   ├── registry.py       # create_backend / register
│   ├── capabilities.py
│   ├── habitat/
│   ├── fake/
│   └── isaac/
├── scene/                # 与引擎无关的几何与动态体
│   ├── map.py · floorplan.py  # OccupancyGrid / PLY · rooms.json
│   ├── world.py · traffic.py · arena.py · field.py · cast.py
│   ├── gen/              # 程序化静态地图（Scenario · Volume · Stage …）
│   └── agents/           # Obstacle · Pedestrian · Force · Behavior · Agents
├── bridge/               # Autolink I/O + 传感采样
│   # Bridge · Messages · Clock · Teleop · Sensors · cameras · stitch
├── robot/                # plant · profiles · urdf · policy/
└── dataset/              # LeRobot v2.1 采集
```

| 易混 | 含义 |
|------|------|
| `backends/` 根模块 | 抽象契约（protocol / registry） |
| `backends/{habitat,fake,isaac}/` | 具体引擎 |
| `scene/gen/` | 程序化静态场景生成 |
| `scene/map.py` · `floorplan.py` | 真值 OccupancyGrid / PLY；InteriorAgent rooms.json |

---

## 2. 分层与数据流

```text
teleop /cmd_vel / PolicyDriver
            │
            ▼
┌───────────────────────────────────────┐
│  runtime.Runner                       │
│  Robot · bridge.Sensors · scene.World │
│  bridge.Bridge · dataset.Collector    │
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
| `runtime` | 配置、周期积分、传感发布、录制钩子 |
| `bridge` | 采样噪声与 Autolink 编解码 |
| `scene` | 占据 / 环境 / 动态体（无 Kit / Habitat API） |
| `robot` | 差速植物、URDF、可选 ONNX 策略 |
| `dataset` | Episode 缓冲 → parquet / video / meta |
| `backends.*` | 唯一接触 Habitat / Kit / 过程射线的代码 |

---

## 3. 一拍（nav）

1. 读 `/cmd_vel` 或 Policy → `Robot.step`  
2. `WorldBackend.set_pose` / `step`；`World.step(dt, robot=…)`  
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
| 包 / 文件 | 名词；一文件一类（实现不放 `__init__`） | `runner.py` → `Runner` |
| 属性 / 变量 | 名词，无前缀 `_` | `sensor_worker`, `episode` |
| 函数 / 方法 | 动词 + 名词 | `poll_command`, `build_feeds` |
| 常量 | 全大写 | `CAPABILITY_DEFAULTS` |

保留 Python 惯例：`__init__` / `__all__` 等 dunder；循环中故意忽略的 `_` 除外。
