# 11 · 数据采集（LeRobot）

AutoSim 可将 teleop / 策略闭环轨迹写成 **LeRobot v2.1** 目录，供模仿学习（BC / ACT / Diffusion Policy 等）使用。

| 相关 | 链接 |
|------|------|
| 架构 | [02](02_architecture.md) |
| 配置 | [05](05_configuration.md) |
| 源码 | `autosim/dataset/` |

---

## 1. 目录与数据流

```text
cmd_vel / PolicyDriver
        │
        ▼
  runtime.Runner.cycle
        │
        ▼
  dataset.DatasetCollector.tick   ← fps 门控
        │
        ▼
  Episode (frames) ──flush──► LeRobotWriter
                                ├─ data/chunk-*/episode_*.parquet
                                ├─ videos/.../observation.images.rgb/*.mp4
                                └─ meta/{info.json, episodes.jsonl, tasks.jsonl}
```

---

## 2. 特征（差速底盘）

| 键 | 形状 | 含义 |
|----|------|------|
| `observation.state` | `[5]` | `x, y, yaw, vx, wz` |
| `action` | `[2]` | 指令 `vx, wz`（teleop / policy） |
| `observation.images.rgb` | `H×W×3` | 主相机（可选） |
| `observation.laser` | `[N]` | 2D 激光（可选） |
| `timestamp` / `frame_index` / `episode_index` / `index` / `task_index` / `next.done` | 标量 | LeRobot 标准索引列 |

---

## 3. 配置

```yaml
habitat:
  dataset:
    enabled: true
    root: datasets/fake_demo
    repo_id: autosim/fake_demo
    task: navigate
    fps: 10.0
    episode_frames: 200   # 0 = 进程退出时整段 flush
    max_episodes: 0       # 0 = 不限
    include_image: true
    include_laser: true
    include_people: false   # true → observation.people + episode_*_people.jsonl
```

别名：`habitat.record` 与 `habitat.dataset` 等价。

依赖：`pip install pyarrow`（parquet）；视频优先 OpenCV（`opencv-python`），缺失时回退 PNG/PPM 到 `images/`。

---

## 4. 加载示例

```python
# 需安装 huggingface lerobot
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

ds = LeRobotDataset("autosim/fake_demo", root="datasets/fake_demo")
print(ds[0]["action"], ds[0]["observation.state"])
```

或直接读 parquet / `meta/info.json` 自建 DataLoader。

---

## 5. 建议工作流

1. Fake / Isaac 开 `dataset.enabled`，键盘 teleop 录若干 episode。  
2. 检查 `meta/episodes.jsonl` 长度与 `videos/`。  
3. 用 LeRobot / 自研 BC 训练；部署可用 `policy_controller` 或 `habitat.policy` ONNX。
