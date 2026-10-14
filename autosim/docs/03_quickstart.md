# 03 · 快速开始

从工作区根目录启动；Isaac 需用 `scripts/run.sh`（自动切换 Isaac Lab Python）。

| 相关 | 链接 |
|------|------|
| 配置 | [05](05_configuration.md) |
| Isaac | [07](07_isaac.md) |
| 排错 | [10](10_troubleshooting.md) |

---

## 1. 依赖

| 项 | 说明 |
|----|------|
| 必需 | `numpy`、`PyYAML`、`protobuf`、`autolink` / `automsgs` Python |
| Fake | 无额外引擎 |
| Habitat | `pip install -e ".[habitat]"`（`habitat-sim`） |
| Isaac | 容器内 `/workspace/isaaclab` + Omniverse；`pip install -e ".[isaac]"` 仅文档化 extras |

构建包（可选）：

```bash
colcon build --packages-select autonomy
# 或
cd src/autonomy/autosim && pip install -e ".[dev]"
```

---

## 2. Fake（最快验证）

```bash
cd /workspace/autonomy
CONFIG=src/autonomy/autosim/config/fake.yaml \
  src/autonomy/autosim/scripts/run.sh sim
```

另开终端遥控：

```bash
src/autonomy/autosim/scripts/run.sh teleop
# w/x 线速度，a/d 角速度，space/s 停车；需 TTY 焦点
```

---

## 3. Habitat

```bash
# 编辑 config/default.yaml：habitat.path → 本地 GLB
CONFIG=src/autonomy/autosim/config/default.yaml \
  src/autonomy/autosim/scripts/run.sh sim
```

无网格时可开 `habitat.scenario.enabled` 生成过程场景（见包 README）。

---

## 4. Isaac

```bash
ISAACLAB_PATH=/workspace/isaaclab \
CONFIG=src/autonomy/autosim/config/isaac.yaml \
  src/autonomy/autosim/scripts/run.sh sim
```

| 环境变量 | 作用 |
|----------|------|
| `ISAACLAB_PATH` | Isaac Lab 根（默认 `/workspace/isaaclab`） |
| `AUTOSIM_ISAAC_HEADLESS=0` | 需要 Kit GUI 时关闭 headless |
| `CONFIG` | YAML 路径 |

场景：默认 InteriorAgent（需自行放到 `usd/worlds/InteriorAgent/`，见 [07](07_isaac.md)）。无资源时把 `habitat.path` 改为 `usd/worlds/simple_room/simple_room.usd`。

相机冒烟：

```bash
# 经 isaaclab.sh -p
ISAACLAB_PATH=/workspace/isaaclab \
  /workspace/isaaclab/isaaclab.sh -p \
  src/autonomy/autosim/scripts/smoke_isaac_camera.py
```

---

## 5. 与 Autoviz

1. 启动 sim（建议 `mode: nav`、`map.publish: true`）。
2. 启动 `autoviz`，Fixed Frame = `map`。
3. 订阅见 [06](06_topics.md) / [08](08_map_and_viz.md)。

演示动图：[`../images/isaac.gif`](../images/isaac.gif)。
