# 09 · 测试

以 pytest 为主；Isaac 全链路需 Kit，单测用 mock / 纯 numpy。

| 相关 | 链接 |
|------|------|
| 架构 | [02](02_architecture.md) |
| 排错 | [10](10_troubleshooting.md) |

---

## 1. 单元测试

```bash
cd src/autonomy/autosim
python3 -m pytest tests -v
```

| 范围 | 示例 |
|------|------|
| 配置 / 通道 | `test_config.py` |
| Fake 联调 | `test_fake.py` · `test_runner_smoke.py`（需 automsgs） |
| Isaac 无 Kit | `test_backend.py`：传感器布局、floorplan、PhysX block_motion mock |
| Traffic / Policy | `test_isaac_traffic_policy.py`（Arena、激光融合、PolicyDriver 配置） |
| 消息编码 | `test_messages.py` |

```bash
# 避开 ROS launch_testing 插件冲突时：
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3.12 -m pytest tests/test_isaac_traffic_policy.py -q
```

依赖：`automsgs` Python 绑定；仓库不再附带 `stubs/`。

---

## 2. Isaac 冒烟（需 Isaac Python）

```bash
ISAACLAB_PATH=/workspace/isaaclab \
  /workspace/isaaclab/isaaclab.sh -p \
  src/autonomy/autosim/scripts/smoke_isaac_camera.py
```

期望输出含 `CAMERA_OK`（RGB 非空且深度有限点足够）。

---

## 3. 手工验收清单

| 项 | 期望 |
|----|------|
| Fake sim + teleop | 机器人在 Autoviz 中移动 |
| Isaac RGB / depth | 图像稳定；深度为伪彩而非乱闪 |
| `/map` | 室内自由空间与墙体合理 |
| `/overall/map` | 家具轮廓可见（USD 采样） |
| 穿墙 | `block_motion: true` 时位姿停在障碍外 |
| `mode: slam` | 无 GT `/map`、无 `map→odom` |

---

## 4. 维护建议

- 新后端：至少 mock `set_pose` / `color_depth` 形状断言。
- 改发布契约：同步 [06](06_topics.md) 与 runner 单测。
- 大场景不进 CI；floorplan 单测用仓库内 `rooms.json`（若存在）或 skip。
