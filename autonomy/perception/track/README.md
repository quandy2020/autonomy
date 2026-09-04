# Track / human following

地面机器人端到端人体跟随，兼容 **YOPO-Simple** 与 **YOPO-MINCO**（平面化适配）。

## 方案

| 项 | 选择 |
|----|------|
| 输出 | `/cmd_vel`（`TwistStamped`） |
| 感知 | 深度为主 |
| 跟踪头 | objectness + 角向非极大值抑制 |
| 轨迹表示 | `simple`（终端态）或 `minco`（2-piece 平面 MINCO_S3NU） |
| 仿真 | autosim |

## 目录

```
track/
  common/constants.hpp                     # 默认值、ONNX 契约、双模式通道数
  common/types.hpp                         # TrackResult / MincoBoundary
  primitive/lattice.*                      # yaw×range lattice
  primitive/minco_trajectory.*             # 平面 2-piece MINCO 求解/采样
  utils/...
  infer/track_inference_engine.*           # simple(6) / minco(12) 解码
  python/minco/                            # 训练可微 + 推理 Numpy MINCO
  python/...
```

## Runtime contract

| 模式 | ONNX `prediction` | 通道语义 |
|------|-------------------|----------|
| `simple` | `[B,6,V,H]` | yaw_offset, range_offset, vx, wz, score, objectness |
| `minco` | `[B,12,V,H]` | inner(2), tail_pos(2), tail_vel(2), tail_acc(2), durations(2), score, objectness |

共用输入：`depth` `[B,1,H,W]`，`observation` `[B,4,V,H]`。

MINCO 控制：解出多项式后在 `minco_sample_horizon_s` 采样速度 → 差速 `linear.x` / `angular.z`。

`radio_range_m` 配置字段语义为 planning horizon（兼容历史命名）。

## Python 训练 / 导出

```bash
cd src/autonomy/autonomy/perception/track/python
pip install -r requirements.txt

# Simple（默认）
PYTHONPATH=. python train.py --epochs 5
PYTHONPATH=. python export_onnx.py --checkpoint checkpoints/yopo_track_last.pt

# Planar MINCO（对齐 YOPO-MINCO 分支）
PYTHONPATH=. python train.py --trajectory-mode minco --epochs 5
PYTHONPATH=. python export_onnx.py --trajectory-mode minco \
  --checkpoint checkpoints/yopo_track_minco_last.pt \
  --output checkpoints/yopo_track_minco.onnx
```

在 `config/perception/track_yopo.lua` 设置：

```lua
trajectory_mode = "minco"
model_path = ".../yopo_track_minco.onnx"
minco_piece_duration_s = 1.0
acc_max_mps2 = 1.0
minco_sample_horizon_s = 0.5
```

## 一键启动（autosim）

```bash
./src/autonomy/autosim/scripts/run.sh sim
export PATH=$PWD/build/autonomy/bin:$PATH
export AUTOLINK_LAUNCH_PATH=$PWD/src/autonomy/autonomy/perception/launch
autolink launch start track.launch
```

无权重时 `allow_heuristic_fallback` 用扇区深度假设（simple 终端态），便于联调。
