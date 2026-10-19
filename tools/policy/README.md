# Policy export & contract tools

配套 `autonomy/control/controller/policy_controller` 的训练→部署脚手架。

| 脚本 | 作用 |
|------|------|
| `export_policy_onnx.py` | 导出 toy / 自定义 PyTorch Twist 策略为 ONNX，可选生成 sidecar |
| `validate_policy_contract.py` | 校验 ONNX I/O ↔ `PolicyConfig`；`--checklist` 打印导出清单 |

## 冒烟

```bash
python3 tools/policy/export_policy_onnx.py --toy \
  --output autonomy/control/conf/models/policy_toy.onnx \
  --sidecar autonomy/control/conf/policy_config_toy_example.pb.txt

python3 tools/policy/validate_policy_contract.py \
  --config autonomy/control/conf/policy_config_toy_example.pb.txt \
  --onnx autonomy/control/conf/models/policy_toy.onnx

python3 tools/policy/validate_policy_contract.py --checklist
```

依赖：`torch`（导出）、`onnx`（校验）。

## 契约 / 单测入口

```bash
# 一键：checklist +（可选）ONNX 校验 + Bazel 单测
./tools/policy/run_policy_checks.sh

# 或分步
python3 tools/policy/validate_policy_contract.py --checklist

# CMake
ctest -R control.policy_ -V

# Bazel
bazel test //autonomy/control:policy_action_schema_test \
           //autonomy/control:policy_safety_filter_test \
           //autonomy/control:policy_path_encoder_test \
           //autonomy/control:policy_frame_align_test
```

## 量产运行建议（PolicyControllerOptions）

| 字段 | 建议 |
|------|------|
| `infer_timeout` | ≤ 控制周期（如 20Hz → 0.04） |
| `require_model` | `true` |
| `require_frame_match` | `true` |
| `robot_frame` | `base_link` |
| `transform_tolerance` | `0.1` |
| `max_consecutive_failures` | `3`–`5`（超限强制零速） |
| `collision_check` + `use_footprint_collision` | `true` |
| `relative_kinematic` | 训练若用相对目标则 `true` |

完整片段：`autonomy/control/conf/policy_controller_production.pb.txt`。

**帧契约**：path 与 ego pose 经 TF 对齐到同一 frame 后，path encoder 再转机器人系；激光/图像/点云 `frame_id` 须等于 `robot_frame`（C++ 不变换）。kinematic 仍为 pose-frame 绝对位姿，训练地图原点需稳定。

**热加载**：`reload_config_allow_prefix` 限制 topic 下发路径；生产 overlay 见 `policy_controller_production.pb.txt`。
