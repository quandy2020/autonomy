# 测试与评测

## 1. 测试分层

```text
unit          → 数学、配置、工厂、状态机
integration   → SlamSystem 多模式 Step
dataset       → EuRoC / KITTI / … 离线
simulation    → Gazebo / AirSim（规划）
benchmark     → 精度 / 效率 / 鲁棒 / 热
```

代码位置：

| 层 | 路径 |
|----|------|
| 单元 | `test/unit/*_test.cpp`（自动收集） |
| 集成 | `test/integration/` |
| 数据集夹具 | `test/dataset/` |
| 仿真 | `test/simulation/` |
| 指标与 runner | `tools/benchmark/` |

## 2. 单元测试范围

| 用例（现有/规划） | 覆盖 |
|-------------------|------|
| `ParseFrontendMode` | 配置枚举 |
| `CreateFrontend` VO/VIO/LIO/LIVO | 工厂 |
| `SlamSystem` 合成 Step | 冒烟集成 |
| TransformTree Lookup | 外参（规划） |
| IMU 预积分短序列 | 数值（规划） |
| OutlierRejection 门控 | 融合（规划） |

运行：

```bash
ctest -R atla2 --test-dir build --output-on-failure
```

## 3. 集成与模式矩阵

| 模式 | 输入 | 后端 | 最低断言 |
|------|------|------|----------|
| VO | image | ceres | Step OK，valid odom |
| VIO | image+imu | ceres | 同上 |
| LIO | lidar+imu | iekf | 同上 + local_map 可空非崩 |
| LIVO | 全 | iekf | Degraded 可触发 |

## 4. 评测指标

| 脚本 | 指标 |
|------|------|
| `metrics/accuracy.py` | ATE / RPE / Scale（可选 evo） |
| `metrics/efficiency.py` | FPS / per_step_ms / RSS |
| `metrics/robustness.py` | 失败率 / 回环召回 |
| `metrics/thermal.py` | 温升 / 降频比 |

数据集配置：`tools/benchmark/configs/{euroc,uma_vi,mun_frl,kitti}.yaml`。

```bash
./tools/benchmark/runners/run_offline.sh euroc
./tools/benchmark/runners/run_embedded.sh mun_frl
```

报告目录：`tools/benchmark/reports/<name>/<timestamp>/`。

## 5. CI 建议

| 门禁 | 命令 | 阻断 |
|------|------|------|
| 编译 | ninja atla2_offline | 是 |
| 单测 | ctest -R atla2 | 是 |
| 冒烟四模式 | offline ×4 platforms | 是 |
| 精度回归 | accuracy vs 基线 json | 指标退化 >10% 阻断（规划） |

## 6. 真值与数据格式

- 轨迹：TUM（`t x y z qx qy qz qw`）或 KITTI  
- 事件：JSONL（`type: ok|lost|loop_*`）供 robustness  
- 标定真值与场景标签随数据集版本号管理  

辅助：`scripts/evaluate.py`（evo 封装）。

## 7. 性能基线（占位，上板后填）

| 平台 | 模式 | FPS | RSS(MB) | ATE(m) |
|------|------|-----|---------|--------|
| x86 desktop | VIO | | | |
| Orin / RK3588 | LIO | | | |

## 8. 相关文档

- [../deployment/verification.md](../deployment/verification.md)  
- [../../tools/benchmark/README.md](../../tools/benchmark/README.md)  
- [../requirements/analysis.md](../requirements/analysis.md)  
