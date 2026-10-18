# 生产完备清单

面向宿主集成（无 ROS）：单线程估计 + 配置驱动模态 + 可观测软失败。

## 1. 已完备能力

| 能力 | 入口 |
|------|------|
| LO / LIO / LIVO | `LivoOdometry::Process` |
| 退化 + 各向异性 | `enable_degeneracy`（默认 on） |
| 动态过滤 | `frontend.dynamic_filter` |
| Confidence gating | `enable_confidence_gating` |
| 强度辅助 | `enable_intensity_assist`（默认 on） |
| 异步回环 + SE(3) PG + ΔT 注入 | `loop_closure.enable` |
| 跨会话 SC | 地图包 `keyframes/` + `RegisterPriorSession` |
| BnB se2/se3_yaw/se3 | `relocalization.*` |
| 地图包 v3 | `MapBuilder::Save/Load` |
| `ProcessStatus` / `soft_skip` | `OdometryOutput` |
| 配置生产校验 | `LoadLivoOdometryConfig` |

## 2. 宿主契约

见 [10_usage.md §1](10_usage.md)。要点：

- `backend_threads == 1`
- `out.soft_skip==true`：包已消费但未更新地图
- `InitLogging` 在进程启动时调用

## 3. 配置门禁

- `camera` → `calib_file` 必须存在
- **禁止**无 LiDAR 的 ONLY_VIO
- `semantic` + `onnx` → 非空 `semantic_model`
- `relocalization.trigger` ∈ `{tracking_lost, manual, cross_session_load}`
- `loop_closure.enable` → `async=true`

推荐：`config/livo_esikf.yaml` 中退化/强度/mapping/reloc 默认 on；门控/回环/自清洁按场景打开。

## 4. 验证命令

```bash
tools/argus/run_stub_tests.sh

slice_kitti --src $KITTI/sequences/00 --dst testdata/kitti_real --frames 3
verify_kitti testdata/kitti_real
offline_kitti --kitti testdata/kitti_real --filter-only

offline_livo --synth config/livo_esikf.yaml

cmake -S autonomy/localization/argus -B build/argus \
  -DARGUS_BUILD_TESTS=ON -DARGUS_BUILD_APPS=ON
cmake --build build/argus && ctest --test-dir build/argus
```

## 5. 已知边界

| 项 | 说明 |
|----|------|
| Bag/mcap | 宿主提供；Argus 只消费 `Add*` |
| ONNX CI | 可选；无 ORT 时用 `mask_file` |
| 独立单目 VO | 不做；生产模态 LO/LIO/LIVO |
| 大规模稀疏 PG | 当前 SE(3) GN；超大图可外挂 |

## 6. 发布检查

- [ ] stub workflow 绿
- [ ] `offline_livo --synth` 出 `map.xyz` + `map_meta`
- [ ] `verify_kitti` 对真实序列通过
- [ ] 宿主单线程 `Process`，监控 `ProcessStatus`
- [ ] 部署 YAML 通过 `LoadLivoOdometryConfig`（无 error）
