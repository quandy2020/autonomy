# 部署与验证

## 1. 构建部署拓扑

```text
autonomy 工作区
└── autonomy/localization          ← autonomy_glob_srcs 含 atla2 源码
    ├── CMakeLists.txt             ← add_subdirectory(atla2)
    └── atla2/
        ├── CMakeLists.txt         ← apps binaries + install config
        ├── config/                ← install → share/.../conf/atla2
        └── apps/
```

库目标：`autonomy_localization`  
可执行：`autonomy.localization.atla2_*`

## 2. 构建步骤

```bash
# 工作区已 cmake 配置后
ninja -C build autonomy.localization.atla2_offline
ninja -C build autonomy.localization.atla2_node
ninja -C build autonomy.localization.atla2_benchmark

# 或
./src/autonomy/autonomy/localization/atla2/scripts/build.sh
```

可选标定 CLI：`atla2_camera_imu_calib` 等（同 CMakeLists）。

安装配置：

```bash
ninja -C build install
# → share/autonomy/localization/conf/atla2/
```

## 3. 运行验证清单

### 3.1 冒烟（桌面）

```bash
./build/bin/autonomy.localization.atla2_offline \
  --config .../config/platforms/drone_livo.yaml --steps 100
```

期望：退出码 0，打印最终 pose。

对四平台各跑一遍：`drone_vo` / `drone_vision_only` / `drone_lidar_imu` / `drone_livo`（+ `vtol_livo_gps`）。

### 3.2 单测

```bash
# BUILD_TEST=ON 时
ctest -R atla2 --test-dir build --output-on-failure
```

### 3.3 评测 runner

```bash
cd .../atla2/tools/benchmark
./runners/run_offline.sh euroc
./runners/run_embedded.sh mun_frl   # 含温升采样
```

检查 `reports/<dataset>/<ts>/` 下 json + log。

## 4. 嵌入式部署

| 项 | 建议 |
|----|------|
| 交叉编译 | 使用工作区 ARM64 toolchain；FEATURES 保持 slam |
| 内存 | 下调 `max_local_map_points`、增大 `voxel_size` |
| 后端 | 优先 `iekf`；避免大窗口 Ceres |
| 实时 | 前端线程高优先级；关闭非必要日志 |
| 热 | 部署后跑 `run_embedded.sh`，关注 `throttle_ratio` |

降级：保证仅 IMU 或仅雷达时进程不崩（Degraded）。

## 5. 配置与版本

- 平台 YAML 与代码版本一并归档  
- 外参文件路径相对 `config/`  
- 启动失败优先查：YAML 解析、`CreateFrontend/Optimizer` 空指针  

## 6. 验证记录模板

| 日期 | 平台 | 二进制 | 结果 | 备注 |
|------|------|--------|------|------|
| | drone_livo | atla2_offline | PASS/FAIL | steps=100 |

## 7. 常见问题

| 现象 | 排查 |
|------|------|
| Init failed | YAML 路径、mode 拼写 |
| Step failed | 合成数据缺模态；前端骨架限制 |
| 找不到 binary | 未编 apps；`BIN_DIR` 不对 |
| 精度差 | 外参 / 时间同步 / 未 deskew |
