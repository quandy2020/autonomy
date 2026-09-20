# 钢镚 L1-W（智身 GENISOM / zsibot）— Autolink Component

独立库 `libautodriver_l1w.so`：覆盖 bundled **ZSL-1W HighLevel**（`autodriver/thirdparty/zsl1w`）全部控制/状态接口。

## 官方 API 映射

| HighLevel | autodriver |
|-----------|------------|
| `standUp` | mode `stand` / tool `stand` / `auto_stand` |
| `lieDown` | tool `lie`；Stop 时可选 `lie_on_stop` |
| `passive` | mode `estop` / tool `passive` |
| `move(vx,vy,wz)` | `/cmd_vel` + mode `wheel` / tool `move` |
| `crawl(vx,vy,wz)` | `/cmd_vel` + mode `walk` / tool `crawl` |
| `climb(vx,vy,wz)` | tool `climb` / `climb=vx,vy,wz` + `/cmd_vel`（非共享 mode） |
| `cancelCrawl` | tool `cancel_crawl` |
| `cancelClimb` | tool `cancel_climb` |
| `shakeHand` | tool `shake_hand` |
| `rearSquat` | tool `rear_squat` |
| `attitudeControl` | tool `attitude=r,p,y,h` |
| 位姿/速度/IMU/电量/ctrlmode/关节 | `ReadChassisState` → `/robot_state`（见下） |

## `/robot_state` 约定

| 字段 | 内容 |
|------|------|
| `pose` / `twist` | SDK 位姿、机体系速度、陀螺 |
| `battery_percent` | `getBatteryPower` |
| `localization_quality` | 由 `getBodyAcc` 相对 1g 的健康度估计 \[0,1\] |
| `map_name` | 调试串：`ctrl:…;gait:…;acc:…;rpy:…;j:…`（无独立 IMU/关节 proto 字段） |
| `active_cmd_id` | ChassisManager 写入的运行模式名（非 SDK） |

## SDK

优先使用仓库内：

```text
autodriver/thirdparty/zsl1w/
  include/zsl-1w/highlevel.h
  lib/aarch64/libmc_sdk_zsl_1w_aarch64.so
```

板端也可安装到 `/opt/genisom_l1_sdk`（`FindGenisomL1w.cmake` 会自动发现）。

```bash
cmake ... -DGenisomL1w_ROOT=/opt/genisom_l1_sdk   # 可选显式指定
```

## 测试

| 模式 | 说明 |
|------|------|
| simulate（默认） | `./build/bin/test_l1w_driver` — 不连狗 |
| 真机 SDK | 见下方环境变量；需已链接 GenisomL1w |

```bash
# 板端本机（Firefly eth0 == 192.168.168.168；sdk_config.target_ip 同值）
export AUTODRIVER_L1W_HW_TEST=1
export AUTODRIVER_L1W_HOST=192.168.168.168
export AUTODRIVER_L1W_LOCAL_IP=192.168.168.168
# AGX 遥控时改为 AGX IP，并与狗端 target_ip 一致，例如：
# export AUTODRIVER_L1W_LOCAL_IP=192.168.168.100
# export AUTODRIVER_L1W_HW_MOTION=1             # 微小 move/crawl
# export AUTODRIVER_L1W_HW_SPECIAL=1            # shake / squat / climb
./build/bin/test_l1w_driver --gtest_filter='L1wDriverHw*'
```

## Component

```text
mainboard -d dag/chassis_l1w.dag
```

| 项 | 值 |
|----|-----|
| `module_library` | `libautodriver_l1w.so` |
| `class_name` | `autodriver::chassis::l1w::L1wComponent` |
| 配置 | `config/chassis/l1w.yaml` |

### 通道示例

```bash
# 站立 / 轮式 / 匍匐（共享 mode 通道）
# → /chassis/mode
stand
wheel
walk

# L1-W 专用工具 → /chassis/tool
lie
passive
cancel_crawl
cancel_climb
climb
climb=0.1,0,0
shake_hand
rear_squat
attitude=0,0,0,0.1
```

网线 AGX↔RK3588：`host=192.168.168.168`，`local_ip=<AGX>`，狗端 `sdk_config.target_ip=<AGX>`。

## 网线联机（AGX ↔ RK3588）

| 端 | IP | 说明 |
|---|---|---|
| RK3588（狗有线） | `192.168.168.168` | 厂默；非 Wi‑Fi `234.1` |
| AGX | `192.168.168.10`（示例） | 同网段静态 IP → `params.local_ip` |

狗端必改：`/opt/export/config/sdk_config.yaml` 的 `target_ip` = AGX IP；启动脚本 `SDK_CLIENT_IP=192.168.168.168`。改完重启狗。
