# 钢镚 L1-W（智身 GENISOM / zsibot）— Autolink Component

独立库 `libautodriver_l1w.so`：覆盖官方 **ZSL-1W HighLevel** 文档全部控制/状态接口。

## 官方 API 映射

| HighLevel | autodriver |
|-----------|------------|
| `standUp` | mode `stand` / tool `stand` / `auto_stand` |
| `lieDown` | tool `lie`；Stop 时可选 `lie_on_stop` |
| `passive` | mode `estop` / tool `passive` |
| `move(vx,vy,wz)` | `/cmd_vel` + mode `wheel` |
| `crawl(vx,vy,wz)` | `/cmd_vel` + mode `walk` |
| `cancelCrawl` | tool `cancel_crawl` |
| `attitudeControl` | tool `attitude=r,p,y,h` |
| 位姿/速度/IMU/电量/ctrlmode | `ReadChassisState` → `/robot_state`（`map_name=ctrl:*`） |

文档未列的 demo 特技（jump / climb / shakeHand）**不在** HighLevel 头文件内，故未封装。

## SDK

```bash
git clone https://github.com/zsibot/genisom_l1_sdk_old.git
cmake ... -DGenisomL1w_ROOT=$PWD/genisom_l1_sdk_old
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
# 站立 / 轮式 / 匍匐
echo stand  > /chassis/mode   # 经 Autolink 发 String
echo wheel  > ...
echo walk   > ...          # crawl 步态

# 工具
echo lie > .../tool
echo 'attitude=0,0,0,0.1' > .../tool
echo cancel_crawl > .../tool
```

网线 AGX↔RK3588：`host=192.168.168.168`，`local_ip=<AGX>`，狗端 `sdk_config.target_ip=<AGX>`。

## 网线联机（AGX ↔ RK3588）

| 端 | IP | 说明 |
|---|---|---|
| RK3588（狗有线） | `192.168.168.168` | 厂默；非 Wi‑Fi `234.1` |
| AGX | `192.168.168.10`（示例） | 同网段静态 IP → `params.local_ip` |

狗端必改：`/opt/export/config/sdk_config.yaml` 的 `target_ip` = AGX IP；启动脚本 `SDK_CLIENT_IP=192.168.168.168`。改完重启狗。

## 未映射到 RobotState 的 SDK 读数

`getBodyAcc`、腿关节角/速/力矩：当前 `vehicle_msgs.RobotState` 无对应字段，故未写入通道。控制面与位姿/速度/电量/ctrlmode 已全覆盖。

官方 demo 特技（jump / climb / shakeHand）不在 HighLevel 头文件内，未封装。
