# autonomy/system — 单机移动机器人管理面

机上控制面：状态真相 + 运维策略。对外 RPC 在 `bridge`；装机推包在 `ansible`。

## 三约定

1. **健康：** 仅 `autonomy.monitor` 采集 → `HealthSnapshotStore`；Bridge `GetHealth` **只读**快照，禁止第二套 `MonitorRegistry`。
2. **急停：** Bridge E-Stop 与 MRM 共用 `SafetyLatch` 文件闩锁；Snapshot 反映闩锁。
3. **升级：** Ansible = 装机/救援；运行中升级 = `system/ota` 状态机（可由 Bridge 触发）。

## 八域

| 域 | 落点 |
|----|------|
| 身份/配置 | `conf/`、`conf/profiles/`、`options` |
| 进程 | `launch/` + `critical_processes`（monitor conf） |
| 健康 | `monitor/` |
| 安全 | `safety/safety_latch` + `monitor/mrm_*` |
| 日志 | `conf/logging.pb.txt` + `logging/` 事件包 |
| 数据 | `conf/data.pb.txt`、`launch/replay.launch`、DataPackage |
| OTA | `ota/` + `tools/package_ota_delta.py` |
| 对外 API | `bridge` + `automsgs` RPC（非本树） |

## 默认 launch 模块集（`autonomy.launch`）

| 模块 | 默认 | respawn | 说明 |
|------|------|---------|------|
| monitor | 是 | 是 | 健康真相源 |
| planning | 是 | 是 | |
| control | 是 | 是 | |
| task | 是 | 是 | |
| perception | 是 | 是 | conf 常 `enabled=false` |
| bridge | 是 | 是 | 需 `-DBUILD_GRPC=ON` |
| foxglove | 是 | 否 | 可视化 |
| localization / autodriver | **否** | — | 另 launch；全栈回灌/仿真见 `replay.launch` / `sim_fullstack.launch` |

## 环境变量

| 变量 | 含义 |
|------|------|
| `AUTONOMY_HEALTH_SNAPSHOT_PATH` | 健康快照文件（默认 `$XDG_RUNTIME_DIR/autonomy/health_snapshot.pb`） |
| `AUTONOMY_SAFETY_LATCH_PATH` | 急停闩锁文件 |
| `PACKAGE_DIR` / `RECORD_FILE` | 回灌数据包或 `.record` |
| `REPLAY_PROFILE` | `sensor_in` \| `mid_stack` \| `open_loop` |
| `GLOG_log_dir` | 日志目录（建议 `$HOME/.autonomy/log/runs/<run_id>`） |
| `XDG_RUNTIME_DIR` | Bridge `RestartModule` 请求文件目录（`$XDG_RUNTIME_DIR/autonomy/`） |

## OTA 包

- **full：** `tools/package_install_tarball.py` → `full.tar.gz` / `autonomy.tar.gz`
- **delta：** `tools/package_ota_delta.py --base … --target … --base-version … --target-version … --output … [--full-tar …]`
  - 产出 `package_manifest.json` + `delta/payloads` + `delta/deletes.txt`（可选附带 `full.tar.gz` 供 fallback）

## 裁决三问（是否进 system）

1. 这一台机上运维？ 2. 不靠业务语义？ 3. 不该归 Bridge/Ansible/Autolink/机外？ — 否 → 不进 system。
