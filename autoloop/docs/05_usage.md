# 05 使用

构建、配置、启动、CLI 与产物路径。命令均在 `autoloop/` 仓库根、或 autonomy 仓库根且 `PATH` 含 `build/bin` 的前提下给出。

## 构建

```bash
# 在 autoloop/ 内
cmake --preset default
cmake --build build
cmake --build build --target autoloop.recorder
cmake --build build --target autoloop-convert
```

测试：`BUILD_TESTING=ON`（preset 默认）。关闭 recorder：`-DAUTOLOOP_BUILD_RECORDER=OFF`。

从 autonomy 工作区联编时，automsgs / autolink 走 `cmake/WorkspaceDeps.cmake`。

## 环境变量

| 变量 | 作用 |
|------|------|
| `AUTOLOOP_POLICY_VERSION` / `AUTONOMY_POLICY_VERSION` | 四元组 policy |
| `AUTONOMY_SOFTWARE_VERSION` | policy 回退 |
| `AUTOLOOP_ENV_VERSION` / `AUTONOMY_ENV_VERSION` | 四元组 env |
| `AUTOLOOP_SEED` / `AUTONOMY_SEED` | 四元组 seed |
| `AUTOLOOP_GIT_SHA` / `AUTONOMY_GIT_SHA` | 覆盖 git SHA |
| `AUTOLOOP_RECORD_ROOT` | recorder `output_dir` 为空时的根 |
| `AUTOLOOP_CONF_PATH` / `AUTONOMY_CONF_PATH` | `--conf` 相对路径搜索 |
| `AUTOLOOP_SAFETY_LATCH_PATH` / `AUTONOMY_SAFETY_LATCH_PATH` | 急停闩文件 |
| `AUTOLOOP_LOG_ROOT` / `AUTONOMY_LOG_ROOT` | P0 危险指针 / crash 日志根 |
| `HIL_ARMED` | 显式允许 HIL 往隔离台架灌回放 |
| `HOME` | 默认记录根回退 |

未设置记录根且无 `HOME` 时 recorder 回退 `/tmp/autoloop/records`。闩锁默认 `~/.autonomy/runtime/safety_latch`。

## 配置

| 文件 | 说明 |
|------|------|
| `autoloop/robot/config/data_agent/recorder.pb.txt` | `RecorderOptions`；`--conf` 未指定时读工作目录 `recorder.pb.txt` |
| `autoloop/robot/config/data_agent/conservative.yaml` | `window_sec=2` `max_samples=2048` `priority_floor=1` |
| `autoloop/robot/config/data_agent/aggressive.yaml` | 4s / 8192 / floor 3 |
| `autoloop/robot/config/data_agent/minimal.yaml` | 1s / 512 / floor 0 |
| `autoloop/robot/config/model_sync/sync_policy.yaml` | 回灌 |
| `autoloop/cloud/conversion/presets/*.yaml` | LeRobot preset |

recorder 字段默认见 [recorder 05](recorder/05_usage.md)。四元组请用环境变量或 conf 填满，否则 episode 进入 `diagnostics/`。

## Launch

```bash
export PATH=$PWD/build/bin:$PATH

# Autoloop bringup（相对 autoloop/autoloop/robot/bringup/launch）
autolink_launch system.launch          # drivers + loc + perc + nav + data_agent + model_sync + monitor
autolink_launch recorder.launch        # 仅旁路采集
autolink_launch recorder_full.launch   # 测试全量
autolink_launch recorder_safety.launch # 仅 L4

# 与 autonomy 栈同机（相对 autonomy/system/launch）
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/system/launch
autolink_launch autonomy.launch        # 含 autoloop.recorder 模块
autolink_launch recorder.launch
autolink_launch recorder_full.launch
autolink_launch safety.launch
```

脚本：`robot/scripts/run_data_agent.sh`、`trigger_snapshot.sh`、`sync_model.sh`、`log_rotate.sh`、`self_check.sh`。

## CLI

```bash
# L1 采集 / L4 / L6 / 边侧（详见 recorder 05）
autoloop.recorder --conf=recorder.pb.txt
autoloop.recorder --conf=recorder.pb.txt --full-dump
autoloop.recorder safety --conf=recorder.pb.txt
autoloop.recorder replay -i ep.mcap --profile mid_stack
autoloop.recorder verdict -a rec.mcap -b replay.mcap -o overlay.json
autoloop.recorder pipeline -i ep.sidecar.pb
autoloop.recorder gate -d ~/.autonomy/records
autoloop.recorder convert -i ep.mcap -o /tmp/lerobot --fps 10
autoloop.recorder upload -s ~/.autonomy/records/episodes -d /data/inbox
autoloop.recorder verify-chain -i ~/.autonomy/records/safety/chain.bin
autoloop.recorder observe -o ep.observations.jsonl --id o1 -d "避障过晚" -n n12

# 云端转换
autoloop-convert convert <episode_dir> <out_dir>
autoloop-convert check <episode_dir>
```

退出码：`verdict` / `pipeline` / `gate` 在 `valid=false` 或 `publish_allowed=false` 时失败；`observe` 缺节点失败；HIL 遇闩锁失败；`convert check` 哈希或四元组失败。

## autoviz

1. Add Panel → **Evidence**（或拖入 `*.sidecar.json`）。
2. 打开 sidecar；头栏 `bt=` 与 `gate.passed`。
3. 同 episode 的 `.mcap` 在 Record 打开时，Evidence cursor `seekTo`。
4. Observe：description + `evidence_node_id` 后写 jsonl。

Evidence 只解析 JSON，不链 `autonomy_system`，不发布 `/cmd_vel`。

机队仪表 JSON：`cloud/observability/dashboards/{fleet_health,takeover_rate,conversion_quality}.json`。告警：`alerts/{error_density,data_loss,safety_event}.yaml`。

## 产物与准入

```text
~/.autonomy/records/                 # recorder
  episodes/<id>.mcap
  episodes/<id>.sidecar.pb
  episodes/<id>.sidecar.json
  episodes/<id>.manifest.json
  diagnostics/
  metrics/episodes.jsonl
  gate/latest.json
  safety/chain.bin

$AUTOLOOP_RECORD_ROOT/ep-<utc_ns>-<kind>/   # data_agent
  snapshot.mcap
  sidecar.pb
  sidecar.json
```

训练与正式回放只使用 `admitted=true`。`diagnostics/` 供排障，不作为数据集。LeRobot 行必须能反查 `robot_id / episode_id / content_sha256`。

## 最小检查

1. 导出四元组环境变量后启动 `autoloop.recorder`，日志 `admitted=1`。
2. 触发一次 BT 任务或急停，检查 `episodes/` 或 `diagnostics/` 出现同 stem 文件。
3. `verify-chain` 对 `safety/chain.bin` 返回 ok。
4. `autoloop-convert check <episode_dir>` 通过；缺字段则应失败。
5. `gate -d ~/.autonomy/records` 读 `gate/latest.json`。
6. autoviz Evidence 能打开 sidecar；无节点的观察无法保存。
7. `--full-dump` 跑一段任务后 Shutdown，单一 session `.mcap` 覆盖全程。
8. 人为清空 `AUTOLOOP_POLICY_VERSION`，确认新 episode 进 diagnostics 且 `record_receiver` 拒收。
9. `model_sync` 切版本后，下一 episode `policy_version` 等于 `LocalRegistry.Current()`。

未落地项与验收见 [15 TODO](15_todo.md)。机上 CLI 细则见 [20 / recorder 05](recorder/05_usage.md)。
