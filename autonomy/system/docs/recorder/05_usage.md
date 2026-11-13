# 05 使用

构建、配置、启动、CLI 与产物路径。命令均在仓库根、`PATH` 含 `build/bin` 的前提下给出。

## 构建

```bash
cmake -B build -DAUTONOMY_BUILD_RECORDER=ON
cmake --build build --target autonomy.recorder
```

测试目标：`system.recorder_test`（CMake）或 `//autonomy/system:recorder_test`（Bazel）。关闭 recorder：`-DAUTONOMY_BUILD_RECORDER=OFF`。

## 环境变量

| 变量 | 作用 |
|------|------|
| `AUTONOMY_POLICY_VERSION` | 四元组 policy（优先） |
| `AUTONOMY_SOFTWARE_VERSION` | policy 回退 |
| `AUTONOMY_ENV_VERSION` | 四元组 env |
| `AUTONOMY_SEED` | 四元组 seed |
| `AUTONOMY_GIT_SHA` | 覆盖 git SHA |
| `AUTONOMY_RECORD_ROOT` | `output_dir` 为空时的根目录 |
| `AUTONOMY_LOG_ROOT` | P0 危险指针写入 `$AUTONOMY_LOG_ROOT/events/` |
| `AUTONOMY_SAFETY_LATCH_PATH` | 急停闩文件（与 monitor / Bridge 共用） |
| `HOME` | `output_dir` 默认 `$HOME/.autonomy/records` |

未设置 `AUTONOMY_RECORD_ROOT` 且无 `HOME` 时回退 `/tmp/autonomy/records`。

## 配置

文件：`autonomy/system/conf/recorder.pb.txt`，schema 为 `RecorderOptions`。`--conf` 未指定时进程读工作目录下 `recorder.pb.txt`（launch 依赖 `AUTOLINK` 配置搜索路径）。

| 字段 | 默认 / 约定 |
|------|-------------|
| `output_dir` | 空 → 环境变量 / `~/.autonomy/records` |
| `buffer_sec` | 30 |
| `full_dump` | false；true 或 `--full-dump`：会话级 MCAP，P2/P3 全留 |
| `flush_period_sec` | 1.0（仅 full_dump） |
| `localization_quality_floor` | 0.3 |
| `channels[]` | 对齐 `control/constants.hpp` 与 `task/common/names.hpp`；`timeout_sec=0` 不监视超时；`depth`/`queue` 为 0 用启发式 |
| `bt_log_channel` | `/behavior_tree/log` |
| `cmd_vel_channel` | `/cmd_vel` |
| `odom_channel` / `plan_channel` | `/odom` `/plan` |
| `teleop_goal_channel` | `/autonomy/task/teleop/goal` |
| `path_deviate_m` | 0.5 |
| `cost_spike_sigma` | 3.0 |
| `p2_sample_rate` / `p3_sample_rate` | 0.5 / 0.1 |
| `action_match_threshold` | 0.95 |
| `attribution_coverage_floor` | 0.8 |
| `regression_delta_limit` | 0.05 |
| `require_version_tuple` | true |
| `max_cmd_vx` / `max_cmd_wz` | 2.0（L4） |
| `time_source` | `TIME_SOURCE_SOFTWARE` |
| `safety_chain_path` | 空 → `$output_dir/safety/chain.bin` |

四元组请用环境变量或 conf 填满，否则 episode 进入 `diagnostics/`。

## Launch

```bash
export PATH=$PWD/build/bin:$PATH
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/system/launch

# 默认栈含 recorder 模块
autolink_launch autonomy.launch

# 仅旁路采集（需栈已在跑或另启）
autolink_launch recorder.launch

# 测试全量（会话 episode + 周期 flush）
autolink_launch recorder_full.launch
# 或
autonomy.recorder --conf=recorder.pb.txt --full-dump

# 仅 L4 闩锁监控
autolink_launch safety.launch
```

`replay.launch` 仍走既有 `.record` 回灌脚本；证据 MCAP 的 SIL 使用下面的 `replay` 子命令。

## CLI

```bash
# L1 采集（无子命令）
autonomy.recorder --conf=recorder.pb.txt
autonomy.recorder --conf=recorder.pb.txt --full-dump

# L4
autonomy.recorder safety --conf=recorder.pb.txt
autonomy.recorder verify-chain -i ~/.autonomy/records/safety/chain.bin

# L6 SIL / HIL（--hil > --sensors-only > --profile）
autonomy.recorder replay -i ep.mcap -r 1 --profile open_loop
autonomy.recorder replay -i ep.mcap --profile mid_stack
autonomy.recorder replay -i ep.mcap --sensors-only
autonomy.recorder replay -i ep.mcap --hil

# L5 发布门禁
autonomy.recorder gate -d ~/.autonomy/records

# 离线证据
autonomy.recorder pipeline -i ep.sidecar.pb --match 0.95 --coverage 0.8
autonomy.recorder verdict -a rec.mcap -b replay.mcap -o overlay.json
autonomy.recorder verdict -a rec.mcap -b replay.mcap --interventional

# 观察（须 -n evidence_node_id）
autonomy.recorder observe -o ep.observations.jsonl --id o1 -d "避障过晚" -n n12 --bt FollowPath --from 0 --to 0

# 边侧
autonomy.recorder convert -i ep.mcap -o /tmp/lerobot --fps 10
autonomy.recorder upload -s ~/.autonomy/records/episodes -d /data/inbox
```

退出码：`verdict` / `pipeline` / `gate` 在 `valid=false` 或 `publish_allowed=false` 时为失败；`observe` 缺节点时失败；HIL 遇闩锁失败。

`convert` / `pipeline` / `verdict` / `observe` / `upload` / `verify-chain` / `gate` 不调用 `autolink::Init`。

## autoviz

1. 启动 autoviz，Add Panel → **Evidence**（或拖入 `*.sidecar.json`）。
2. 打开 `<id>.sidecar.json`；头栏显示 `bt=` 失败节点与 `gate.passed`；Compare 页含门禁字段、overlay A/B、根因卡片。
3. 若同时用 Record 打开同 episode 的 `.mcap`，Evidence cursor 会 `seekTo` 回放时间轴。
4. Observe：填写 description 与节点后写入 `<id>.observations.jsonl`。

Evidence 不依赖 `autonomy_system`，只解析 JSON。

## 产物与准入

```text
~/.autonomy/records/
  episodes/<id>.mcap
  episodes/<id>.sidecar.pb
  episodes/<id>.sidecar.json
  episodes/<id>.manifest.json
  episodes/<id>.overlay.json
  diagnostics/                 # admitted=false
  metrics/episodes.jsonl
  gate/latest.json
  safety/chain.bin
```

`manifest.json` 字段：`episode_id`、`admitted`、`priority`、`upload`、`expert_demo`、`sampled`、`policy`、`bt_root`、`robot_id`。

训练与正式回放只使用 `episodes/` 且 `admitted=true`。`diagnostics/` 供排障，不作为数据集。

## 最小检查

1. 导出四元组环境变量后启动 recorder，确认日志 `admitted=1`。
2. 触发一次 BT 任务或急停，检查 `episodes/` 或 `diagnostics/` 出现同 stem 六件套。
3. `verify-chain` 对 `safety/chain.bin` 返回 ok。
4. `pipeline -i *.sidecar.pb` 打印 `gate=`。
5. `gate -d ~/.autonomy/records` 读 `gate/latest.json`。
6. autoviz Evidence 能打开 sidecar，头栏有 `gate.passed`，且无节点的观察无法保存。
7. `--full-dump` 跑一段任务后 Shutdown，检查单一 session `.mcap` 覆盖全程而非仅 30s。
