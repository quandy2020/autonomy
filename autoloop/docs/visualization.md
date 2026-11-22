# Visualization（可可视化）

可视化是证据的只读投影：所有面板以 `episode_id` 为游标，seek 到同一 `mono_ns`。禁止可视化进程发布 `/cmd_vel`。

## 三层界面

### 1. Episode 分诊（工程师）

对齐 autoviz Evidence：读 `sidecar.json` + `snapshot.mcap`。

| 面板 | 数据 | 联动 |
|------|------|------|
| 时间轴 | DualClock 段、触发点、闩锁 | cursor → 全面板 |
| 通道 overlay | odom / plan / cmd | 绑定 `evidence_node_id` |
| 因果 / 事件 | triggers + BT 转移 | 点击节点 → MCAP seek |
| 四元组条 | admitted / missing_fields | 未准入红色，仍可播 |
| 安全链 | hash 校验勾 | 失败则整页告警 |

观察写入 `.observations.jsonl` 时必须带 `evidence_node_id`。

### 2. 机队仪表（值班）

`cloud/observability/dashboards/`：

| 仪表 | 看什么 | 下钻 |
|------|--------|------|
| `fleet_health.json` | data_agent 心跳、ring 深度、上传积压 | robot_id → 最近 episode |
| `takeover_rate.json` | 接管 / 任务失败率（按 policy_version） | 失败 episode 列表 |
| `conversion_quality.json` | 转换成功率、缺字段率 | diagnostics 队列 |

告警：`error_density` / `data_loss` / `safety_event`。告警 payload 含 `trace_id`。

### 3. 治理 / 训练（数据同学）

success / failure pool 卡片：缩略时间轴 + 四元组 + 人审状态。点卡片打开分诊 UI，不另开无身份的视频桶。

## 全量落盘可视化

测试 `--full-dump` 的长 MCAP 只在时间轴上抽样渲染（默认 10 Hz overlay），原始字节仍按帧可 seek。避免 UI 把全量点云一次性载入。

## 数据约定

- 清单与 UI 用 JSON；机器链路用 `sidecar.pb`。
- 视频若经 `video_encoder` 转出，meta 仍指向原 `episode_id`。
- 云端 dashboard JSON 为 Grafana / 同类导入文件，不是实时控制台。
