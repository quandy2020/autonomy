# 09 可视化

可视化是证据的只读投影：所有面板以 `episode_id` 为游标，seek 到同一单调轴。禁止可视化进程发布 `/cmd_vel`。

## Episode 分诊

对齐 autoviz Evidence：读 `sidecar.json` + MCAP。

| 面板 | 数据 | 联动 |
|------|------|------|
| 时间轴 | DualClock 段、触发点、闩锁 | cursor → 全面板 |
| 通道 overlay | odom / plan / cmd | 绑定 `evidence_node_id` |
| 因果 / 事件 | triggers + BT 转移 | 点击节点 → MCAP seek |
| 四元组条 | admitted / missing_fields | 未准入红色，仍可播 |
| 安全链 | hash 校验勾 | 失败则整页告警 |
| Compare | overlay A/B、根因卡片、`gate.passed` | 与 Record `seekTo` |

Observe 写 `.observations.jsonl`：无 description 或无 `evidence_node_ids` 则拒绝。拖入 `*.sidecar.json` 打开 EvidenceDock。

`--full-dump` 长 MCAP：时间轴抽样渲染（默认 10 Hz overlay），原始字节仍按帧 seek，禁止一次性载入全量点云。

## 机队仪表

`cloud/observability/dashboards/`（Grafana 导入 JSON，非实时控制台）：

| 文件 | 看什么 | 下钻 |
|------|--------|------|
| `fleet_health.json` | data_agent 心跳、ring 深度、上传积压 | robot_id → 最近 episode |
| `takeover_rate.json` | 接管 / 任务失败率（按 policy_version） | 失败 episode 列表 |
| `conversion_quality.json` | 转换成功率、缺字段率 | diagnostics 队列 |

告警 YAML：`error_density` / `data_loss` / `safety_event`。payload 必须含 `trace_id`。健康字段对齐 `DataAgentHealth` proto。

## 治理卡片

success / failure pool：缩略时间轴 + 四元组 + 人审状态。点卡片打开分诊 UI，不另开无身份视频桶。视频若经 `video_encoder` 转出，meta 仍指向原 `episode_id`。

清单与 UI 用 JSON；机器链路用 `sidecar.pb`。
