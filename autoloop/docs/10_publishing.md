# 10 发布与回灌

回灌目标：云端产出的 policy 在门禁通过后，于任务间隙原子出现在机上 `current`，且下一 episode 四元组能证明「跑的就是这一版」。

## PublishGate

同时满足才允许 `model_publisher` 更新 registry 的 `current`：

| # | 闸 | 数据源 |
|---|-----|--------|
| 1 | 构建 `git_sha` 与制品一致 | registry 元数据 |
| 2 | `regression_set` 全绿（固定 seed） | `regression_runner` |
| 3 | 安全链近期无未解释断裂 | `verify-chain` / 云端 Verify |
| 4 | 训练 meta 含完整四元组模板（新 `policy_version`） | `lerobot_builder` / training |
| 5 | failure_pool 阻断单已关闭 | `human_review` |
| 6 | 边侧 recorder gate（回放 valid、覆盖、回归 δ） | `gate/latest.json` |

未过门禁进 `candidates/`，机上默认不拉。`policy_version` 建议 `<name>-<git_sha前8>-<date>`，禁止 `latest`。

## 发布流水

```text
training artifact
    → model_registry.Put(version, sha256, tuple_template)
    → PublishGate
    → model_publisher 标记 current_candidate
    → 机上 model_sync
```

## model_sync（已决）

1. 拉 `current_candidate` 清单，核对 sha256。
2. 写入 `cache/<version>`（`CopyFile` 或等价）。
3. `LocalRegistry`：写 `next` → fsync 目录 → `rename(next, current)`。
4. 失败保持旧 `current`，telemetry `last_error`。
5. 任务状态为 RUNNING 时拒绝 `SwitchTo`，IDLE 再切。
6. 回滚：`current` 指回上一 `policy_version`，同样 rename，并打 `event_code=model_rollback`。

当前 `LocalRegistry::SwitchTo` 若直接写 `current`，以本契约为验收，见 [15 TODO](15_todo.md) T-SYNC-01。

## 混版

新 episode sidecar 必须：

- `version.policy_version == LocalRegistry.Current()`
- `version.git_sha` 为运行中二进制
- `version.env_version` 为当前地图 / 标定

字段非空但与 `current` 不等：`admitted` 仍可为真（四元组齐），`telemetry_gate` 打 **混版**，该 episode 不得作为「新模型」成功样本。
