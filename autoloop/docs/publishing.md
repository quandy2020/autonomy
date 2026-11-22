# Publishing（可回灌）

回灌目标：云端产出的 policy 在门禁通过后，于任务间隙原子出现在机上 `current`，且下一 episode 四元组能证明「跑的就是这一版」。

## 门禁 PublishGate

同时满足才允许 `model_publisher` 更新 registry 的 `current`：

1. 构建 `git_sha` 与制品一致。
2. `regression_set` 全绿（固定 seed）。
3. 安全链近期无未解释断裂。
4. 转换 / 训练元数据含完整四元组模板（新 `policy_version`）。
5. 人审：来自 failure_pool 的阻断单已关闭。

未过门禁的制品可进 `registry` 的 `candidates/`，机上默认不拉。

## 发布

```
training artifact
    → model_registry.Put(version, sha256, tuple_template)
    → PublishGate
    → model_publisher 标记 current_candidate
```

`policy_version` 建议：`<name>-<git_sha前8>-<date>`，禁止只写 `latest`。

## 机上 model_sync

1. 拉 `current_candidate` 清单，核对 sha256。
2. 写入 `cache/<version>`（`FetchModel` = autolink `CopyFile` 或等价）。
3. `LocalRegistry`：写 `next` → fsync → rename 为 `current`。
4. 失败保持旧 `current`，telemetry `last_error`。
5. **RUNNING 任务中禁止切换**；下一 IDLE 再切。

回滚：把 `current` 指回上一 `policy_version`，同样走原子 rename。回滚后必须立刻打一条结构化事件 `event_code=snapshot` 以外的 `model_rollback`，便于接管率对比。

## 闭环闭合检查

新 episode sidecar 必须：

- `version.policy_version == LocalRegistry.Current()`
- `version.git_sha` 为运行中二进制
- `version.env_version` 为当前地图 / 标定

任一不等：`admitted` 仍可因字段非空而为真，但 `telemetry_gate` 打 **混版** 标记，该 episode 不得作为「新模型」的成功样本。
