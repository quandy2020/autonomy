# 02 · Policy（决策层）

Policy 回答「这次 RPC 允不允许、按什么规则限流 / 校验」。实现位于 `autonomy/bridge/policy/`。

## 1. 鉴权（`policy/auth`）

### 模式（`GrpcOptions.auth_mode`）

| 值 | 含义 |
|----|------|
| `AUTH_MODE_NONE` | 不校验（默认；适合 loopback 开发） |
| `AUTH_MODE_TLS` | 依赖传输层 mTLS；拦截器侧仅检查 peer 是否已认证（若运行时可得） |
| `AUTH_MODE_BEARER_TOKEN` | 要求 metadata `authorization: Bearer <token>` 与配置 token 一致 |

### Bearer 规则

- 头名：`authorization`（小写；gRPC 元数据键大小写不敏感，校验前规范化）。
- 值前缀：`Bearer `（注意空格）；其余为 opaque token。
- 配置：`bearer_token` 明文，或 `bearer_token_file` 读文件（文件优先）。
- 失败：gRPC `UNAUTHENTICATED`，detail=`missing or invalid bearer token`。

## 2. 元数据（`policy/metadata`）

### 标准键（`metadata_keys.hpp`）

| 键 | 用途 |
|----|------|
| `x-robot-id` | 可选/强制的机器人身份（由 `require_robot_id_metadata` 控制） |
| `authorization` | Bearer |
| `x-request-id` | 可选追踪关联（Logging / OTel 读取，不强制） |

### 校验

- `MetadataValidator`：按选项检查必填键非空。
- 缺 `x-robot-id`（当 require=true）：`FAILED_PRECONDITION`，detail=`missing x-robot-id`。
- 未知键：忽略（不拒绝）。

## 3. 限流（`policy/rate_limit`）

- 算法：令牌桶（`TokenBucket`），容量 ≈ `rate_limit_burst`（默认 = QPS），补充速率 = `rate_limit_qps`。
- 维度：第一期进程级全局限流（所有方法共享）；后续可按 method 扩展。
- 失败：`RESOURCE_EXHAUSTED`，detail=`rate limit exceeded`。
- 关闭：`enable_rate_limit=false`（默认）。

## 4. 可靠性（`policy/reliability`）

### Deadline

- 尊重客户端 gRPC deadline；`DeadlinePolicy::IsExpired(now, deadline)` 供拦截器 / 工具使用。
- 服务端不主动延长 deadline。

### Retry hints

- `retry_hints.hpp` 仅文档化客户端建议（幂等读可重试；START 类命令依赖 `goal_id` 幂等，勿盲目重试）。
- 服务端不实现自动重试。

## 5. 拒绝码表

| 场景 | gRPC StatusCode | detail（稳定字符串） |
|------|-----------------|----------------------|
| 无/错 Bearer | `UNAUTHENTICATED` | `missing or invalid bearer token` |
| 缺强制 robot-id | `FAILED_PRECONDITION` | `missing x-robot-id` |
| 限流 | `RESOURCE_EXHAUSTED` | `rate limit exceeded` |
| Deadline 已过 | `DEADLINE_EXCEEDED` | `deadline exceeded` |

业务层 `OkStatus` / `ErrorStatus`（`automsgs` Status）与上述**正交**：拦截器拒绝时 Handler 不会执行。
