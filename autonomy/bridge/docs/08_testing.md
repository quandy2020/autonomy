# 08 · 测试

## 1. 目录

```text
autonomy/bridge/test/
  README.md
  unit/policy/          # TokenAuthenticator · MetadataValidator · TokenBucket · Deadline
  unit/tools/           # ChannelArgs · Credentials · Interceptor chain
  unit/grpc/            # 自 grpc/ 迁入的核心单测 + 补充
  functional/           # ApplyPlatform · Health · Metadata/Auth/限流强制
```

命名：一律 `*_test.cpp`。CMake 通过 `autonomy_configure_tests` 对 `autonomy/bridge/**/*_test.cpp` 递归 GLOB，无需改 CMakeLists（除非排除规则）。

## 2. 路径迁移

| 旧路径 | 新路径 |
|--------|--------|
| `grpc/task_muxer_test.cpp` | `test/unit/grpc/task_muxer_test.cpp` |
| `grpc/work_scheduler_test.cpp` | `test/unit/grpc/work_scheduler_test.cpp` |
| `grpc/cancel_registry_test.cpp` | `test/unit/grpc/cancel_registry_test.cpp` |
| `grpc/clients/command_dispatch_test.cpp` | `test/unit/grpc/command_dispatch_test.cpp` |

## 3. 用例矩阵

### Unit · policy

- [x] Bearer 空 / 错 / 对；NONE 放行 — `unit/policy/token_authenticator_test.cpp`
- [x] Metadata 缺 `x-robot-id`（require=true）拒绝；多余键忽略 — `metadata_validator_test.cpp`
- [x] TokenBucket QPS 边界与突发 — `token_bucket_test.cpp`
- [x] Deadline 过期判定 — `deadline_policy_test.cpp`

### Unit · tools

- [x] ChannelArgs 从 options 映射 — `channel_args_builder_test.cpp`
- [x] TLS 路径缺失 → insecure — `credentials_factory_test.cpp`
- [x] 拦截器链顺序与短拒 — `server_interceptor_chain_test.cpp` + `*_interceptor_test.cpp`

### Unit · grpc

- [x] TaskMuxer / WorkScheduler / CancelRegistry / DispatchCommands（迁入）
- [x] CommandIdempotencyCache — `idempotency_test.cpp`
- [x] OkStatus / ErrorStatus — `rpc_status_test.cpp`
- [x] BackgroundCommandSession 门控 — `session_gate_test.cpp`

### Functional

- [x] ApplyPlatform 开关副作用（health / reflection / keepalive） — `platform_bootstrap_test.cpp`
- [x] Metadata / Auth / RateLimit 强制语义 — `*_enforcement_test.cpp`
- [x] Health SERVING / NOT_SERVING — `health_service_test.cpp`

## 4. 如何跑（说明；本阶段不执行 build / ctest）

```bash
# 配置开启测试后（示例）
cmake -B build -DBUILD_TESTING=ON -DBUILD_GRPC=ON ...
cmake --build build -j
ctest --test-dir build -R "bridge" --output-on-failure

# 或匹配单文件生成的测试目标名（以生成规则为准）
ctest --test-dir build -R "token_authenticator_test" --output-on-failure
```

无 `grpc++_reflection` 时，reflection 相关断言启用；否则 `platform_bootstrap_test` 对 reflection 用例 `GTEST_SKIP`。

## 5. 手工平台验证

前置：启动 `autonomy.bridge`（或含 Bridge 的 launch），确认 `conf/bridge.pb.txt` 中：

- `enable_health_check: true`
- 开发机可设 `enable_server_reflection: true`（机载生产保持 false）

### 内置客户端（推荐）

```bash
autonomy.bridge list
autonomy.bridge describe SystemService/Heartbeat
autonomy.bridge call SystemService/Heartbeat -d '{"sequence":1}' \
  --target 127.0.0.1:5005
autonomy.bridge call SystemService/GetInfo -d '{}'
autonomy.bridge call SystemService/Heartbeat -d '{}' \
  --bearer <token> --robot-id robot-1
```

支持 unary 与 server-streaming；`-d @file.json` 从文件读请求体。

### Health

```bash
# 安装 grpc_health_probe（Go）或使用发行包
grpc_health_probe -addr=127.0.0.1:5005
# 期望：status: SERVING
```

### Reflection + grpcurl

```bash
# 仅 enable_server_reflection=true 且链接了 grpc++_reflection 时有效
grpcurl -plaintext 127.0.0.1:5005 list
grpcurl -plaintext 127.0.0.1:5005 list automsgs.rpcs.system.SystemService
grpcurl -plaintext 127.0.0.1:5005 describe automsgs.rpcs.system.SystemService

# 带 Bearer（auth_mode=AUTH_MODE_BEARER_TOKEN 时）
grpcurl -plaintext \
  -H 'authorization: Bearer <token>' \
  -H 'x-robot-id: robot-1' \
  127.0.0.1:5005 automsgs.rpcs.system.SystemService/Heartbeat
```

### Metadata 强制

当 `require_robot_id_metadata: true`：省略 `x-robot-id` 应收到 gRPC `FAILED_PRECONDITION`（detail=`missing x-robot-id`）。

## 6. FEATURE 降级

| FEATURE | 缺库行为 |
|---------|----------|
| `grpc_reflection` | 不定义 `AUTONOMY_HAVE_GRPC_REFLECTION`；`ApplyPlatform` 跳过 reflection 并打 WARN |
| `otel` | 无 OpenTelemetry SDK 时使用 `NoopTracerProvider`；不红 CI |

相关测试应 `GTEST_SKIP` 或条件编译，不导致默认 CI 红。

## 7. 本阶段验证范围

**不执行** `cmake` / `build` / `ctest`。完备性检查：

1. `test/**/*_test.cpp` 与上表矩阵一一对应  
2. `docs/01–08` + `policy/` + `tools/` + `ApplyPlatform` 接线存在  
3. 手工步骤见 §5（供后续联调）
