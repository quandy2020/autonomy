# 06 · 配置参考（GrpcOptions）

Proto：[`../proto/grpc_options.proto`](../proto/grpc_options.proto)  
样例：[`../conf/bridge.pb.txt`](../conf/bridge.pb.txt)

## 1. 基础

| 字段 | 默认 | 说明 |
|------|------|------|
| `host` | `127.0.0.1` | 绑定地址 |
| `port` | `5005` | 监听端口 |
| `num_grpc_threads` | 实现默认 4 | CQ 线程 |
| `num_event_threads` | 实现默认 4 | 事件线程 |
| `num_worker_threads` | `0` → `max(2, event/2)` | Action / Session 池 |
| `uplink_server_address` | 空 | 预留下行/上行（日志） |
| `upload_batch_size` | 0 | 预留 |

## 2. 平台开关

| 字段 | 默认 | 说明 |
|------|------|------|
| `enable_health_check` | true | Default health service |
| `enable_server_reflection` | false | Proto reflection |
| `enable_metadata_interceptor` | true | 元数据拦截器（是否强制键见下） |
| `require_robot_id_metadata` | false | 强制 `x-robot-id` |
| `enable_rate_limit` | false | 全局限流 |
| `rate_limit_qps` | 0 | ≤0 表示不限 |
| `rate_limit_burst` | 0 | 0 → 等于 qps |

## 3. 传输

| 字段 | 默认 | 说明 |
|------|------|------|
| `max_receive_message_bytes` | 0 → 100MB | 接收上限 |
| `max_send_message_bytes` | 0 → 100MB | 发送上限 |
| `keepalive_time_ms` | 0 → 不设 | gRPC keepalive |
| `keepalive_timeout_ms` | 0 | |
| `permit_keepalive_without_calls` | false | |

## 4. 安全

| 字段 | 默认 | 说明 |
|------|------|------|
| `enable_ssl_encryption` | false | 启用 TLS |
| `tls_cert_path` / `tls_key_path` / `tls_ca_path` | 空 | PEM 路径 |
| `tls_require_client_cert` | false | mTLS |
| `auth_mode` | NONE | NONE / TLS / BEARER_TOKEN |
| `bearer_token` | 空 | |
| `bearer_token_file` | 空 | 优先于 token 字段 |
| `enable_google_auth` | false | 第一期未实现 |

## 5. OTel

| 字段 | 默认 | 说明 |
|------|------|------|
| `enable_opentelemetry` | false | |
| `otel_service_name` | `autonomy.bridge` | |
| `otel_exporter_endpoint` | 空 | |

## 6. 推荐模板

**开发（loopback）：**

```text
grpc {
  host: "127.0.0.1"
  port: 5005
  enable_health_check: true
  enable_server_reflection: true
  auth_mode: AUTH_MODE_NONE
}
```

**机载生产：**

```text
grpc {
  host: "127.0.0.1"
  port: 5005
  enable_health_check: true
  enable_server_reflection: false
  enable_rate_limit: true
  rate_limit_qps: 50
  auth_mode: AUTH_MODE_BEARER_TOKEN
  bearer_token_file: "/etc/autonomy/bridge_token"
}
```
