# 04 · 安全与 TLS

## 1. 威胁模型（简表）

| 部署 | 风险 | 推荐 |
|------|------|------|
| 仅 `127.0.0.1` | 本机进程滥用 | `AUTH_MODE_NONE` + Health on；Reflection off |
| LAN / 云边 | 窃听、伪造客户端 | TLS（`enable_ssl_encryption`）+ 可选 mTLS + Bearer |
| 公网 | 扫描、枚举 | TLS + Bearer/mTLS；**禁止 Reflection**；限流 on |

## 2. TLS 文件

| 选项 | 含义 |
|------|------|
| `tls_cert_path` | 服务端证书 PEM |
| `tls_key_path` | 服务端私钥 PEM |
| `tls_ca_path` | 校验客户端证书的 CA（mTLS） |
| `tls_require_client_cert` | true 时要求客户端证书 |

`enable_ssl_encryption=true` 但路径缺失时：**回退 insecure 并打 ERROR 日志**（避免静默假安全）。详见 `tools/transport/credentials_factory`。

## 3. Bearer 流程

1. 客户端设置 metadata：`authorization: Bearer <token>`
2. Auth 拦截器比对 `bearer_token` / `bearer_token_file`
3. 失败 → `UNAUTHENTICATED`

可与 TLS 叠加（TLS 保机密性，Bearer 保身份）。

## 4. Google Auth

历史字段 `enable_google_auth` 保留；第一期 **不实现** Google ID token 校验（文档占位）。若启用仅打警告。

## 5. 运维检查清单

- [ ] 生产 Reflection = false
- [ ] 非 loopback 时 enable_ssl_encryption = true 且证书有效
- [ ] Bearer token 不进 git；用文件或密钥注入
- [ ] 限流按机载 CPU 调 QPS
