# Autoviz gRPC Client 面板设计

- **日期**：2026-08-28
- **状态**：已批准（待实现计划）
- **参考**：[Postman gRPC request interface](https://learning.postman.com/docs/sending-requests/grpc/grpc-request-interface/)
- **范围**：新建独立 gRPC 客户端面板（`GrpcDock`），支持 unary + client/server/bidi streaming；通用服务器 + 内置 `automsgs` RPC；Server Reflection 与 `.proto` / 预置定义
- **不做（首版）**：复杂 Authorization（OAuth 等）、Scripts、集合协作/分享、gRPCurl 导出、多请求 Tab 编辑器、取代 Autolink Service Call、嵌入 gRPC server

## 1. 背景与目标

autoviz 现有 **Service Call** 面板走 Autolink unary `Client<RawMessage>`，无法对接仓库 `automsgs/proto/rpcs/*`（如 `TeleopService`）及 bridge 上的标准 gRPC。用户需要 Postman / Insomnia 风格的 gRPC 客户端面板，用于调试域内与通用 gRPC 服务。

**成功标准**

1. 可对任意 gRPC URL（含 TLS）选择 method 并 Invoke，看到 status / latency / JSON 响应。
2. 四种 method type 均可操作（unary 单次；streaming 有 Send / End / 时间线）。
3. 可通过 Reflection、导入 `.proto`、或一键加载 `automsgs` RPC 获得方法列表与 schema。
4. 无 gRPC 依赖时项目仍可编译；面板提示不可用。
5. 不破坏现有 Service / Teleop / Publish 面板。

## 2. 决策摘要

| 项 | 选择 |
|----|------|
| 调用类型 | 全量：unary + client / server / bidi streaming |
| 目标服务 | 通用客户端 + 内置 `automsgs/proto/rpcs` 快捷加载 |
| 服务定义 | Reflection **与** `.proto` / 内置包 |
| 与 Service Call | **新建独立面板**，旧面板保留 |
| 实现路径 | autoviz 内嵌 `grpc++` 动态客户端（Descriptor + DynamicMessage + GenericStub） |

## 3. UI 设计

对齐 Postman gRPC 请求区，Dock ID：`GrpcDock`，Catalog 名：**gRPC**。

### 3.1 连接栏

- TLS 开关（Enable TLS）
- Server URL（支持历史下拉；可接受 `host:port` 或 `grpc://` / `https://` 形式，内部规范化）
- Method 下拉：`Service / Method`，图标区分 unary / client-stream / server-stream / bidi
- 主按钮：
  - Unary：**Invoke**
  - Streaming 进行中：**Send** + **End Streaming**（client/bidi）；server-stream 以 **End** / 取消为主

### 3.2 请求 Tab

| Tab | 内容 |
|-----|------|
| **Message** | JSON 编辑器；Beautify；Use Example Message（按 schema 生成） |
| **Metadata** | 键值对表格（发出去的 initial metadata） |
| **Service definition** | Reflection 状态与刷新；导入 `.proto`（及依赖路径）；**Load automsgs RPCs** |
| **Settings** | 校验证书、SNI/server name override、超时 ms、最大响应消息 MB、是否包含默认值字段 |

首版 **不做**：Authorization 复杂类型、Scripts。

### 3.3 响应区

- **Unary**：Tab = Response / Metadata / Trailers；状态徽章（如 `0 OK`）+ 耗时；JSON 正文；搜索可选后置
- **Streaming**：时间线（Sent / Received / Error / Info），默认最新在上；Clear（仅清视图）；连接状态文案

### 3.4 面板壳

- `PanelDockWidget` + `CreateRvizPanelTitleBarTools`（settings / expand / More）
- Settings 可进 Property Inspector（与 Service/Publish 一致）
- 右侧 Documentation（字段表）**可后置**到同一实现里程碑末尾或下一迭代；首版可用 Message 旁的 schema 提示代替

## 4. 架构

```
ui/grpc/                  Qt 主线程 UI
  GrpcPanel
  GrpcEditorWidget        连接栏 + Tab + 响应/时间线
  GrpcSettingsWidget
  grpc_types / config_io

integration/grpc/         工作线程 + gRPC
  GrpcSession             Channel、TLS、deadline、取消
  GrpcDescriptorStore     Reflection / FileDescriptorSet / 内置 rpcs
  GrpcInvoker             动态 unary + 三种 streaming
  GrpcJsonCodec           JSON ↔ DynamicMessage
```

### 4.1 动态调用

- 不按 service 生成 stub；使用 protobuf 描述符与 `GenericStub`（或 `ClientAsyncReaderWriter` 等异步 API + 完成队列）
- JSON 编解码复用/扩展现有 `protobuf_json_compat` 思路，面向 `DynamicMessage`

### 4.2 服务定义来源

1. **Server Reflection**：连接后拉取；失败则提示改用导入/内置
2. **导入 `.proto`**：用户选文件 + 可选 include 路径；编译进 `DescriptorPool`
3. **内置 automsgs RPC**：构建时打包或运行时定位 `automsgs/proto/rpcs/*.proto`（及 `common.proto` 依赖），一键 Load

三者写入同一 `GrpcDescriptorStore`，Method 下拉统一消费。

### 4.3 线程模型

- RPC 与反射在 **worker**（`QThread` 或专用线程 + completion queue）
- UI 只通过 queued signal 收：`streamEvent`、`unaryFinished`、`statusFinished`、`definitionUpdated`、`errorOccurred`
- 面板关闭 / 析构：abort 活跃 session，join/停止 worker

### 4.4 构建依赖

- CMake 选项例如 `AUTOVIZ_ENABLE_GRPC`（默认：找到 gRPC/protobuf 插件则 ON）
- OFF 时：仍注册面板，打开后显示「未启用 gRPC」；不链接 `grpc++`

## 5. 数据流与 Streaming 行为

### 5.1 Unary

`Invoke` → JSON 校验并转 protobuf → 带 metadata 发起 → 收到响应 → JSON + metadata/trailers + status/time → session 结束。

### 5.2 Server streaming

`Invoke` 建流 → 每条 Recv 追加 **Received** → 对端结束或用户取消/超时 → trailers + status。

### 5.3 Client streaming

`Invoke` 建流 → 每次 **Send** 发送当前 Message 编辑器内容（**Sent**）→ **End Streaming** 半关闭发送 → 等待最终响应消息（若有）与 status。

### 5.4 Bidirectional

Sent/Received 交错写入同一时间线；**End Streaming** 结束客户端发送；对端关闭或错误结束 session。

### 5.5 公共规则

- 每面板 **一个**活跃 session；再次 Invoke 前取消上一个（可确认或直接 abort，实现时选直接 abort 并提示）
- Message 非法：不发 RPC，状态栏/编辑器报错
- Clear：只清空时间线视图，不断开流

## 6. 错误处理

| 场景 | 行为 |
|------|------|
| 连接/TLS/超时 | 响应区 status（如 `14 UNAVAILABLE`）+ 短说明 |
| Reflection / proto 失败 | Service definition Tab 内联错误；Method 列表空 |
| 流中错误 | 时间线 Error 条目 + 结束 session |
| Worker 异常 | 捕获后 signal 回 UI，禁止跨线程直接碰 Qt 控件 |

## 7. 持久化

对齐 Publish 面板 session 模式：

- 保存：URL、TLS、选中 method、Message JSON、Metadata、Settings、定义来源（reflection / 文件路径列表 / automsgs）
- 不保存：时间线、一次性响应正文（可选：仅保存上次 unary 响应当「example」，首版可不做）
- `cloneConfigFrom` 支持 duplicate dock

## 8. 与现有面板关系

| 面板 | 关系 |
|------|------|
| Service Call | 并存；协议不同 |
| Teleop | 仍发 `/cmd_vel`；gRPC `TeleopService` 由本面板调试 |
| Publish / Raw | 无直接依赖；可复用 JSON/字段树视觉风格 |

## 9. 文件与接线（实现指引）

1. `autoviz/ui/grpc/*` — 面板 UI  
2. `autoviz/integration/grpc/*` — 客户端核心  
3. `panel_catalog.cpp` — `GrpcDock` 条目  
4. `visualization_frame.*` — create/wire/activate/duplicate/add-menu  
5. `resources/icons/panels/panel_grpc.svg` + `autoviz.qrc` + `icon_loader`  
6. `session_config` / `config_session` — 可选 PersistConfig  
7. `cmake` — `AUTOVIZ_ENABLE_GRPC` 与依赖探测  
8. `tests/` — codec / descriptor store 单测  

Sources 为 `GLOB_RECURSE`，一般无需手列 cpp。

## 10. 测试计划

- 单元：`GrpcJsonCodec` round-trip；method type 从 descriptor 解析；从 fixture `.proto` 加载 store  
- 手动：连 Postman Echo 或本地 bridge；覆盖四种 method（有环境时）  
- 编译矩阵：`AUTOVIZ_ENABLE_GRPC=OFF` 通过编译

## 11. 实现分期（建议）

| 阶段 | 交付 |
|------|------|
| M1 | 面板壳 + URL/TLS/Settings UI + 无 gRPC 占位 |
| M2 | DescriptorStore（proto 文件 + 内置 automsgs）+ Method 下拉 + Unary Invoke |
| M3 | Reflection；Metadata/Trailers；session 持久化 |
| M4 | Server / Client / Bidi streaming + 时间线 |
| M5 | 打磨（Example Message、错误文案、文档侧栏可选） |

## 12. 开放细节（实现计划中敲定，不阻塞本 spec）

- 异步 API 选型：completion queue vs 同步 stub 包在线程池（streaming 倾向 CQ）
- 内置 proto：资源嵌入 vs 安装路径查找
- Method 切换时是否自动生成 Example Message
