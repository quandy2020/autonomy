# OrbisView Phase 2 Implementation Plan

> Spec: `docs/superpowers/specs/2026-09-13-orbisview-layered-migration-design.md` §12 阶段二

**Goal:** 浏览器可经 WebSocket 发现通道、订阅/取消订阅，并收到 `StreamEnvelope`（模拟源可独立验证；Autolink 可选）。

**Constraints:** 不依赖 foxglove-sdk；不阻塞 Autolink 回调；`thirdparty/` 仍不构建。

---

### Task 1: core 协议与会话

- [x] `StreamEnvelope` JSON 序列化
- [x] `ChannelInfo` / 订阅状态

### Task 2: streaming + WebSocket

- [x] `ThrottleQueue`（最新帧优先、有界、丢帧计数）
- [x] 轻量 POSIX WebSocket 服务（握手 + 文本帧）
- [x] 控制协议：`list_channels` / `subscribe` / `unsubscribe` / `status`

### Task 3: adapters

- [x] `MockSource` 定时注入信封
- [x] Autolink `ChannelDiscovery` + `RawMessage` 订阅（`ORBISVIEW_WITH_AUTOLINK`）

### Task 4: server + binary + mock page

- [x] `OrbisViewServer` 装配
- [x] `autonomy.orbisview` + `frontend/mock_client.html`
- [x] Python WS smoke：`SMOKE_OK`
