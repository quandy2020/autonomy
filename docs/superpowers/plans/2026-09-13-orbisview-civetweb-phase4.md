# OrbisView CivetWeb + Phase 4

**Goal:** 本工程原生重写；CivetWeb 承载 HTTP/WS；阶段四工程分析能力落地。

- [x] Vendor CivetWeb v1.16 → `deps/civetweb`（与 Apollo `thirdparty/` 隔离）
- [x] 删除自研 POSIX WS；`OrbisCivetWsHandler` + `/ws`
- [x] 频率/延迟 `channel_stats`、录制回放 JSONL、插件注册
- [x] 前端：Inspector / Diagnostics / Stats / Recorder / Exploration / Navigation / Mapping
- [x] 验证：`CIVET_P4_SMOKE_OK` + `npm run build`
