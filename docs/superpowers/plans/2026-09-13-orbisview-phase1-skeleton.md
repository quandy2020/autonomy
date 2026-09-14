# OrbisView Phase 1 Implementation Plan

> **For agentic workers:** Execute task-by-task. Spec: `docs/superpowers/specs/2026-09-13-orbisview-layered-migration-design.md`

**Goal:** 完成设计「阶段一：上游归档与工程骨架」验收条件。

**Architecture:** `autonomy/orbisview/` 分层（core/backend/frontend/rendering/adapters/plugins）+ `thirdparty/` 隔离上游；CMake 用 `BUILD_ORBISVIEW` 门控，`thirdparty/` 永不 `add_subdirectory`。

**Tech Stack:** CMake, C++17 stubs, npm package.json（不提交 node_modules）

**Out of scope this plan:** 阶段二 WebSocket / StreamEnvelope 实现；阶段三面板迁移；阶段四录制回放。

---

### Task 1: 目录与许可证

- [x] 按设计 §5 创建分层目录
- [x] `README.md`、`LICENSES/`（Apache-2.0 + 上游追溯）

### Task 2: 上游快照

- [x] rsync Apollo `modules/dreamview` → `thirdparty/dreamview`（排除 dist/node_modules/.cache）
- [x] 同理 `dreamview_plus`
- [x] 各目录 `ORIGIN.md`（repo URL + commit `d53aa3da…`）

### Task 3: CMake 骨架

- [x] 根 `CMakeLists.txt`：`option(BUILD_ORBISVIEW …)` + companion `add_subdirectory`
- [x] 各层 INTERFACE library；`thirdparty/` 无 add_subdirectory
- [x] `frontend/` 不参与 C++ 链接，仅 `orbisview_frontend_hint`

### Task 4: 前端入口

- [x] `frontend/package.json` + README；gitignore `node_modules`/`dist`

### Task 5: 验证

- [x] 树与设计一致；`cmake -S autonomy/orbisview -B …` configure + `orbisview_all` 成功
