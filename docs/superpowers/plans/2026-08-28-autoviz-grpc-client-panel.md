# Autoviz gRPC Client Panel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 autoviz 新增独立 `GrpcDock` 面板，提供 Postman 风格的 gRPC 客户端（unary + 三种 streaming、Reflection / `.proto` / 内置 automsgs RPC）。

**Architecture:** Qt UI（`ui/grpc/`）经 queued signal 驱动 `integration/grpc/` 动态客户端：`GrpcDescriptorStore` 管定义，`GrpcSession`+`GrpcInvoker` 在 worker 线程用 `grpc::GenericStub` / 同步 ReaderWriter 发 RPC；JSON ↔ `DynamicMessage` 用 protobuf util。无 gRPC 库时 `AUTOVIZ_ENABLE_GRPC=OFF`，面板显示占位。

**Tech Stack:** C++17、Qt6 Widgets、protobuf、grpc++（可选）、GTest、现有 `PanelDockWidget` / `panel_catalog` 模式

**Spec:** `docs/superpowers/specs/2026-08-28-autoviz-grpc-client-panel-design.md`

**Build / test（后续步骤默认）：**

```bash
cd autoviz
cmake -B build -DBUILD_AUTOVIZ_TESTS=ON -DAUTOVIZ_ENABLE_GRPC=ON
cmake --build build -j --target autoviz
ctest --test-dir build -R 'grpc_' --output-on-failure
```

---

## File Map

| 路径 | 职责 |
|------|------|
| `autoviz/cmake/Config.cmake` | 增加 `AUTOVIZ_ENABLE_GRPC` option |
| `autoviz/cmake/Dependencies.cmake` | 探测/链接 gRPC |
| `autoviz/cmake/App.cmake` | 条件 `target_link_libraries` + `AUTOVIZ_ENABLE_GRPC` define |
| `autoviz/cmake/Tests.cmake` | 注册 `grpc_*` gtests |
| `autoviz/autoviz/integration/grpc/grpc_types.hpp` | MethodType、StreamEvent、SessionConfig 等纯类型 |
| `autoviz/autoviz/integration/grpc/grpc_json_codec.hpp/.cpp` | JSON ↔ DynamicMessage |
| `autoviz/autoviz/integration/grpc/grpc_descriptor_store.hpp/.cpp` | DescriptorPool：文件 / 内置 / reflection 入口 |
| `autoviz/autoviz/integration/grpc/grpc_session.hpp/.cpp` | Channel + TLS + 取消 |
| `autoviz/autoviz/integration/grpc/grpc_invoker.hpp/.cpp` | Unary + streaming 调用 |
| `autoviz/autoviz/integration/grpc/grpc_worker.hpp/.cpp` | QObject worker，信号回 UI |
| `autoviz/autoviz/ui/grpc/grpc_types.hpp/.cpp` | 面板 PersistConfig |
| `autoviz/autoviz/ui/grpc/grpc_config_io.hpp/.cpp` | session 读写 |
| `autoviz/autoviz/ui/grpc/grpc_settings_widget.hpp/.cpp` | Settings Tab 控件 |
| `autoviz/autoviz/ui/grpc/grpc_editor_widget.hpp/.cpp` | 连接栏 + 请求/响应 UI |
| `autoviz/autoviz/ui/grpc/grpc_panel.hpp/.cpp` | 面板壳 |
| `autoviz/autoviz/ui/panel_catalog.cpp` | 注册 `GrpcDock` |
| `autoviz/autoviz/ui/icon_loader.cpp` | 图标映射 |
| `autoviz/autoviz/ui/visualization_frame.hpp/.cpp` | create/wire/duplicate/add-menu |
| `autoviz/resources/icons/panels/panel_grpc.svg` | 图标 |
| `autoviz/resources/autoviz.qrc` | 资源 |
| `autoviz/tests/grpc_json_codec_test.cpp` | codec 单测 |
| `autoviz/tests/grpc_descriptor_store_test.cpp` | store 单测 |
| `autoviz/tests/fixtures/grpc_hello.proto` | 测试用最小 proto |

---

### Task 1: CMake 开关 `AUTOVIZ_ENABLE_GRPC`

**Files:**
- Modify: `autoviz/cmake/Config.cmake`
- Modify: `autoviz/cmake/Dependencies.cmake`
- Modify: `autoviz/cmake/App.cmake`

- [ ] **Step 1: 在 Config.cmake 增加 option**

在现有 `option(BUILD_AUTOVIZ_TESTS ...)` 附近加入：

```cmake
option(AUTOVIZ_ENABLE_GRPC "Build autoviz gRPC client panel (requires gRPC C++)" ON)
```

- [ ] **Step 2: 在 Dependencies.cmake 探测 gRPC**

在 `autonomy_require_protobuf()` 之后：

```cmake
set(_AUTOVIZ_HAS_GRPC OFF)
if(AUTOVIZ_ENABLE_GRPC)
  find_package(gRPC CONFIG QUIET)
  find_package(Protobuf CONFIG QUIET)
  if(TARGET gRPC::grpc++)
    set(_AUTOVIZ_HAS_GRPC ON)
  else()
    message(WARNING "AUTOVIZ_ENABLE_GRPC=ON but gRPC::grpc++ not found; gRPC panel disabled")
    set(AUTOVIZ_ENABLE_GRPC OFF)
  endif()
endif()
```

- [ ] **Step 3: App.cmake 条件链接**

对 `autoviz` 目标：

```cmake
if(AUTOVIZ_ENABLE_GRPC AND _AUTOVIZ_HAS_GRPC)
  target_compile_definitions(autoviz PRIVATE AUTOVIZ_ENABLE_GRPC=1)
  target_link_libraries(autoviz PRIVATE gRPC::grpc++)
  # 运行时查找仓库内 proto（相对 AUTOVIZ_DEPS_ROOT）
  target_compile_definitions(autoviz PRIVATE
    AUTOVIZ_AUTOMSGS_PROTO_ROOT="${AUTOVIZ_DEPS_ROOT}/automsgs/proto")
else()
  target_compile_definitions(autoviz PRIVATE AUTOVIZ_ENABLE_GRPC=0)
endif()
```

- [ ] **Step 4: 配置并确认 OFF 路径可编**

```bash
cmake -B autoviz/build -DAUTOVIZ_ENABLE_GRPC=OFF && cmake --build autoviz/build -j --target autoviz
```

Expected: 成功；无强制链接 grpc++。

- [ ] **Step 5: Commit**

```bash
git add autoviz/cmake/Config.cmake autoviz/cmake/Dependencies.cmake autoviz/cmake/App.cmake
git commit -m "build(autoviz): add AUTOVIZ_ENABLE_GRPC CMake option"
```

---

### Task 2: `grpc_types` + JSON codec（TDD）

**Files:**
- Create: `autoviz/autoviz/integration/grpc/grpc_types.hpp`
- Create: `autoviz/autoviz/integration/grpc/grpc_json_codec.hpp`
- Create: `autoviz/autoviz/integration/grpc/grpc_json_codec.cpp`
- Create: `autoviz/tests/fixtures/grpc_hello.proto`
- Create: `autoviz/tests/grpc_json_codec_test.cpp`
- Modify: `autoviz/cmake/Tests.cmake`

- [ ] **Step 1: 写 fixture proto**

`autoviz/tests/fixtures/grpc_hello.proto`:

```protobuf
syntax = "proto3";
package autoviz.test;
message HelloRequest { string greeting = 1; }
message HelloResponse { string reply = 1; }
service HelloService {
  rpc SayHello(HelloRequest) returns (HelloResponse);
  rpc LotsOfReplies(HelloRequest) returns (stream HelloResponse);
  rpc LotsOfGreetings(stream HelloRequest) returns (HelloResponse);
  rpc BidiHello(stream HelloRequest) returns (stream HelloResponse);
}
```

- [ ] **Step 2: 写失败测试 `grpc_json_codec_test.cpp`**

```cpp
#include <gtest/gtest.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/compiler/importer.h>
#include "autoviz/integration/grpc/grpc_json_codec.hpp"

using namespace autoviz::integration::grpc_client;

TEST(GrpcJsonCodec, RoundTripHelloRequest) {
  // 用 Importer 加载 fixtures/grpc_hello.proto（测试里设 include 路径）
  // ... 取得 HelloRequest descriptor ...
  std::string err;
  auto msg = JsonToDynamicMessage(R"({"greeting":"Cooper"})", desc, &err);
  ASSERT_NE(msg, nullptr) << err;
  std::string json;
  ASSERT_TRUE(DynamicMessageToJson(*msg, /*include_defaults=*/false, &json, &err))
      << err;
  EXPECT_NE(json.find("Cooper"), std::string::npos);
}

TEST(GrpcJsonCodec, InvalidJsonFails) {
  std::string err;
  auto msg = JsonToDynamicMessage("{", desc, &err);
  EXPECT_EQ(msg, nullptr);
  EXPECT_FALSE(err.empty());
}
```

（测试内用 `google::protobuf::compiler::DiskSourceTree` + `Importer` 加载 fixture；路径相对 `AUTOVIZ_ROOT/tests/fixtures`。）

- [ ] **Step 3: 注册测试目标**

在 `Tests.cmake`：

```cmake
_autoviz_display_gtest(grpc_json_codec_test
  ${AUTOVIZ_ROOT}/tests/grpc_json_codec_test.cpp
  ${AUTOVIZ_SRC_ROOT}/integration/grpc/grpc_json_codec.cpp)
target_compile_definitions(grpc_json_codec_test PRIVATE
  AUTOVIZ_TEST_FIXTURES_DIR="${AUTOVIZ_ROOT}/tests/fixtures")
```

- [ ] **Step 4: 运行确认失败**

```bash
cmake --build autoviz/build -j --target grpc_json_codec_test
# Expected: compile fail (missing headers) OR link fail
```

- [ ] **Step 5: 实现 types + codec**

`grpc_types.hpp`（节选）：

```cpp
namespace autoviz::integration::grpc_client {
enum class MethodType { kUnary, kClientStreaming, kServerStreaming, kBidiStreaming };
struct GrpcStatusView {
  int code = 0;  // grpc_status_code
  std::string message;
  int64_t latency_ms = 0;
};
enum class StreamEventKind { kSent, kReceived, kError, kInfo };
struct StreamEvent {
  StreamEventKind kind;
  std::string json_or_text;
  int64_t timestamp_ms = 0;
};
}  // namespace
```

`grpc_json_codec.cpp`：用 `google::protobuf::util::JsonStringToMessage` / `MessageToJsonString`，配合 `DynamicMessageFactory`；打印选项走 `autoviz/common/protobuf_json_compat.hpp` 的 `SetAlwaysPrintPrimitiveFields`。

- [ ] **Step 6: 测试通过并提交**

```bash
ctest --test-dir autoviz/build -R grpc_json_codec_test --output-on-failure
git add autoviz/autoviz/integration/grpc/ autoviz/tests/grpc_json_codec_test.cpp \
  autoviz/tests/fixtures/grpc_hello.proto autoviz/cmake/Tests.cmake
git commit -m "feat(autoviz): add gRPC JSON DynamicMessage codec"
```

---

### Task 3: DescriptorStore（文件 + method 列表）

**Files:**
- Create: `autoviz/autoviz/integration/grpc/grpc_descriptor_store.hpp/.cpp`
- Create: `autoviz/tests/grpc_descriptor_store_test.cpp`
- Modify: `autoviz/cmake/Tests.cmake`

- [ ] **Step 1: 写失败测试**

```cpp
TEST(GrpcDescriptorStore, LoadHelloProtoListsMethods) {
  GrpcDescriptorStore store;
  std::string err;
  ASSERT_TRUE(store.loadProtoFile(
      std::string(AUTOVIZ_TEST_FIXTURES_DIR) + "/grpc_hello.proto",
      {AUTOVIZ_TEST_FIXTURES_DIR}, &err)) << err;
  const auto methods = store.listMethods();
  ASSERT_GE(methods.size(), 4u);
  const auto* say = store.findMethod("autoviz.test.HelloService.SayHello");
  ASSERT_NE(say, nullptr);
  EXPECT_EQ(store.methodType(say), MethodType::kUnary);
  EXPECT_EQ(store.methodType(store.findMethod(
                "autoviz.test.HelloService.BidiHello")),
            MethodType::kBidiStreaming);
}
```

- [ ] **Step 2: 实现 store**

API：

```cpp
class GrpcDescriptorStore {
 public:
  bool loadProtoFile(const std::string& path,
                     const std::vector<std::string>& include_paths,
                     std::string* err);
  bool loadAutomsgsRpcs(std::string* err);  // 用 AUTOVIZ_AUTOMSGS_PROTO_ROOT
  // reflection：Task 6
  bool loadFromFileDescriptorSet(const google::protobuf::FileDescriptorSet&,
                                 std::string* err);
  std::vector<MethodInfo> listMethods() const;
  const google::protobuf::MethodDescriptor* findMethod(
      const std::string& full_name) const;
  MethodType methodType(const google::protobuf::MethodDescriptor*) const;
  std::string exampleJson(const google::protobuf::Descriptor* msg) const;
 private:
  google::protobuf::DescriptorPool pool_;
  google::protobuf::DynamicMessageFactory factory_;
};
```

`methodType`：根据 `client_streaming()` / `server_streaming()` 组合映射四种类型。

`loadAutomsgsRpcs`：扫描 `${AUTOVIZ_AUTOMSGS_PROTO_ROOT}/rpcs/*.proto`，include `proto` 根目录；测试里可对 `loadProtoFile` 单测，`loadAutomsgsRpcs` 在有源码树时手动验证。

- [ ] **Step 3: 测试通过并提交**

```bash
ctest --test-dir autoviz/build -R grpc_descriptor_store_test --output-on-failure
git add ... && git commit -m "feat(autoviz): add gRPC descriptor store for proto files"
```

---

### Task 4: 面板壳 + Catalog + Frame 接线（无 RPC）

**Files:**
- Create: `autoviz/autoviz/ui/grpc/grpc_types.hpp/.cpp`
- Create: `autoviz/autoviz/ui/grpc/grpc_panel.hpp/.cpp`
- Create: `autoviz/autoviz/ui/grpc/grpc_settings_widget.hpp/.cpp`
- Create: `autoviz/autoviz/ui/grpc/grpc_editor_widget.hpp/.cpp`（先做布局占位）
- Create: `autoviz/resources/icons/panels/panel_grpc.svg`
- Modify: `autoviz/resources/autoviz.qrc`
- Modify: `autoviz/autoviz/ui/panel_catalog.cpp`
- Modify: `autoviz/autoviz/ui/icon_loader.cpp`
- Modify: `autoviz/autoviz/ui/visualization_frame.hpp/.cpp`

- [ ] **Step 1: Catalog 条目**

在 `ServiceDock` 条目后插入：

```cpp
{"GrpcDock", "PanelGrpc", "gRPC",
 "Invoke gRPC methods (unary and streaming) with JSON payloads"},
```

- [ ] **Step 2: 图标**

复制 `panel_service.svg` 为 `panel_grpc.svg`（可改颜色/标注 gRPC）；`autoviz.qrc` 增加文件；`icon_loader.cpp`：

```cpp
{"PanelGrpc", "panels/panel_grpc"},
{"GrpcDock", "PanelGrpc"},
```

- [ ] **Step 3: GrpcPanel 骨架**

对照 `ServicePanel`：`installTitleBarTools`、`settingsToggled`、`configChanged`、`cloneConfigFrom`。  
`GrpcEditorWidget` 首版：TLS check、URL edit、method combo（空）、Invoke 按钮、Message `QPlainTextEdit`、Response `QPlainTextEdit`、底部状态 label。  
若 `#if !AUTOVIZ_ENABLE_GRPC`：中间大标签 “gRPC support was not enabled at build time”。

- [ ] **Step 4: Frame wiring**

仿 `createServicePanelDock` / `wireServicePanel` / `updateServiceDockTitle`：

- `createGrpcPanelDock`
- `wireGrpcPanel`
- `onAddPanelMenuTriggered` 分支 `GrpcDock`
- `duplicatePanelDock` / `activatePanelDock` 分支

- [ ] **Step 5: 手测**

```bash
./autoviz/build/bin/autoviz
```

Add Panel → gRPC → 出现 dock；settings 可切。

- [ ] **Step 6: Commit**

```bash
git commit -m "feat(autoviz): scaffold gRPC panel dock and catalog entry"
```

---

### Task 5: Session + Unary Invoke（需 AUTOVIZ_ENABLE_GRPC）

**Files:**
- Create: `autoviz/autoviz/integration/grpc/grpc_session.hpp/.cpp`
- Create: `autoviz/autoviz/integration/grpc/grpc_invoker.hpp/.cpp`
- Create: `autoviz/autoviz/integration/grpc/grpc_worker.hpp/.cpp`
- Modify: `grpc_editor_widget.*`、`grpc_panel.*`

- [ ] **Step 1: GrpcSession**

```cpp
class GrpcSession {
 public:
  bool connect(const std::string& target, bool use_tls,
               bool verify_cert, const std::string& ssl_override,
               std::string* err);
  std::shared_ptr<grpc::Channel> channel() const;
  void cancel();
};
```

`target`：去掉 `grpc://` 前缀，保留 `host:port`。TLS：`grpc::SslCredentials`；否则 `InsecureChannelCredentials`。

- [ ] **Step 2: GrpcInvoker::unaryCall**

使用 `grpc::GenericStub` + `ByteBuffer`：  
`DynamicMessage` → `SerializeToString` → `GrpcBuffer`；响应反序列化到 output descriptor → JSON。

同步调用包在 `GrpcWorker` 的 `QThread` 槽里，结果 `emit unaryFinished(json, status)`。

- [ ] **Step 3: UI 接线**

- Service definition：**Load .proto…** / **Load automsgs RPCs** → 填充 method combo（显示 `pkg.Service / Method`，userData=full_name）
- Invoke：abort 旧 worker 调用 → 新 unary  
- 显示 status `code name` + `latency_ms` + Response JSON

- [ ] **Step 4: 手测 unary**

对本地或 Postman Echo 等价服务；或用 `grpcurl` 自建 hello server。至少用 fixture proto + 自写最小 server（可选）或 bridge。

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(autoviz): gRPC unary invoke via dynamic GenericStub"
```

---

### Task 6: Reflection + Metadata/Trailers + 持久化

**Files:**
- Modify: `grpc_descriptor_store.*`（`loadFromReflection(channel)`）
- Modify: `grpc_invoker.*`（metadata 进出）
- Create: `autoviz/autoviz/ui/grpc/grpc_config_io.hpp/.cpp`
- Modify: `session_config` / `config_session`（若 Publish 有对应模式则照抄 `PublishPanelPersistConfig`）

- [ ] **Step 1: Reflection**

用 `grpc::reflection::v1alpha::ServerReflection` 客户端拉取 `FileDescriptorSet`，再 `loadFromFileDescriptorSet`。失败时 Service definition Tab 显示错误字符串。

- [ ] **Step 2: Metadata UI**

`QTableWidget` 两列 Key/Value；Invoke 时写入 `ClientContext::AddMetadata`。  
响应：从 `ClientContext` 取 initial/trailing metadata，填 Metadata/Trailers Tab（QPlainTextEdit 键值文本即可）。

- [ ] **Step 3: PersistConfig**

字段：`url`, `tls`, `method_full_name`, `message_json`, `metadata`（vector pair）, settings, `definition_mode`（`none|files|automsgs|reflection`）, `proto_paths`。  
`cloneConfigFrom` + session save/load 对齐 Publish。

- [ ] **Step 4: Commit**

```bash
git commit -m "feat(autoviz): gRPC reflection, metadata, and session persistence"
```

---

### Task 7: Streaming + 时间线

**Files:**
- Modify: `grpc_invoker.*`、`grpc_worker.*`、`grpc_editor_widget.*`

- [ ] **Step 1: Invoker API**

```cpp
// 均在 worker 线程运行；通过回调/信号推送 StreamEvent
bool startServerStream(...);
bool startClientStream(...);  // 之后 Send / WritesDone
bool startBidiStream(...);
bool streamSend(const std::string& json, std::string* err);
void streamHalfClose();  // client/bidi
void streamCancel();
```

实现策略（锁定）：**同步** `ClientReader` / `ClientWriter` / `ClientReaderWriter` 跑在 **专用 QThread**；`streamSend` 经互斥队列投递到该线程。Server stream 用读循环 `Read()` 直到 false。

- [ ] **Step 2: 时间线 UI**

`QListWidget` 或自定义 list：每项前缀 `[Sent]` / `[Received]` / `[Error]` + JSON/文本。最新插入顶部。Clear 只 `clear()` 列表。连接状态 label：`Idle` / `Streaming` / `Finished`。

- [ ] **Step 3: 按钮状态机**

| 状态 | 主按钮区 |
|------|----------|
| Idle + unary | Invoke |
| Idle + server-stream | Invoke |
| Idle + client/bidi | Invoke（开始流） |
| Streaming client/bidi | Send + End Streaming |
| Streaming server | End（取消） |

再次 Invoke：`streamCancel()` 后启动新 session，状态栏 Info「Previous call cancelled」。

- [ ] **Step 4: 手测三种 streaming**（有环境用 fixture service；无则至少编译 + UI 状态机）

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(autoviz): gRPC client/server/bidi streaming with timeline"
```

---

### Task 8: Example Message、错误打磨、文档小栏（可选）

**Files:**
- Modify: `grpc_descriptor_store`（`exampleJson`：标量填 0/`""`/false，message 递归一层）
- Modify: `grpc_editor_widget`（Use Example Message；method 切换且编辑器空时自动填）
- Modify: 错误文案统一到 status label

- [ ] **Step 1: Example Message 按钮与自动填充**
- [ ] **Step 2: 无 gRPC 构建手测 OFF 路径仍可打开面板**
- [ ] **Step 3: 更新 spec 状态行为「实现中/已完成」可选**
- [ ] **Step 4: Commit**

```bash
git commit -m "feat(autoviz): gRPC example messages and UX polish"
```

---

## Spec Coverage Check

| Spec 项 | Task |
|---------|------|
| 独立 GrpcDock / Catalog / 图标 | 4 |
| TLS + URL + Method + Invoke | 4–5 |
| Message / Metadata / Service definition / Settings | 4–6 |
| Unary | 5 |
| Server/Client/Bidi streaming + 时间线 | 7 |
| Reflection + proto + automsgs | 3, 6 |
| Session 持久化 | 6 |
| AUTOVIZ_ENABLE_GRPC 降级 | 1, 4 |
| 单测 codec/store | 2, 3 |
| 不做 Auth/Scripts/取代 Service | 未列入任务 |

## 开放细节（本计划已锁定）

- Streaming：**同步 stub + 专用 QThread + 发送队列**（不用 completion queue，降低复杂度）
- 内置 proto：运行时 `AUTOVIZ_AUTOMSGS_PROTO_ROOT`
- Method 切换：编辑器为空则自动 Example Message
