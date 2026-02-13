# Autonomy Basic Examples

基础示例：Channel 发布/订阅、Action 服务端/客户端、Service 服务端/客户端，对应 autolink 的 talker/listener、action_server/action_client、service 能力。

## 目录结构

```
basic/
├── CMakeLists.txt
├── README.md
├── proto/
│   └── examples.proto      # 独立 proto（Driver、ExampleAction 等）
├── demo_channel_writer.cpp # Channel 写（Chatter，依赖 autolink unit_test）
├── demo_channel_reader.cpp # Channel 读
├── demo_action_server.cpp  # Action 服务端（example_action）
├── demo_action_client.cpp  # Action 客户端
├── demo_server_reader.cpp  # Service 服务端（test_server）
└── demo_server_writer.cpp  # Service 客户端
```

- **Channel**：使用 autolink 的 `unit_test.pb.h`（Chatter），话题 `channel/chatter`。
- **Action / Service**：使用本目录 `proto/examples.proto` 生成的消息（包名 `autolink.examples.proto`），可与 autolink 示例互通。

## 构建

在项目根目录配置并编译后，可执行文件生成在 build 目录，例如：

```bash
cd build
cmake ..
make
# 可执行文件：autonomy.basic.demo_channel_writer 等
```

## 运行

两两配对运行（先起 server/reader，再起 client/writer）。

**Channel（写 / 读）**

```bash
# 终端 1
./autonomy.basic.demo_channel_writer

# 终端 2
./autonomy.basic.demo_channel_reader
```

**Action（服务端 / 客户端）**

```bash
# 终端 1
./autonomy.basic.demo_action_server

# 终端 2：默认 target=10；可选参数：target_number [cancel_after_seconds]
./autonomy.basic.demo_action_client
./autonomy.basic.demo_action_client 5 2   # 计数到 5，2 秒后取消
```

**Service（服务端 / 客户端）**

```bash
# 终端 1
./autonomy.basic.demo_server_reader

# 终端 2
./autonomy.basic.demo_server_writer
```
