# 构建

依赖：**Qt 6**、**Ogre 1.x**（默认自动 vendor **1.12.10**）、**automsgs**、**autolink**、内嵌 **fkYAML**。可选 FFmpeg。视口不使用纯 OpenGL 后端。不链接 ROS / `libautonomy`。

构建系统为 [autocmake](../../autocmake/)：共享库目标 `autoviz`（`libautoviz`），可执行文件目标 `autoviz_app`（磁盘名 `bin/autoviz`）。

## 独立工程（推荐）

在 `autoviz/` 下配置时会内嵌同级 `autolink` / `automsgs`。

```bash
cd autoviz   # 或 src/autonomy/autoviz
python3 tools/configure.py --release
python3 tools/build.py
./build/bin/autoviz
```

等价 CMake：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target autoviz_app -j"$(nproc)"
```

| 平台 | 额外要点 |
|------|----------|
| **Linux** | Ubuntu 24.04；`./deploy/linux/install_deps.sh` 后 `./deploy/linux/build.sh --release` |
| **macOS** | `-DCMAKE_PREFIX_PATH="$(brew --prefix qt@6)"`；或 `./deploy/macos/build.sh`；见 [deploy/macos](../../deploy/macos/README.md) |
| **Windows** | MSVC + Qt6；`-DCMAKE_PREFIX_PATH=C:\Qt\6.x\msvc2019_64`；见 [deploy/windows](../../deploy/windows/README.md) |

macOS 示例：

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$(brew --prefix qt@6)"
cmake --build build --target autoviz_app
./build/bin/autoviz
```

## 工作空间 / 超项目

```bash
# autocmake 工作空间（按 package.xml 拓扑）
../autocmake/scripts/autocmake build --base-path . --packages-select autoviz

# 或 colcon
colcon build --packages-select autoviz

# Autonomy 根目录内嵌
cmake -B build -DBUILD_AUTOVIZ=ON
cmake --build build --target autoviz_app
```

## 常用 CMake 选项

| 选项 | 默认 | 说明 |
|------|------|------|
| `AUTOVIZ_OGRE_VENDOR` | OFF | FetchContent 构建 Ogre 1.12.10 |
| `AUTOVIZ_OGRE_AUTO_VENDOR` | **ON** | 系统非 1.12 时自动 vendor |
| `AUTOVIZ_USE_ASSIMP` | ON | Ogre mesh_loader 使用 Assimp |
| `AUTOVIZ_OGRE_ROOT` | — | 预编译 Ogre 前缀 |

Ogre 细节见 [rendering/ogre.md](../rendering/ogre.md)。

## 测试

`AUTOVIZ_BUILD_CORE_TESTS=ON` 会构建两组回归测试：

| 测试 | 覆盖 |
|------|------|
| `autoviz_interaction_test` | `MessageQueue` 最新值 / FIFO / 移动语义 / 全局清理 |
| `autoviz_mosaic_restore_test` | 会话 `MainPanelState` 恢复、嵌套 split、缺失面板占位与持久化 |
| `autoviz_session_config_test` | `.autoviz` 读写 round-trip：全局项、显示树/分组、属性、视图、工具与布局 blob |
| `autoviz_channel_payload_test` | Autolink 帧头剥离与异常回退（截断/异常 magic/短包） |
| `autoviz_ws_frame_test` | WebSocket 二进制帧：opcode + channel id + timestamp 字节序与回退 |
| `autoviz_display_registry_test` | 42 个已注册 Display 逐个工厂创建 + 默认属性完整性 |
| `autoviz_tf_buffer_test` | TF 插入、链式 lookup、revision 增量与 clear |
| `autoviz_ws_server_test` | 本机 WebSocket 握手 + 订阅 + 二进制广播（回环集成） |

```bash
# 一条命令：构建 + 跑测试（自动打开测试选项）
python3 tools/build.py --test

# 独立 CMake 构建
cmake -B build -DAUTOVIZ_BUILD_CORE_TESTS=ON
./scripts/run_autoviz_tests.sh

# Autonomy 工作空间（colcon 构建树）
AUTOVIZ_BUILD_DIR=build/autonomy ./src/autonomy/autoviz/scripts/run_autoviz_tests.sh

# 仅跑 ctest
ctest --test-dir build/autonomy -R autoviz --output-on-failure
```

脚本把 JUnit 写到 `<build>/autoviz-test-results/junit.xml`，不会混入其它包的历史 ctest XML。
`tools/build.py --test` 走同一套过滤与结果目录，并支持 `AUTOVIZ_BUILD_DIR` 指向 colcon 构建树。

## 产物布局

```text
build/
├── bin/autoviz              # 薄入口，链接 libautoviz
└── lib/libautoviz.*         # 核心共享库（autoviz::core）
```

## 相关文档

- [使用](usage.md) · [部署](deployment.md)
