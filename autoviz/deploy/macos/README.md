# macOS 部署

Autoviz 在 **macOS（Intel / Apple Silicon）** 上作为独立 Qt6 桌面应用构建，**不依赖 ROS**。仅需同级 `autolink` / `automsgs`。

完整说明见 [`docs/DEPLOYMENT.md`](../../docs/DEPLOYMENT.md)。

## 依赖（Homebrew）

```bash
brew install cmake ninja qt@6 yaml-cpp protobuf
# 可选：OpenGL 走系统框架；Ogre 后端见 DEPLOYMENT（默认关闭）
```

CMake 会通过 `EnsureProtobuf319.cmake` 自动选用 Homebrew 前缀（`/opt/homebrew`），
**不要求** Linux Docker 那套 `/usr/local` Protobuf 3.19 钉扎。若仍找不到：

```bash
export AUTONOMY_PROTOBUF_PREFIX="$(brew --prefix protobuf)"
# 或清理旧缓存后重配：
rm -rf build && cmake -B build -DCMAKE_PREFIX_PATH="$(brew --prefix qt@6);$(brew --prefix)"
```

Apple Silicon 上 Qt 前缀通常为 `/opt/homebrew/opt/qt@6`；Intel 为 `/usr/local/opt/qt@6`。

## 构建

在 **autoviz 包根**（或 autonomy 超工程根）执行：

```bash
# 独立工程
cmake -S . -B build \
  -DCMAKE_PREFIX_PATH="$(brew --prefix qt@6)" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build --target autoviz -j"$(sysctl -n hw.ncpu)"

./build/bin/autoviz
# C++ tutorials (built with autoviz):
# ./build/bin/examples/autoviz_cpp_01_poses --rate 10
```

超工程：

```bash
cmake -B build -DBUILD_AUTOVIZ=ON -DCMAKE_PREFIX_PATH="$(brew --prefix qt@6)"
cmake --build build --target autoviz
```

或：

```bash
python3 tools/configure.py --release -- \
  -DCMAKE_PREFIX_PATH="$(brew --prefix qt@6)"
python3 tools/build.py
```

## 运行时

| 变量 | 分隔符 | 说明 |
|------|--------|------|
| `AUTOVIZ_PLUGIN_PATH` | `:` | Display / Tool 等插件 |
| `AUTOVIZ_RESOURCE_PATH` | `:` | `package://` mesh 搜索前缀 |

可执行文件已设置 `@loader_path` RPATH（见 `cmake/App.cmake`），通常只需保证 `lib/` 与 `bin/autoviz` 相对布局正确。

若动态库仍找不到：

```bash
export DYLD_LIBRARY_PATH="$(pwd)/build/lib:${DYLD_LIBRARY_PATH:-}"
```

## Python 教程（examples/python）

推荐系统/Homebrew Python，避免 Isaac Sim 污染的解释器：

```bash
/usr/bin/python3 examples/python/01_tutorial_poses.py
# 或
export AUTOVIZ_PYTHON="$(brew --prefix python@3.12)/bin/python3"
```

`_bootstrap.py` 在 Darwin 上会设置 `DYLD_LIBRARY_PATH`（而非仅 `LD_LIBRARY_PATH`）。

## 打包（现状）

- 当前提供构建与运行说明；`.app` / DMG / notarization 流程尚未脚本化。
- 可参考 QGC [`deploy/macos`](https://github.com/mavlink/qgroundcontrol/tree/master/deploy/macos) 自行扩展 `macdeployqt`。

## 已知注意点

| 项 | 说明 |
|----|------|
| OpenGL | Qt 默认 OpenGL；较新 macOS 可能走兼容层，优先用自带 GPU 驱动 |
| Ogre | 可选；macOS 上默认建议 `AUTOVIZ_USE_OGRE=OFF` |
| Gatekeeper | 未签名二进制首次打开需「系统设置 → 隐私与安全性」允许 |
| ROS | 不需要；与 ROS 互通请用外部 `autonomy_ros`，Autoviz 仍只连 Autolink |
| Protobuf | 使用 Homebrew 35.x；JSON API / `string_view` 已做兼容 |
| 链接 | `glog` 经 `glog::glog` 导入目标（含 `/opt/homebrew/lib`） |
