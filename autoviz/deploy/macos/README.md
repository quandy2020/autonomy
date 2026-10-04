# macOS 部署

Autoviz 在 **Intel / Apple Silicon** 上作为独立 Qt 6 桌面应用构建与分发，**不依赖 ROS**（仅需同级 `autolink` / `automsgs`）。

通用安装布局与环境变量见 [docs/guide/deployment.md](../../docs/guide/deployment.md)。

## 目录

| 文件 | 用途 |
|------|------|
| [`build.sh`](build.sh) | 一键 configure + build，可选打 `.app` / `.dmg` |
| [`create_app_bundle.sh`](create_app_bundle.sh) | 从 `build/` 生成 `Autoviz.app`（`macdeployqt`） |
| [`create_dmg.sh`](create_dmg.sh) | 从 `.app` 生成压缩 DMG |
| [`Info.plist.in`](Info.plist.in) | Bundle 元数据模板 |
| [`_common.sh`](_common.sh) | 路径 / 版本辅助 |

## 依赖（Homebrew）

```bash
brew install cmake ninja qt@6 yaml-cpp protobuf glog
# 可选：ffmpeg（视频解码）、assimp（mesh）
```

| 架构 | Qt 前缀 |
|------|---------|
| Apple Silicon | `/opt/homebrew/opt/qt@6` |
| Intel | `/usr/local/opt/qt@6` |

CMake / `tools/configure.py` 会在 macOS 上自动写入 `CMAKE_PREFIX_PATH`（若尚未设置）。若仍找不到 Qt：

```bash
export CMAKE_PREFIX_PATH="$(brew --prefix qt@6)"
# 或
rm -rf build
cmake -B build -DCMAKE_PREFIX_PATH="$(brew --prefix qt@6);$(brew --prefix)"
```

Protobuf 走 Homebrew（无需 Linux Docker 那套 3.19 钉扎）。可选：`export AUTONOMY_PROTOBUF_PREFIX="$(brew --prefix protobuf)"`。

## 开发构建

```bash
cd autoviz   # 包根

# 推荐
./deploy/macos/build.sh --release
./build/bin/autoviz

# 或 tools
python3 tools/configure.py --release
python3 tools/build.py
```

启用 Ogre（已默认，可省略）：

```bash
./deploy/macos/build.sh --release
```

超工程内：

```bash
cmake -B build -DBUILD_AUTOVIZ=ON -DCMAKE_PREFIX_PATH="$(brew --prefix qt@6)"
cmake --build build --target autoviz_app
```

## 打 .app / DMG

```bash
# 构建并打包
./deploy/macos/build.sh --release --app
./deploy/macos/build.sh --release --dmg          # 含 .app + DMG

# 已有 build/ 时单独打包
./deploy/macos/create_app_bundle.sh
./deploy/macos/create_dmg.sh
open dist/macos/Autoviz.app
```

产物默认在 `dist/macos/`：

```text
dist/macos/
├── Autoviz.app
└── Autoviz-<version>-<arch>.dmg
```

`.app` 布局（与运行时 `applicationDirPath()/../share/autonomy` 一致）：

```text
Autoviz.app/Contents/
├── MacOS/autoviz
├── Frameworks/          # Qt、libautoviz、autolink、Homebrew 依赖
├── PlugIns/             # Qt 平台插件（macdeployqt）
├── Resources/Autoviz.icns
├── share/autonomy/autoviz/
│   ├── default.autoviz
│   └── ogre_media/
└── Info.plist
```

签名：

```bash
# 默认 ad-hoc（-）
./deploy/macos/create_app_bundle.sh --sign "-"

# Developer ID（分发前）
./deploy/macos/create_app_bundle.sh \
  --sign "Developer ID Application: Your Name (TEAMID)"
```

公证（需 Apple 开发者账号，示意）：

```bash
xcrun notarytool submit dist/macos/Autoviz-*.dmg \
  --apple-id YOU@example.com --team-id TEAMID --password @keychain:AC_PASSWORD \
  --wait
xcrun stapler staple dist/macos/Autoviz-*.dmg
```

## 运行时

| 变量 | 分隔符 | 说明 |
|------|--------|------|
| `AUTOVIZ_PLUGIN_PATH` | `:` | Display / Tool 等插件 |
| `AUTOVIZ_RESOURCE_PATH` | `:` | `package://` mesh 搜索前缀 |
| `AUTOVIZ_OGRE_MEDIA_PATH` | — | Ogre 资源根（可选） |

`.app` 内已带 `share/autonomy`；开发树用 `build/bin` + `build/lib` 的 `@loader_path` RPATH，通常无需改 `DYLD_LIBRARY_PATH`。

若开发构建仍缺库：

```bash
export DYLD_LIBRARY_PATH="$(pwd)/build/lib:${DYLD_LIBRARY_PATH:-}"
./build/bin/autoviz
```

## 已知注意点

| 项 | 说明 |
|----|------|
| OpenGL | 仅作为 Ogre RenderSystem；无纯 GL 视口 |
| Ogre | **必需**（1.12；默认 auto-vendor） |
| Gatekeeper | 未公证二进制首次打开：系统设置 → 隐私与安全性 → 仍要打开 |
| ROS | 不需要；与 ROS 互通用外部 `autonomy_ros`，Autoviz 只连 Autolink |
| 依赖体积 | `macdeployqt` + 二次收束 Homebrew dylib；分发前在干净机器上验证 `otool -L` |
| 公证 | 脚本未内置 notarytool；见上文示意命令 |

## 相关文档

- [构建](../../docs/guide/build.md) · [部署总览](../../docs/guide/deployment.md) · [使用](../../docs/guide/usage.md)
