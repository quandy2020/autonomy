# 部署

Autoviz 可在 Linux / macOS / Windows 独立分发，不要求目标机安装 ROS。

## 运行时依赖

| 组件 | 用途 |
|------|------|
| Qt 6 | Core / Gui / Widgets / OpenGL / Xml / Svg / Network |
| Ogre 1.x | 唯一视口后端（默认 auto-vendor 1.12.10） |
| libautolink / libautomsgs | 通信与消息（随包或同前缀安装） |
| yaml-cpp、glog、protobuf | 配置与日志 |
| FFmpeg（可选） | H264/H265/VP9 解码 |

## 推荐安装布局

```text
prefix/
├── bin/autoviz
├── lib/
│   ├── libautoviz.so|.dylib|.dll
│   ├── libautolink.*
│   └── libautomsgs.*
├── share/autonomy/autoviz/
│   ├── default.autoviz
│   └── ogre_media/            # Ogre 启用时
├── share/applications/        # Linux .desktop
└── lib/autoviz_plugins/       # 可选用户插件
```

可执行文件与共享库通过 `RPATH` / `@loader_path` 相对查找；通常无需改写 `LD_LIBRARY_PATH`。

macOS 若仍缺库：

```bash
export DYLD_LIBRARY_PATH="$PWD/build/lib:${DYLD_LIBRARY_PATH:-}"
./build/bin/autoviz
```

打包 `.app` / DMG：

```bash
./deploy/macos/build.sh --release --dmg
# 产物：dist/macos/Autoviz.app 、 Autoviz-<ver>-<arch>.dmg
```

详见 [deploy/macos/README.md](../../deploy/macos/README.md)。

Ubuntu 24.04 打包：

```bash
./deploy/linux/install_deps.sh
./deploy/linux/build.sh --release --deb
# 或可重定位目录：./deploy/linux/build.sh --release --bundle
```

详见 [deploy/linux/README.md](../../deploy/linux/README.md)。

## 平台说明

| 平台 | 文档 |
|------|------|
| Linux | [deploy/linux/README.md](../../deploy/linux/README.md)（依赖、AppDir、`.deb`） |
| macOS | [deploy/macos/README.md](../../deploy/macos/README.md)（`build.sh` / `.app` / DMG） |
| Windows | [deploy/windows/README.md](../../deploy/windows/README.md) |
| Docker | [deploy/docker/](../../deploy/docker/) |

## 插件与资源

- 插件：安装到 `lib/autoviz_plugins/`，运行前设置 `AUTOVIZ_PLUGIN_PATH`
- Mesh：设置 `AUTOVIZ_RESOURCE_PATH` 指向含 `package://` 解析前缀的目录树

## 相关文档

- [构建](build.md) · [使用](usage.md)
