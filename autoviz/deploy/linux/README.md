# Linux / Ubuntu 部署

Autoviz 在 **Ubuntu 24.04** 上作为独立 Qt 6 桌面应用构建与分发，**不依赖 ROS**（仅需同级 `autolink` / `automsgs`）。

通用安装布局见 [docs/guide/deployment.md](../../docs/guide/deployment.md)。

## 目录

| 文件 | 用途 |
|------|------|
| [`install_deps.sh`](install_deps.sh) | `apt` 安装**构建**依赖 |
| [`install_runtime_deps.sh`](install_runtime_deps.sh) | `apt` 安装**运行时**依赖（目标机） |
| [`build.sh`](build.sh) | configure + build，可选 `--prefix` / `--bundle` / `--deb` |
| [`create_bundle.sh`](create_bundle.sh) | 可重定位 `AppDir` + `.tar.gz`（`AppRun`） |
| [`create_deb.sh`](create_deb.sh) | 安装到 `/usr` 的 `.deb`（选择性暂存） |
| [`AppRun`](AppRun) | AppDir 启动器 |
| [`debian/`](debian/) | `postinst` / `postrm` / `copyright` |
| `*.desktop.in` / `*.appdata.xml.in` / `*.mime.xml` | 桌面项、AppStream、MIME |

## 系统要求

| 发行版 | Qt | 说明 |
|--------|----|------|
| **Ubuntu 24.04** | 6.4 | 推荐；含 `Qt6::OpenGLWidgets` |
| Ubuntu 22.04 | 6.2 | 系统 Qt **没有** `OpenGLWidgets`。改用 24.04，或自备 Qt ≥ 6.4 并设置 `CMAKE_PREFIX_PATH` |

运行时不需要 ROS。与 ROS 互通用外部 `autonomy_ros`，Autoviz 只连 Autolink。

## 依赖与构建

独立树（`cmake -S autoviz`）：

```bash
cd autoviz
./deploy/linux/install_deps.sh
./deploy/linux/build.sh --release
./build/bin/autoviz
```

Monorepo（SpaceHero / `build/autonomy`）已编好时可直接打包：

```bash
cd src/autonomy/autoviz
./deploy/linux/create_deb.sh --build-dir /workspace/autonomy/build/autonomy
# 或依赖自动探测：
export AUTONOMY_BUILD_DIR=/workspace/autonomy/build/autonomy
./deploy/linux/create_deb.sh
```

`default_build_dir` 探测顺序：`AUTOVIZ_BUILD_DIR` → `autoviz/build` → `AUTONOMY_BUILD_DIR` → `<workspace>/build/autonomy`。

## Debian 包

```bash
./deploy/linux/build.sh --release --deb
# 或仅打包：
./deploy/linux/create_deb.sh

sudo apt install ./dist/linux/autoviz_0.1.0_amd64.deb
```

目标机若尚未装 Qt 运行时：

```bash
./deploy/linux/install_runtime_deps.sh
sudo apt install ./dist/linux/autoviz_*.deb
```

### 包内布局

| 路径 | 内容 |
|------|------|
| `/usr/bin/autoviz` | 启动包装脚本（设置 `AUTOVIZ_*` / `LD_LIBRARY_PATH`） |
| `/usr/lib/autoviz/autoviz` | 真实可执行文件 |
| `/usr/lib/autoviz/*.so*` | `libautoviz` / `libautolink` / `libautomsgs` / Ogre；以及 `/usr/local` 的 `libglog` / `libprotobuf` / `libassimp` 等 |
| `/usr/lib/autoviz/ogre/` | `RenderSystem_GL` / `Codec_STBI` 等插件 |
| `/usr/share/autonomy/autoviz/` | `default.autoviz`、`ogre_media/` |
| `/usr/share/autolink/conf/` | `autolink.pb.conf`（启动脚本设 `AUTOLINK_PATH`） |
| `/usr/share/applications/` | `.desktop` |
| `/usr/share/mime/packages/` | `*.autoviz` MIME |
| `/etc/ld.so.conf.d/autoviz.conf` | 私有库搜索路径（`postinst` 会 `ldconfig`） |

**不会**把整个 Autonomy monorepo（感知/规划 launch、OGRE 头文件等）打进 `.deb`。

`Depends` 使用 22.04/24.04 双兼容的 Qt 包名（`libqt6*t64 | libqt6*`）。SONAME 跨 LTS 不兼容的库（FFmpeg / `libglog` / `libprotobuf` / `libassimp`）会 vendor 进 `/usr/lib/autoviz`，因此可在 SpaceHero（22.04）打包后安装到本机 24.04。YAML 使用仓库内嵌的 fkYAML。

## 可重定位 AppDir

```bash
./deploy/linux/build.sh --release --bundle
tar -xzf dist/linux/autoviz-0.1.0-$(uname -m).tar.gz
./AppDir/AppRun
```

与 `.deb` 相同的选择性暂存；Qt 仍用系统包。

## 运行时环境变量

| 变量 | 分隔符 | 说明 |
|------|--------|------|
| `AUTOVIZ_PLUGIN_PATH` | `:` | Display / Tool 插件 |
| `AUTOVIZ_RESOURCE_PATH` | `:` | `package://` mesh 搜索前缀 |
| `AUTOVIZ_OGRE_MEDIA_PATH` | — | Ogre 资源根（包装脚本已设） |
| `AUTOVIZ_OGRE_PLUGIN_DIR` | — | Ogre 插件目录（包装脚本已设） |

## 已知注意点

| 项 | 说明 |
|----|------|
| Qt 版本 | 需要 Qt ≥ 6.4（`OpenGLWidgets`） |
| Wayland | 包装脚本默认 `QT_QPA_PLATFORM=xcb` |
| Ogre | Vendored 1.12 随包发布，避免与系统 Ogre 14 冲突 |
| 分发 | `.deb` / AppDir 链接系统 Qt，勿跨 Ubuntu 大版本直接拷贝 |

## 相关文档

- [构建](../../docs/guide/build.md) · [部署总览](../../docs/guide/deployment.md) · [Docker](../docker/)
