# Linux / Ubuntu 部署

Autoviz 在 **Ubuntu 24.04** 上作为独立 Qt 6 桌面应用构建与分发，**不依赖 ROS**（仅需同级 `autolink` / `automsgs`）。

通用安装布局见 [docs/guide/deployment.md](../../docs/guide/deployment.md)。

## 目录

| 文件 | 用途 |
|------|------|
| [`install_deps.sh`](install_deps.sh) | `apt` 安装构建依赖 |
| [`build.sh`](build.sh) | configure + build，可选 `--prefix` / `--bundle` / `--deb` |
| [`create_bundle.sh`](create_bundle.sh) | 可重定位 `AppDir` + `.tar.gz`（`AppRun`） |
| [`create_deb.sh`](create_deb.sh) | 安装到 `/usr` 的 `.deb` |
| [`AppRun`](AppRun) | 可重定位前缀的启动器 |
| `*.desktop.in` / `*.appdata.xml.in` | 桌面项与 AppStream（CMake `install` 生成） |

## 系统要求

| 发行版 | Qt | 说明 |
|--------|----|------|
| **Ubuntu 24.04** | 6.4 | 推荐；含 `Qt6::OpenGLWidgets` |
| Ubuntu 22.04 | 6.2 | 系统 Qt **没有** `OpenGLWidgets`。改用 24.04，或自备 Qt ≥ 6.4 并设置 `CMAKE_PREFIX_PATH` |

运行时不需要 ROS。与 ROS 互通用外部 `autonomy_ros`，Autoviz 只连 Autolink。

## 依赖与构建

```bash
cd autoviz

./deploy/linux/install_deps.sh          # sudo apt
./deploy/linux/build.sh --release
./build/bin/autoviz
```

启用 Ogre（已默认，可省略 `--ogre`）：

```bash
./deploy/linux/install_deps.sh
./deploy/linux/build.sh --release
```

安装到前缀（含 `.desktop`、图标、`share/autonomy/autoviz`）：

```bash
./deploy/linux/build.sh --release --prefix /opt/autoviz
/opt/autoviz/bin/autoviz
```

等价 CMake：

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --target autoviz_app -j"$(nproc)"
cmake --install build --prefix /opt/autoviz
```

## 可重定位目录

项目库放在 `AppDir/lib`，Qt 等仍用系统包（与构建机同一 Ubuntu 版本）。

```bash
./deploy/linux/build.sh --release --bundle
tar -xzf dist/linux/autoviz-0.1.0-$(uname -m).tar.gz
./AppDir/AppRun
```

`AppRun` 设置：

| 变量 | 默认 |
|------|------|
| `LD_LIBRARY_PATH` | `$APPDIR/lib`（及 multiarch） |
| `AUTOVIZ_RESOURCE_PATH` | `$APPDIR/share/autonomy` |
| `AUTOVIZ_PLUGIN_PATH` | `$APPDIR/lib/autoviz_plugins` |
| `QT_QPA_PLATFORM` | Wayland 会话下默认 `xcb` |

开发树已设 `$ORIGIN/../lib` RPATH，直接跑 `./build/bin/autoviz` 通常不必改 `LD_LIBRARY_PATH`。

## Debian 包

```bash
./deploy/linux/build.sh --release --deb
sudo apt install ./dist/linux/autoviz_0.1.0_amd64.deb
```

包安装到 `/usr`（`bin/autoviz`、库、desktop、metainfo）。`Depends` 由 `dpkg-shlibdeps` 根据本机共享库生成，应在目标发行版上打包。

## 运行时

| 变量 | 分隔符 | 说明 |
|------|--------|------|
| `AUTOVIZ_PLUGIN_PATH` | `:` | Display / Tool 等插件 |
| `AUTOVIZ_RESOURCE_PATH` | `:` | `package://` mesh 搜索前缀 |
| `AUTOVIZ_OGRE_MEDIA_PATH` | — | Ogre 资源根（可选） |

桌面项：`org.autonomy.autoviz.desktop`（`Exec=autoviz`）。安装到非 `/usr` 时把 `bin` 加入 `PATH`，或自行改 `Exec`。

## 已知注意点

| 项 | 说明 |
|----|------|
| Qt 版本 | 需要 Qt ≥ 6.4（`OpenGLWidgets`） |
| Wayland | `AppRun` 默认 `QT_QPA_PLATFORM=xcb`；要用 Wayland 时自行覆盖 |
| Ogre | 可选；系统 Ogre 14 与 rviz GLSL 不等价，见 [rendering/ogre.md](../../docs/rendering/ogre.md) |
| Protobuf | Linux Docker 若存在 `/usr/local` 的 3.19，CMake 会优先钉扎；本机 apt 则用发行版 protobuf |
| 分发 | `.deb` 与 AppDir 都链接系统 Qt，不要拿到不同 Ubuntu 版本上直接跑 |

## 相关文档

- [构建](../../docs/guide/build.md) · [部署总览](../../docs/guide/deployment.md) · [Docker](../docker/)
