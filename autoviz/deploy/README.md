# Autoviz 部署

参照 [QGroundControl `deploy/`](https://github.com/mavlink/qgroundcontrol/tree/master/deploy) 布局，提供 Autoviz 专用打包与开发容器脚本。Autoviz 为**独立 CMake 工程**（`cmake -S autoviz -B build`），仅依赖同级的 `autolink` / `automsgs`，不构建 `libautonomy`。

## 目录

| 路径 | 用途 |
|------|------|
| [`docker/`](docker/) | 开发/CI 构建镜像与 `run-docker.sh` |
| [`linux/`](linux/) | Ubuntu：依赖安装、构建、AppDir / `.deb` |
| [`macos/`](macos/) | macOS：Homebrew 构建、`Autoviz.app` / DMG 打包脚本 |
| [`windows/`](windows/) | Windows 安装说明（见 [`docs/guide/deployment.md`](../docs/guide/deployment.md)） |

完整运行时依赖、环境变量与安装布局见 [`docs/guide/deployment.md`](../docs/guide/deployment.md)。

## 快速开始（Docker）

在 **已具备 Autonomy 第三方依赖** 的环境（如 SpaceHero 容器或 `src/autonomy/docker/` 构建的镜像）中：

```bash
# 从 autoviz 包根目录
./deploy/docker/run-docker.sh ubuntu Release

# 启用 Ogre（已默认；可省略）
./deploy/docker/run-docker.sh ubuntu Release
```

等价于容器内执行：

```bash
cd /project/source/autoviz
python3 tools/configure.py --release
python3 tools/build.py
```

产物：`autoviz/build/bin/autoviz`。

### 使用已有 Autonomy 镜像

若本机已有 SpaceHero / Autonomy 开发镜像，可跳过镜像构建，直接挂载源码：

```bash
docker run --rm -it \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -v /path/to/src/autonomy:/project/source \
  -w /project/source/autoviz \
  spacehero \
  bash -lc 'python3 tools/configure.py --release && python3 tools/build.py'
```

GUI 运行需挂载 X11/Wayland 与 GPU（`-e DISPLAY -v /tmp/.X11-unix` 等），见 [`docs/guide/deployment.md`](../docs/guide/deployment.md)。

## Linux / Ubuntu

推荐 **Ubuntu 24.04**（系统 Qt 6.4，含 `OpenGLWidgets`）。步骤见 [`linux/README.md`](linux/README.md)。

```bash
./deploy/linux/install_deps.sh
./deploy/linux/build.sh --release --deb
sudo apt install ./dist/linux/autoviz_*.deb
```

Monorepo（已有 `build/autonomy`）只打包：

```bash
cd src/autonomy/autoviz
./deploy/linux/create_deb.sh --build-dir ../../../../build/autonomy
# 或: AUTONOMY_BUILD_DIR=/path/to/build/autonomy ./deploy/linux/create_deb.sh
```

详见 [`linux/README.md`](linux/README.md)。

## 与主仓库 Docker 的关系

- **`src/autonomy/docker/`**：完整 Autonomy 栈（Ceres、OpenCV、gRPC 等 thirdparty 安装），SpaceHero 等开发环境。
- **`autoviz/deploy/docker/`**：Autoviz 专用 entrypoint（调用 `tools/configure.py` / `tools/build.py`）。默认基础镜像是 `ubuntu:22.04`（系统 Qt 6.2，没有 `OpenGLWidgets`）；本机部署请用 [`linux/`](linux/) 的 Ubuntu 24.04 流程。完整 configure 请在 SpaceHero 或 `AUTONOMY_BASE_IMAGE` 指向的镜像内运行。
