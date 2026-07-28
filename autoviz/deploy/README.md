# Autoviz 部署

参照 [QGroundControl `deploy/`](https://github.com/mavlink/qgroundcontrol/tree/master/deploy) 布局，提供 Autoviz 专用打包与开发容器脚本。Autoviz 为**独立 CMake 工程**（`cmake -S autoviz -B build`），仅依赖同级的 `autolink` / `automsgs`，不构建 `libautonomy`。

## 目录

| 路径 | 用途 |
|------|------|
| [`docker/`](docker/) | 开发/CI 构建镜像与 `run-docker.sh` |
| [`linux/`](linux/) | `.desktop`、AppStream metainfo、AppRun、图标 |
| [`windows/`](windows/) | Windows 安装说明（见 [`docs/DEPLOYMENT.md`](../docs/DEPLOYMENT.md)） |

完整运行时依赖、环境变量与安装布局见 [`docs/DEPLOYMENT.md`](../docs/DEPLOYMENT.md)。

## 快速开始（Docker）

在 **已具备 Autonomy 第三方依赖** 的环境（如 SpaceHero 容器或 `src/autonomy/docker/` 构建的镜像）中：

```bash
# 从 autoviz 包根目录
./deploy/docker/run-docker.sh ubuntu Release

# 启用 QML Vehicle 3D + Ogre
AUTOVIZ_EXTRA_CONFIGURE="--qml --ogre" ./deploy/docker/run-docker.sh ubuntu
```

等价于容器内执行：

```bash
cd /project/source/autoviz
python3 tools/configure.py --release --qml
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
  bash -lc 'python3 tools/configure.py --release --qml && python3 tools/build.py'
```

GUI 运行需挂载 X11/Wayland 与 GPU（`-e DISPLAY -v /tmp/.X11-unix` 等），见 [`docs/DEPLOYMENT.md`](../docs/DEPLOYMENT.md)。

## Linux 桌面集成

安装后（`cmake --install autoviz/build --prefix /opt/autonomy`）可手动安装桌面文件：

```bash
cmake -S . -B build
cmake --build build --target autoviz
cmake --install build --prefix /opt/autoviz
```

或从模板生成：

```bash
sed -e 's/@AUTOVIZ_APP_NAME@/autoviz/g' \
    deploy/linux/org.autonomy.autoviz.desktop.in > org.autonomy.autoviz.desktop
```

## 与主仓库 Docker 的关系

- **`src/autonomy/docker/`**：完整 Autonomy 栈（Ceres、OpenCV、gRPC 等 thirdparty 安装），SpaceHero 等开发环境。
- **`autoviz/deploy/docker/`**：Autoviz 专用 entrypoint（调用 `tools/configure.py` / `tools/build.py`）与 **Qt6/QML 运行时包**；默认 `ubuntu:22.04` 基础镜像**不含** Autonomy thirdparty，完整 configure 请在 SpaceHero 或 `AUTONOMY_BASE_IMAGE` 指向的镜像内运行。
