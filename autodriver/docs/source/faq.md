# 常见问题

## 找不到配置文件

```
failed to load autodriver config: ... (optional override: export AUTODRIVER_PATH=<config parent>)
```

确认：

1. `AUTODRIVER_PATH` 指向**含 `config/` 的目录**（不是 `config` 本身）
2. 默认文件名为 `autodriver_hardware.yaml`
3. 安装树下检查 `share/autodriver/config/`

构建树开发时推荐：

```bash
export AUTODRIVER_PATH=$PWD/autodriver   # 在 src/autonomy 下
```

## 插件加载失败

- 确认 `LD_LIBRARY_PATH` 包含 `build/lib`
- `library` 字段与实际 `.so` 名称一致（如 `libautodriver_imu.so`）
- 自定义路径可设置 `plugin_dir` 或使用编译宏 `AUTODRIVER_PLUGIN_DIR`

## RealSense 不可用

构建日志中若无 `librealsense2 ... enabled`，则 RealSense backend 未编入。安装 Intel librealsense2 后重新配置 CMake，或关闭 `-DAUTODRIVER_WITH_REALSENSE=OFF`。

## CAN / 串口权限

Linux 下访问 `/dev/ttyUSB*`、`can0` 通常需要 dialout 组或 `sudo`。CAN 接口需先 `ip link set can0 up type can ...`。

## 与 Autolink 的关系

Autodriver **采集**；**发布**仅通过 `autodriver/bridge`。运行 hub 前需正确设置 `AUTOLINK_PATH` 与 Autolink 运行时配置，否则 Publisher 初始化可能失败。

## 文档构建

```bash
pip install -r autodriver/docs/requirements.txt
cmake --build build --target docs
```

需系统安装 `mkdocs`；未找到时 CMake 会禁用 `docs` 目标并打印提示。
