# 3. 开发工具

**什么时候用：** 装依赖、格式化、或找仓库根常用脚本。

### 格式化

```bash
python3 tools/python/clang_format_sources.py
python3 tools/python/clang_format_sources.py --check
python3 tools/python/clang_format_sources.py --dry-run
```

依据根 `.clang-format`。

### 依赖

```bash
python3 -m install_deps
python3 -m install_deps --apt-only
python3 -m install_deps --thirdparty-only
python3 -m install_deps --resume-from install_opencv.sh
```

面向 Ubuntu 22.04；脚本在 `docker/install/`。gRPC 依赖随 `BUILD_GRPC=ON` 安装（见 Installation）。

### 其它

| 用途 | 位置 |
|------|------|
| 环境 | `scripts/setup_environment.bash` |
| Bridge 文档校验 | `docs/scripts/check_bridge_rpc_docs.py` |
| Bazel 包装 | `./autonomy.sh`（可选） |

### 常见坑

| 现象 | 说明 |
|------|------|
| install_deps 中途失败 | `--resume-from` 续跑 |
| format 改太多文件 | 单开 commit 或只 format  touched |
