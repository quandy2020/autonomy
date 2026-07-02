# 3. 开发脚本

项目根目录 `scripts/` 提供两类常用脚本。

### 3.1 format.py — 代码格式化

按根目录 `.clang-format`（Google 风格、4 空格缩进、80 列）格式化 `autonomy/` 与 `autolink/` 下 C/C++ 文件。

```bash
# 格式化全部（默认目录）
python3 scripts/format.py

# 仅检查，不修改（CI 用）
python3 scripts/format.py --check

# 预览将格式化的文件
python3 scripts/format.py --dry-run

# 指定目录
python3 scripts/format.py autonomy/navigator
python3 scripts/format.py autonomy autolink
```

自动查找 `clang-format` / `clang-format-17` … `clang-format-13`，macOS 亦支持 Xcode CLT 路径。

CMake 可选钩子：`google_format_code`（`cmake/functions.cmake`）在构建前调用此脚本。

### 3.2 install_dependency.py — 依赖安装

与 `docker/dockerfile/autonomy.x86_64.dockerfile` 保持同步，安装 APT 包与 `docker/install/*.sh` 第三方库。

```bash
# 完整安装
python3 scripts/install_dependency.py

# 仅 APT
python3 scripts/install_dependency.py --apt-only

# 仅第三方（/usr/local）
python3 scripts/install_dependency.py --thirdparty-only

# 列出 APT 包
python3 scripts/install_dependency.py --list-apt

# 从中断处恢复
python3 scripts/install_dependency.py --resume-from install_opencv.sh
```

主要第三方脚本包括：`install_glog.sh`、`install_protobuf.sh`、`install_ceres_solver.sh`、`install_opencv.sh`、`install_behaviortree_cpp.sh` 等。

### 3.3 注意事项

1. 格式化前建议先提交或备份代码
2. 全量格式化 1700+ 文件可能耗时较长
3. `install_dependency.py` 面向 Ubuntu；其他发行版需手动对照安装

### 3.4 相关文档

- [20 Other · 代码风格](../20_Other/02_code_style.md)
- [02 Installation · 依赖安装](../02_Installation/04_dependencies.md)
