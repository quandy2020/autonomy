# tools/

全部为 Python 工具（格式化、打包、板端 Swap、CMake 检查）。

工作流入口（环境 / 依赖 / NFS）仍在 [`scripts/`](../scripts/)。

| Path | Purpose |
|------|---------|
| `clang_format_sources.py` | 按仓库 `.clang-format` 格式化 C/C++ |
| `package_install_tarball.py` | 构建并打包 install-tree（Ansible 制品） |
| `enable_board_swap.py` | 板端创建/启用 swapfile（`CONFIG_SWAP=n` 时退出） |
| `cmake/` | CMake preset / install-consumer 检查包 |

```bash
# From autonomy repo root (PYTHONPATH=. or just run as scripts)
python3 tools/clang_format_sources.py --check
python3 tools/package_install_tarball.py --output dist/autonomy.tar.gz
python3 tools/enable_board_swap.py

# CMake checks
python3 -m tools.cmake -v
# or
python3 -m unittest tools.cmake.test_presets tools.cmake.test_install_consumer -v
```
