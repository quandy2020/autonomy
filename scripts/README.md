# scripts/

开发与 CI 用工具脚本。

| 路径 | 用途 |
|------|------|
| `install_deps/` | `python3 -m install_deps` 依赖安装 |
| `format.py` | C/C++ 代码格式化 |
| `package_autonomy_artifact.sh` | Ansible 制品打包 |

```bash
cd scripts
python3 -m install_deps
python3 -m install_deps --thirdparty-only --skip-installed
python3 format.py --check
```

依赖数据：`install_deps/data/*.json`；第三方安装脚本：`docker/install/`。
