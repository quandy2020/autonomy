# scripts/

开发与 CI 用工具脚本。

| 路径 | 用途 |
|------|------|
| **`setup.bash`** | **统一环境变量**：`source scripts/setup.bash` |
| `install_deps/` | `python3 -m install_deps` 依赖安装 |
| `format.py` | C/C++ 代码格式化 |
| `package_autonomy_artifact.sh` | Ansible 制品打包 |

```bash
# 工作区环境（PATH / AUTONOMY_* / AUTOLINK_* / GLOG_* …）
source scripts/setup.bash

cd scripts
python3 -m install_deps
python3 -m install_deps --thirdparty-only --skip-installed
python3 format.py --check
```

依赖数据：`install_deps/data/*.json`；第三方安装脚本：`docker/install/`。

### `setup.bash` 要点

- 自动定位仓库根，设置 `AUTONOMY_ROOT` / `AUTONOMY_PATH` / `AUTONOMY_BUILD_DIR`
- 将 `build/bin`、`build/lib`（及可选 `AUTONOMY_INSTALL_PREFIX`）加入 `PATH` / `LD_LIBRARY_PATH`
- Autolink：`AUTOLINK_PATH`、`AUTOLINK_LAUNCH_PATH`、`AUTOLINK_CONF_PATH` / `DAG` / `LIB`、`AUTOLINK_DOMAIN_ID`
- Autonomy：`AUTONOMY_CONF_PATH`、`AUTONOMY_BT_PLUGIN_PATH`、`AUTONOMY_CONFIG_DIR`、`BRIDGE`
- Autodriver：`AUTODRIVER_PATH`
- glog 默认写到 `~/.autonomy/log` 并同时打到 stderr

覆盖示例：

```bash
export AUTONOMY_BUILD_DIR=$PWD/build/autonomy-minimal
export AUTONOMY_SETUP_QUIET=1
source scripts/setup.bash
```
