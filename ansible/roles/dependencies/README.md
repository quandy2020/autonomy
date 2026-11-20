# dependencies 能力契约

输入：autonomy_install_dependencies、autonomy_dependency_skip_installed、autonomy_source_dir。

输出：调用现有 install_deps；保留 APT 依赖步骤，以安装器 [RUN] 输出报告变更。

依赖：deployment_defaults；源码和基础部署工具须已准备。

# dependencies 角色

部署时安装系统与第三方依赖。**不重复** `install_deps` 的逻辑，只做调用封装。

## 分层

```text
scripts/install_deps/           # python3 -m install_deps
docker/install/*.sh             # 第三方安装脚本
roles/dependencies/             # Ansible：何时跑、传什么参数
```

## 文件

| 文件 | 说明 |
|------|------|
| `defaults/main.yml` | 包路径、`--skip-installed` 参数 |
| `files/install_dependencies.sh` | 薄包装：`cd scripts && python3 -m install_deps` |
| `tasks/main.yml` | build / artifact 两种 repo root |
| `tasks/install.yml` | 统一安装任务 |

## 变量

| 变量 | 默认 | 说明 |
|------|------|------|
| `autonomy_install_dependencies` | `true` (deployment_defaults) | 为 `false` 时整个角色被 playbook 跳过 |
| `autonomy_dependency_skip_installed` | `true` | 为 `true` 时传 `--skip-installed`（保留编译所需 APT 依赖） |
| `autonomy_dependency_package_marker` | `scripts/install_deps/app.py` | 检测包是否存在 |

## 行为

- **build 模式**：使用 `autonomy_source_dir` 下的 `scripts/install_deps/`
- **artifact 模式**：`rsync install_deps/` + `docker/install/` 到 `autonomy_build_dir`

机器人车队默认 `autonomy_install_dependencies: false`。
