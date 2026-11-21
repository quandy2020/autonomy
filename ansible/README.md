# Autonomy 部署

修改目标机器与编译配置后，在本目录执行：

```bash
./deploy.sh release robots -l robot01 -K
```

入口会准备控制机工具，串起预检、编译、打包、部署和服务验证。首次配置见 [使用指南](docs/README.md)。

## 支持的功能

| 功能 | 入口与行为 |
| --- | --- |
| 构建前检查 | `style` 检查第一方 C/C++ 的 Google 风格格式，任何错误或警告阻断构建。`check` 做常规预检，`plan` 只读检查目标、系统、架构、磁盘和所需命令，可用 `--release-config` 匹配构建目标。 |
| 本机和远程编译 | `build` 编译并安装，复用 CMake 构建目录，可启用 ccache 和 CTest。`full` 按 inventory 选择源码或制品部署，并配置服务。 |
| 模块编译 | `build --module` 和 `cross-build --module` 可重复选择 `automsgs`、`autolink`、`autodriver`、`autoviz`、`automanip`、`autonomy` 或 `autonomy/<domain>`。自动纳入依赖，独立缓存。未指定发布基线时，模块产物留在构建目录。 |
| 原生 Docker 发布 | `release` 使用已有构建容器编译、打包和部署，支持可选的第三方动态库及 Autoviz 插件打包。 |
| ARM 交叉发布 | `cross-build` 在控制机使用共享 Docker 镜像、Profile 和 sysroot 生成制品。`cross-release` 编译一次并发布给同 Profile 的目标。覆盖 RK3588、RK3506 和 Jetson Orin 系列的五个 Profile。 |
| 制品部署和增量分发 | `deploy` 分发已有归档。`--incremental` 复用目标上摘要一致的文件，同一基线可共用差分包。首次部署和差分无收益时传完整包。 |
| 模块增量部署 | `cross-release --module … --base-artifact …` 把模块运行文件合入基线并增量发布。适用于 `robots` 和 `staging`，要求活动版本与基线匹配并配置健康探针，保留现场配置和 systemd 单元。 |
| 日常运维 | `push` 同步配置，有变更时重启；`restart` 重启已有服务；`verify` 只读核对安装版本、服务和已配置的健康探针。 |
| 滚动发布与恢复 | 按批次激活，默认逐台。制品发布、配置推送和服务重启共用主机锁。健康检查失败会停止后续批次，并在有可恢复快照时还原状态并复检。 |
| CI 和生产晋级 | 提供 Ansible lint、语法检查和 Molecule 场景。生产经审批复用 staging 验收的同一制品，按提交、Profile 和 SHA-256 校验并保留审计。 |

## 配置与目录

- `inventory/` 管理本机、`robots`、`staging`、`production`、`sim` 和 `demo`，用 `-l` 选择主机或组。`demo` 支持用户目录安装。
- `vars/` 管理原生构建、交叉构建和版本锁定；`playbooks/` 编排流程，`roles/` 以 `openbot.autonomy_ops` Collection 提供可复用角色。
- `scripts/` 提供控制机初始化、制品校验、差分分发、部署审计、Vault 初始化及运行冒烟检查；`tests/` 和 `molecule/` 存放验证用例。

隔离回归已覆盖模块编译、差分、错误基线拒绝和探针失败恢复。真实 ARM 构建、板端 systemd 和 GitHub Actions 生产发布仍待验收。

[常用命令行参数](docs/cli.md) · [本机与远程编译](docs/getting-started.md) · [交叉编译与模块发布](docs/cross-release.md) · [日常操作与回滚](docs/deployment.md) · [CI/CD](docs/ci-cd.md) · [排障](docs/troubleshooting.md) · [设计](DESIGN.md)
