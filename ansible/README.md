# Autonomy 运维与部署

采用一个 playbook、两种交付方式：开发机可现场编译；机器人分发经过测试的原生架构制品。共享设置位于 `roles/deployment_defaults/defaults/main.yml`，inventory、host_vars 和 `-e` 可以覆盖。完整设计和迁移说明见 [DESIGN.md](DESIGN.md)。

本目录是本地 Collection `openbot.autonomy_ops`（`galaxy.yml`）。`deploy.sh`、CI 和 Molecule 在运行前从当前源码安装本地 Collection，角色使用 FQCN，安装包仅提供运行角色及支持文件，排除 inventory、控制机入口/审计脚本、测试目录和 vault 密码文件，不发布到公共 Galaxy。

## 控制机准备

控制机使用 Python 3.12，目标机支持 Ubuntu 22.04 / 24.04、x86_64 / aarch64。staging、production 默认严格要求 Ubuntu 22.04。

```bash
cd ansible
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements-dev.txt
ansible-galaxy collection install -r requirements.yml
pre-commit install
./scripts/ci.sh
```

编辑 `inventory/{robots,staging,production}/hosts.yml`，填写实际地址和 SSH 用户，预先校验并登记目标机 SSH host key。生产发布用户需要适当的免交互 sudo 权限。不要将 SSH 私钥或明文密码放进 inventory；加密变量可由 `scripts/vault-init.sh` 初始化，`.vault_pass` 不入库。

## 日常操作

```bash
./deploy.sh check robots                         # 只读预检
./deploy.sh build                                # 本机源码编译安装
./deploy.sh build staging -l robot01              # 指定编译机，明确使用 build 模式
./deploy.sh push robots -l robot01                # 从控制机同步组件配置，重启并验证
./deploy.sh restart robots -l robot01             # 重启并验证
```

`./deploy.sh` 默认对本机做完整部署；`full` 使用 inventory 选定的 build/artifact 方式。`deploy` 强制制品模式，省略环境时使用 robots。显式环境参数优先于 `AUTONOMY_DEPLOY_ENV`。非交互执行不会自动询问 sudo 密码，可显式使用 `-K`。

`production.yml`、`staging.yml`、`sim.yml` 为场景入口，`site.yml` 导入统一的逐机事务编排。生产 profile 必须启用版本锁，通过受批准的 GitHub Actions Environment 发布；个人终端只允许 production 的 `check`。真正的权限边界是仅 CI 持有发布 SSH 凭据及 Environment 审批。

默认逐台部署，任一目标失败即停止后续目标。`-f 4` 控制连接并发，不改变 `serial: 1`；只有确认可并行的环境才使用 `-e autonomy_deploy_serial=4`。

## 编译一次，部署同一制品

以下打包命令必须在**实际编译机**执行。原生 x86_64 和 ARM64 分别构建；制品需与目标机 OS、第三方库基线保持一致，tarball 不是完整系统镜像。

```bash
# 编译机：使用独立 CMake 目录，避免复用 colcon 的缓存
./deploy.sh build staging -l robot01

# 登录上述编译机，在源码根目录执行；--build-dir 填该机器实际构建目录
python3 tools/python/package_install_tarball.py --skip-build \
  --build-dir /var/lib/autonomy/src/autonomy/build \
  --output /tmp/autonomy.tar.gz
# 将 tar.gz 和 tar.gz.sha256 取回控制机；同机执行则无需传输
```

也可以在 SpaceHero 已完成 colcon 编译后打包，复用 `build/autonomy`：

```bash
cd /workspace/autonomy/src/autonomy
python3 tools/python/package_install_tarball.py --skip-build \
  --build-dir /workspace/autonomy/build/autonomy \
  --output /tmp/autonomy.tar.gz
```

打包只执行暂存安装，不写编译机 `/opt/autonomy`，同时生成包含 Git SHA、架构、前缀及实际 CMake 缓存选项的 manifest 与 SHA-256 sidecar。配置来自实际 `autonomy/*/conf/`，归档内布局为 `share/autonomy/config/<component>/`。

```bash
# 控制机：验证收到的制品，建议先限制一台
cd /path/to/artifacts
sha256sum --check autonomy.tar.gz.sha256
artifact_sha=$(cut -d ' ' -f 1 autonomy.tar.gz.sha256)
cd /path/to/autonomy/ansible
./deploy.sh deploy robots -l robot01 \
  -e autonomy_artifact_path=/path/to/artifacts/autonomy.tar.gz \
  -e "autonomy_artifact_sha256=$artifact_sha"
```

完整部署先在控制机校验归档路径、链接、manifest，再检查目标架构和传输校验和。在同一文件系统的临时目录解压，验证所选服务二进制后发布到 `/opt/autonomy-releases/<sha256>`，最后切换 `/opt/autonomy` 链接。重复部署同一制品不会重复解压；安装目录不会混入旧版本文件。只允许同一目标有一个发布进程。

首次从真实目录 `/opt/autonomy` 迁移时，旧目录移动到 release 目录下的 `legacy-<epoch>`，请安排维护窗口。后续版本切换使用链接。上一版本由 `/opt/autonomy.previous` 保留；历史版本不会自动删除，应监测磁盘并在确认版本不再被引用后人工清理。

构建安装禁止写入制品链接，避免修改已发布版本；开发编译与制品部署请使用不同主机或不同安装前缀。

## 配置与服务

默认启用 localization、task、bridge，实际运行 `autonomy.localization`、`autonomy.task`、`autonomy.bridge`。bridge 使用 `--conf=/etc/autonomy/bridge/bridge.pb.txt`。`occupancy_grid` 保留为可选项，只有制品包含对应可执行文件时才启用。

完整制品部署从制品复制配置；`push` 始终从控制机当前源码复制 `autonomy/{localization,task,bridge}/conf/`，不会重新复制机器人上的旧 bundle。文件权限为 0640，目录为 0750。配置目录不做整目录删除，机器人专有文件须单独管理；同名文件会更新，标定配置上线前需要核对。

单机配置可在 `inventory/<environment>/host_vars/<host>.yml` 覆盖 `autonomy_services`、组件配置路径及 `autonomy_config_components`。默认完整部署会在二进制、配置或 unit 变化后重启服务，并检查 `systemctl is-active`。`autonomy_restart_on_config_change: false` 仅关闭单独 push 的自动重启。重启模式遇到缺失 unit 会报错。

现场诊断：

```bash
systemctl status autonomy-localization autonomy-task autonomy-bridge
journalctl -u autonomy-task -n 100 --no-pager
readlink -f /opt/autonomy
cat /opt/autonomy/share/autonomy/MANIFEST.json
```

CI 不输出运行时 journal 内容，以免夹带敏感信息。检查 active 只能证明进程已启动，不能替代定位有效性、传感器数据和机器人动作验收。

## 版本锁

`use_locked_versions: true` 显式启用 `roles/version_lock`，生产默认强制开启。锁文件 `vars/lock.yml` 与代码同仓：锁定 OS/架构策略，源码编译要求 40 位提交，生产完整制品发布还要求 64 位 SHA-256 和匹配的 `autonomy_expected_git_sha`。这些身份由 release workflow 传入。

需要 Ansible 安装依赖的锁定场景，还必须从已批准的平台基线填入 `apt_packages` 的**精确版本**；角色写入优先级 1001 的 APT preferences，允许回到该版本。当前没有伪造目标机的包版本，所以映射留空；锁定模式下打开依赖安装会明确失败。制品模式默认不安装依赖，运行时第三方 ABI 由匹配的构建机/目标机基线提供。

```bash
./deploy.sh check production
./deploy.sh build staging -e use_locked_versions=true \
  -e autonomy_repo_version=YOUR_APPROVED_40_CHARACTER_COMMIT
# 安装依赖前，先按实际平台完善 vars/lock.yml；占位值不能执行
```

保留现有 `autonomy_*` 参数兼容旧 inventory，这是有注释、有检查的命名例外；不同时批量重命名调用方。通用应用目录/用户能力已独立为 `runtime_bootstrap`，不再扩张 common 角色。

## CI/CD

- `.github/workflows/ansible.yml`：PR/push 运行真实模块 lint、语法检查、入口/打包测试、inventory 覆盖测试、最小 CMake 构建安装，以及 Molecule 的 Ubuntu 22.04/24.04 × services/build 矩阵。10 个角色各有独立 converge 入口和能力契约，主矩阵通过显式前提链覆盖角色。
- `.github/workflows/ansible-release.yml`：先复用完整质量门禁，再手动指定完整 Git SHA 与原生架构；在准备好的 self-hosted builder 构建并运行 CTest，随后打包、上传同一构建结果。默认仅生成制品。`artifact_run_id` 可选择历史 run 的同名制品，跳过重新构建。production 必须引用一个**成功完成 staging 发布**的 run：工作流读取其成功结论及 staging 审计，核对同一提交和制品摘要后才允许推广/回滚。staging 复用已有制品时，也会保留相同字节供后续推广。
- 发布工作流选择 staging/production 时，deploy job 使用对应 GitHub Environment。启用前必须在仓库设置中配置 required reviewers、允许发布的分支，以及该环境的 `AUTONOMY_SSH_PRIVATE_KEY`、`AUTONOMY_SSH_KNOWN_HOSTS`。不要给 PR job 使用生产 SSH 凭据或生产构建 runner。
- 异构车队使用 workflow 的 `limit` 选择实际 inventory 中的主机/架构组；默认 `autonomy`。审计会记录选择器并拒绝空目标清单，制品仍逐机核对实际架构。组标签须来自真实资产资料，不预填猜测的 CPU 架构。
- 每次发布上传允许列表式 `deployment-audit.json`：代码提交、inventory Git tree、锁文件摘要、制品摘要/来源 run、执行者、目标 host 清单和公开额外参数；不导出完整 inventory 变量、环境变量或秘密。
- 构建 runner 标签为 `self-hosted, linux, autonomy-build-x86_64` 或 `autonomy-build-aarch64`，预装第三方依赖、CMake、Ninja、ccache、测试所需资源，保持 ABI 基线一致。正式使用前应确认所选提交已包含这些工作流和脚本。

```bash
./scripts/ci.sh
./scripts/molecule.sh                # 默认 services 综合场景；需要 systemd 容器
MOLECULE_ROLE=build MOLECULE_IMAGE=jrei/systemd-ubuntu:24.04 ./scripts/molecule.sh
# 也可选择 source / dependencies / artifact / config / version_lock 等独立角色场景
# 可选：使用已有隔离容器测试真实制品角色，不接触生产机器
AUTONOMY_TEST_CONTAINER=my-test-container \
  python3 -m unittest discover -s tests -p test_artifact_lifecycle.py -v
```

生产发布使用完整模式，避免用 `--tags artifact` 等局部标签跳过预检、配置或服务验收。`--check` 是 Ansible 的预测执行，不证明一个尚未安装的制品能成功运行。

## 回滚

健康检查失败会停止车队后续发布，不自动驱动机器人恢复动作。通过受批准的 release workflow 指定旧的 `artifact_run_id`、提交和架构，重新部署上一次通过验收的 tarball 及其校验和，这会同时恢复该制品的二进制和捆绑配置，再重启、验证。不要只切换二进制链接而保留不兼容的新配置。运行数据、地图、日志不在制品目录中，不参与清理或回滚。

## 采用参考规范的边界

入口、Collection、精确依赖、显式角色依赖、版本锁、CI 矩阵、审计和 pre-commit 均有代码门禁。`requirements-lock.txt` 固化控制机的传递依赖解析结果；升级时须在干净 Python 3.12 环境重新解析并跑门禁。保留既有 `inventory/` 命名及本机入口，生产/staging/sim 各自独立，避免仅为复数目录名迁移调用方。

尚未连接真实 CMDB/AWX，也没有自动删除历史版本或自动回滚机器人。动态 inventory 可作为 `ansible-playbook -i` 输入接入；在真正接入并记录 inventory 快照前，不宣称支持数百节点的资产同步。当前活跃状态检查仍不是硬件业务验收。
