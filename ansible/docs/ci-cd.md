# CI/CD 操作步骤

日常单机部署直接使用 [部署步骤](README.md)。本页供仓库和发布维护者使用。

## 本地检查

1. 在 `ansible/` 准备开发环境（部署入口只自动安装部署工具）：

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements-dev.txt
ansible-galaxy collection install -r requirements.yml
```

2. 执行基础门禁，结尾应显示 `[ansible-ci] OK`：

```bash
./scripts/ci.sh
```

3. 需要集成验证时，在可运行 privileged systemd 容器的 Docker 宿主机串行执行：

```bash
MOLECULE_ROLE=services MOLECULE_IMAGE=jrei/systemd-ubuntu:22.04 ./scripts/molecule.sh
MOLECULE_ROLE=build MOLECULE_IMAGE=jrei/systemd-ubuntu:22.04 ./scripts/molecule.sh
MOLECULE_ROLE=services MOLECULE_IMAGE=jrei/systemd-ubuntu:24.04 ./scripts/molecule.sh
MOLECULE_ROLE=build MOLECULE_IMAGE=jrei/systemd-ubuntu:24.04 ./scripts/molecule.sh
```

成功应包含 idempotence、verify 的 `Successful`。GitHub Actions 还会使用隔离 Linux 容器验证完整归档安装、差分替换和无效制品拒绝。测试使用小型构建夹具，不代表完整应用或硬件已验收。

## 首次配置 GitHub 发布（只做一次）

1. 配置原生构建 runner：标签 `self-hosted`、`linux`，另加 `autonomy-build-x86_64` 或 `autonomy-build-aarch64`。预装 CMake、Ninja、ccache、编译依赖及测试资源，基线须与目标匹配。
2. 修改 staging/production inventory，确认 GitHub 托管部署 runner 能通过网络访问目标，发布用户具有免交互 sudo 权限。
3. 创建 GitHub Environments `staging`、`production`，设置审批人，并限制到同一受保护的发布分支。分别配置 `AUTONOMY_SSH_PRIVATE_KEY`、`AUTONOMY_SSH_KNOWN_HOSTS` secrets。
4. 审核 [lock.yml](../vars/lock.yml)，当前生产基线为 Ubuntu 22.04，默认不安装依赖。若开启锁定依赖安装，必须先填写精确 APT 版本。
5. 在分支保护中启用 [Ansible workflow](../../.github/workflows/ansible.yml) 的必需检查。

## 发布到 staging

选择 `cross_profile` 可以使用交叉构建。为对应 Profile 准备受信任的 Linux runner，标签为 `autonomy-cross-<profile>`，并预装 Docker、共享构建镜像、clang-format 和对应的 sysroot。将 sysroot 放在 runner 工具缓存的 `autonomy-cross/sysroots/<profile>/`。先提交并初始化第一方子模块，交叉构建还依赖与目标版本匹配的宿主代码生成工具。

工作流校验原制品摘要，并要求交叉制品附有 sysroot 摘要和镜像 ID。格式门禁失败时可从 workflow 下载 `autonomy-style-report`。staging 和 production 的每台目标都需在 inventory 配置 `autonomy_health_probe_argv`，例如指向已安装的只读心跳检查程序。

启用 Bridge 的主机可将 [probe_bridge.py](../scripts/probe_bridge.py) 安装到目标机的 `/usr/local/libexec/autonomy-bridge-probe`，权限设为 `0755`，再按实际安装路径和 Bridge 地址配置 inventory。探针检查只读心跳的应用状态、序号和时间戳。

```yaml
autonomy_health_probe_argv:
  - /usr/local/libexec/autonomy-bridge-probe
  - --bridge
  - /opt/autonomy/bin/autonomy.bridge
  - --target
  - 127.0.0.1:5005
```

1. 打开 GitHub Actions → **Autonomy release** → **Run workflow**。
2. 填写 `revision`（完整 40 位小写提交 SHA）、`architecture`（目标架构）、`limit`（先指定一台主机）。
3. `artifact_run_id` 留空，`deploy_environment` 选择 `staging`，触发并完成环境审批。
4. 等整个 run 成功，再验收目标平台功能；记录该 run 的数字 ID。`none` 只生成制品，不算 staging 发布。

## 推广到生产或回滚

1. 再次触发 **Autonomy release**。
2. 在同一发布分支选择与 staging 相同的提交、架构和 `cross_profile`，并填写该 staging run ID。
3. `deploy_environment` 选择 `production`，`limit` 填生产目标，提交审批。
4. 等整个 run 成功，再做平台验收。回滚时选择已验收旧版本的 staging run、原始提交和架构。

本地和 CI 都调用 `deploy.sh deploy`。生产阶段直接下载 staging 验收过的制品，核对 run 成功状态与 `accepted.json` 中记录的提交、Profile 和 SHA-256。旧 run 若只有尝试记录 `deployment-audit.json`，须重新执行 staging 验收。部署工具和 inventory 来自当前发布工作流的提交，审计同时记录制品和部署工具的提交。制品及验收记录默认保留 30 天，部署审计保留 90 天。失败时停止后续批次，并按[部署指南](deployment.md#更新应用版本)的条件恢复失败主机。

工作流源码：[质量门禁](../../.github/workflows/ansible.yml) · [发布](../../.github/workflows/ansible-release.yml)。
