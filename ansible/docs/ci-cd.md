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

成功应包含 idempotence、verify 的 `Successful`。测试使用小型构建夹具，不代表完整应用或硬件已验收。

## 首次配置 GitHub 发布（只做一次）

1. 配置原生构建 runner：标签 `self-hosted`、`linux`，另加 `autonomy-build-x86_64` 或 `autonomy-build-aarch64`。预装 CMake、Ninja、ccache、编译依赖及测试资源，基线须与目标匹配。
2. 修改 staging/production inventory，确认 GitHub 托管部署 runner 能通过网络访问目标，发布用户具有免交互 sudo 权限。
3. 创建 GitHub Environments `staging`、`production`，设置审批人和允许的发布分支；分别配置 `AUTONOMY_SSH_PRIVATE_KEY`、`AUTONOMY_SSH_KNOWN_HOSTS` secrets。
4. 审核 [lock.yml](../vars/lock.yml)，当前生产基线为 Ubuntu 22.04，默认不安装依赖。若开启锁定依赖安装，必须先填写精确 APT 版本。
5. 在分支保护中启用 [Ansible workflow](../../.github/workflows/ansible.yml) 的必需检查。

## 发布到 staging

1. 打开 GitHub Actions → **Autonomy release** → **Run workflow**。
2. 填写 `revision`（完整 40 位小写提交 SHA）、`architecture`（目标架构）、`limit`（先指定一台主机）。
3. `artifact_run_id` 留空，`deploy_environment` 选择 `staging`，触发并完成环境审批。
4. 等整个 run 成功，再验收目标平台功能；记录该 run 的数字 ID。`none` 只生成制品，不算 staging 发布。

## 推广到生产或回滚

1. 再次触发 **Autonomy release**。
2. 使用已成功 staging 的相同提交、架构；`artifact_run_id` 填该 staging run ID。
3. `deploy_environment` 选择 `production`，`limit` 填生产目标，提交审批。
4. 等整个 run 成功，再做平台验收。回滚时选择已验收旧版本的 staging run、原始提交和架构。

工作流下载同一制品并核对 staging 审计，不重新编译。审计文件即使存在也可能来自失败 run，应同时核对 run 成功状态。制品默认保留 30 天、审计 90 天，过期前按组织策略保存已批准原件。失败会停止后续机器，不自动回滚。

工作流源码：[质量门禁](../../.github/workflows/ansible.yml) · [发布](../../.github/workflows/ansible-release.yml)。
