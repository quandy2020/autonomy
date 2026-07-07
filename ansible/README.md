# Ansible 一键部署

在 Ubuntu 22.04 目标机上部署 autonomy 全栈，支持**现场编译**与**制品分发**，以及 **staging / production** 多环境 inventory。

## 前置条件

```bash
pip install "ansible>=8,<10" "ansible-lint>=24,<26"
# 可选：Molecule 冒烟测试
pip install -r requirements-dev.txt
```

## 快速开始

### 选择环境

| 命令 | Inventory |
|------|-----------|
| `./deploy.sh` | `inventory/hosts.yml`（本机 local） |
| `./deploy.sh staging` | `inventory/staging/hosts.yml` |
| `./deploy.sh production` | `inventory/production/hosts.yml` |

```bash
cd ansible
./deploy.sh staging
```

### 制品部署（config 已捆绑进 tar）

```bash
../scripts/package_autonomy_artifact.sh --output ../dist/autonomy.tar.gz
# MANIFEST.json 中 config_bundled=true；production 从制品内安装 /etc/autonomy
./deploy.sh production
```

禁用捆绑：`BUNDLE_CONFIG=OFF ../scripts/package_autonomy_artifact.sh`

### 其他命令

```bash
ansible-playbook -i inventory/staging/hosts.yml playbooks/check.yml
./deploy.sh staging --tags config
ansible-playbook -i inventory/staging/hosts.yml playbooks/restart.yml
```

## 编译加速（ccache）

staging / build 模式默认启用 `autonomy_use_ccache: true`，缓存目录 `/var/cache/autonomy/ccache`。

```bash
./deploy.sh staging --tags build,ccache
```

## 密钥（ansible-vault）

```bash
./scripts/vault-init.sh
ansible-vault edit --vault-password-file .vault_pass inventory/group_vars/vault.yml
./deploy.sh production
```

## 测试

```bash
./scripts/ci.sh          # lint + syntax-check
./scripts/molecule.sh    # Docker 冒烟（common + config + services）
```

## 常用变量

| 变量 | 说明 |
|------|------|
| `autonomy_use_ccache` | build 模式启用 ccache |
| `autonomy_config_from_artifact` | 从制品内 `share/autonomy/config` 安装配置 |
| `autonomy_config_bundle_in_artifact` | 打包脚本是否捆绑 config |

## 目录结构

```text
ansible/
├── deploy.sh
├── molecule/default/     # Molecule 冒烟场景
├── scripts/
│   ├── ci.sh
│   ├── molecule.sh
│   └── vault-init.sh
└── roles/
    ├── ccache/
    ├── config/
    └── ...
```
