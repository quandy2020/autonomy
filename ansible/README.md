# Ansible 一键部署

在 Ubuntu 22.04 目标机上部署 autonomy 全栈（localization + occupancy grid + task + bridge），支持**现场编译**与**制品分发**两种模式。

## 前置条件

- 控制机已安装 Ansible 2.14+：`pip install ansible`
- 目标机 Ubuntu 22.04，SSH 免密或 `--ask-become-pass`
- 目标机可访问 GitHub（build 模式拉源码）

## 快速开始

### 1. 配置 inventory

编辑 `inventory/hosts.yml`，设置 `ansible_host` 与 `ansible_user`：

```yaml
autonomy:
  hosts:
    robot01:
      ansible_host: 192.168.1.100
      ansible_user: ubuntu
```

### 2. 现场编译部署（默认）

在仓库根目录：

```bash
cd ansible
./deploy.sh
```

等价于：

```bash
ansible-playbook playbooks/site.yml
```

仅更新配置并重启服务：

```bash
./deploy.sh --tags config,services
```

### 3. 制品分发部署

控制机打包（无需写入本机 `/opt`）：

```bash
./scripts/package_autonomy_artifact.sh --output dist/autonomy.tar.gz
```

在 `inventory/group_vars/all.yml` 或 host vars 中设置：

```yaml
autonomy_deploy_mode: artifact
autonomy_artifact_path: /absolute/path/to/autonomy.tar.gz
```

部署：

```bash
cd ansible
./deploy.sh --tags artifact,config,services
```

## 路径约定

| 项 | 默认路径 |
|----|----------|
| 安装前缀 | `/opt/autonomy` |
| 运行时配置 | `/etc/autonomy` |
| 数据目录 | `/var/lib/autonomy` |
| 日志 | `/var/log/autonomy` |
| 运行用户 | `autonomy` |

## systemd 服务

| 单元 | 说明 |
|------|------|
| `autonomy-localization` | Cartographer SLAM |
| `autonomy-occupancy-grid` | 栅格地图发布 |
| `autonomy-task` | 任务调度 / 行为树 |
| `autonomy-bridge` | gRPC bridge |

通过 `autonomy_services` 列表启用/禁用（见 `inventory/group_vars/all.yml`）。

```bash
sudo systemctl status autonomy-task
sudo journalctl -u autonomy-localization -f
```

## 常用变量

| 变量 | 说明 |
|------|------|
| `autonomy_deploy_mode` | `build` \| `artifact` |
| `autonomy_repo_version` | Git 分支/标签 |
| `autonomy_build_grpc` | 是否编译 bridge |
| `autonomy_install_dependencies` | 是否执行 `install_dependency.py` |
| `autonomy_localization_configuration_basename` | Cartographer lua 配置 |

## Playbook 标签

| 标签 | 作用 |
|------|------|
| `common` | 用户、目录 |
| `source` | git clone（build） |
| `deps` | 安装依赖 |
| `build` | cmake + ninja install |
| `artifact` | 解压制品 |
| `config` | 同步 `config/` → `/etc/autonomy` |
| `services` | systemd 单元 |

## 目录结构

```text
ansible/
├── deploy.sh              # 一键入口
├── ansible.cfg
├── inventory/
│   ├── hosts.yml
│   └── group_vars/all.yml
├── playbooks/site.yml
└── roles/
    ├── common/
    ├── source/
    ├── dependencies/
    ├── build/
    ├── artifact/
    ├── config/
    └── services/
```
