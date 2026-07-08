# Ansible 部署

Ubuntu 22.04+ 目标机一键部署 autonomy。单一 playbook（`site.yml`）+ `deploy.sh` 模式切换。

## 安装

```bash
pip install "ansible>=8,<10" "ansible-lint>=24,<26"
```

## 模式

所有模式共用 `playbooks/site.yml`，由 `autonomy_play_mode` 控制：

| 命令 | 说明 |
|------|------|
| `./deploy.sh` | 全量：编译或制品 + 配置 + 服务 |
| `./deploy.sh build` | 仅编译安装到 `/opt/autonomy` |
| `./deploy.sh deploy robots -e autonomy_artifact_path=...` | 制品推到车队 |
| `./deploy.sh push robots` | 只同步配置并重启 |
| `./deploy.sh check robots` | 预检 OS/磁盘/架构 |
| `./deploy.sh restart robots` | 重启服务 |

`autonomy_deploy_mode`：`build`（现场编译）| `artifact`（分发 tar.gz）

## 车队部署

```bash
cd ansible

# 1. ARM 编译机 build + 打包
./deploy.sh build robots -l robot01
../scripts/package_autonomy_artifact.sh --output ../dist/autonomy.tar.gz

# 2. 推到全部机器人
./deploy.sh deploy robots \
  -e autonomy_artifact_path=$PWD/../dist/autonomy.tar.gz \
  -f 4

# 3. 只改 config/
./deploy.sh push robots
```

编辑 `inventory/robots/hosts.yml` 填入 IP，配置 SSH 免密后执行。

## Inventory

| 参数 | 文件 | 用途 |
|------|------|------|
| （默认） | `inventory/hosts.yml` | 本机 |
| `robots` | `inventory/robots/hosts.yml` | 车队（artifact） |
| `staging` | `inventory/staging/hosts.yml` | 实验室（build） |
| `production` | `inventory/production/hosts.yml` | 生产 |

环境变量覆盖：`AUTONOMY_DEPLOY_ENV=robots ./deploy.sh`

## 标签

```bash
./deploy.sh --tags config
./deploy.sh --tags build,deps
```

| 标签 | 内容 |
|------|------|
| `build` | source + build（含 ccache） |
| `deps` | dependencies |
| `artifact` | 制品解压 |
| `config` | `/etc/autonomy` |
| `services` | systemd |
| `preflight` | 预检 |

## 本机开发

`inventory/host_vars/localhost.yml` 已设 `autonomy_use_local_source: true`。

```bash
./deploy.sh build
./deploy.sh          # 全量
```

## 依赖安装

Ansible 不重复安装逻辑，调用 `scripts/install_deps`：

```bash
cd ../scripts && python3 -m install_deps
./deploy.sh build    # build 模式自动调用 dependencies 角色
```

机器人 artifact 部署默认 `autonomy_install_dependencies: false`。

## 常用变量

| 变量 | 说明 |
|------|------|
| `autonomy_artifact_path` | 控制机本地 `.tar.gz` 路径 |
| `autonomy_services` | 启用的服务列表 |
| `autonomy_install_dependencies` | 是否装系统依赖 |
| `autonomy_run_verify` | 部署后检查 `systemctl is-active` |

## 目录结构

```text
ansible/
├── deploy.sh              # 入口
├── playbooks/
│   ├── site.yml           # 唯一 playbook
│   └── includes/vault.yml
├── inventory/
│   ├── hosts.yml          # localhost
│   ├── robots/            # 车队
│   ├── staging/
│   └── production/
└── roles/
    ├── preflight/
    ├── common/
    ├── source/
    ├── dependencies/
    ├── build/             # 含 ccache
    ├── artifact/
    ├── config/
    └── services/          # 含 verify
```

## 排错

```bash
./deploy.sh -vvv
systemctl status autonomy-localization autonomy-task autonomy-bridge
journalctl -u autonomy-bridge -n 30 --no-pager
```

## 测试

```bash
./scripts/ci.sh
./scripts/molecule.sh
```
