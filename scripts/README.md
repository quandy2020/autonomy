# scripts/

开发与 CI 用工具脚本。依赖安装入口为 **`install_deps`** Python 包。

## 依赖安装

| 层 | 路径 | 用途 |
|----|------|------|
| CLI | `install_deps/` | `python3 -m install_deps` |
| 脚本 | `../docker/install/*.sh` | 第三方库编译安装 |
| 部署 | `../ansible/roles/dependencies/` | Ansible 调用同一模块 |

```bash
cd scripts
python3 -m install_deps
python3 -m install_deps --thirdparty-only --skip-installed
python3 -m install_deps --list-apt

# Ansible
cd ../ansible && ./deploy.sh build
```

### 包结构

```text
install_deps/
├── data/
│   ├── apt_packages.json
│   ├── thirdparty.json
│   └── settings.json
├── config.py
├── detector.py
├── apt.py
├── thirdparty.py
└── app.py
```

修改依赖：APT 列表 → `data/apt_packages.json`；脚本顺序与检测规则 → `data/thirdparty.json`；安装实现 → `docker/install/`。
