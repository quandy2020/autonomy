# 一条命令部署

在宿主机的 `ansible/` 目录执行：

```bash
./deploy.sh release robots -l robot01 -K
```

自动完成：准备控制机工具 → SpaceHero 编译 → 打包与校验 → SSH 部署 → 更新配置 → 重启并检查服务。任何一步失败都会停止。`-K` 提示输入目标机 sudo 密码；已有免密 sudo 时可省略。

## 首次使用只改配置

1. [inventory/robots/hosts.yml](../inventory/robots/hosts.yml)：填写 `robot01` 的真实 IP 和 SSH 用户。
2. [vars/release.yml](../vars/release.yml)：填写编译容器、源码/构建目录和构建参数，默认使用现有 SpaceHero。
3. [inventory/robots/group_vars/all.yml](../inventory/robots/group_vars/all.yml)：配置目标服务和部署设置；单机差异放在 `inventory/robots/host_vars/robot01.yml`。

宿主机需要 Python 3.12（含 venv）、Docker 和运行中的编译容器；首次准备工具需要联网。目标机须能通过可信 SSH 登录、有系统 Python 3、sudo 和匹配的运行依赖。完整部署会覆盖同名配置并重启服务；同一编译容器不要同时运行多个 release。

## 换场景只换配置或参数

```bash
# 临时只部署 task 服务
./deploy.sh release robots -l robot01 -K -e '{"autonomy_services": ["task"]}'

# 使用另一套编译配置（先复制 release.yml 并修改）
./deploy.sh release robots -l robot01 -K --release-config vars/release-lab.yml

# 已有制品：在 robots 配置中填写 autonomy_artifact_path，直接部署
./deploy.sh deploy robots -l robot01 -K

# 宿主机用户目录安装到 /home/quandy/workspace/demo（无需 sudo）
./deploy.sh release demo --release-config vars/release-demo.yml

# 在目标机直接编译并部署：地址与参数放在 staging inventory
./deploy.sh full staging -l dev01 -K

# 只更新配置 / 只重启
./deploy.sh push robots -l robot01 -K
./deploy.sh restart robots -l robot01 -K
```

`release.yml` 中的 `artifact_path` 相对 `ansible/` 解析；构建参数是 YAML 字符串列表。`--release-config` 只影响控制机编译，目标配置仍由所选 inventory 管理。临时覆盖目标参数使用 `-e`，长期设置写入 inventory。

`failed=0`、`unreachable=0` 表示部署完成，服务 active 仍需结合机器人实际功能验收。`release` 不代替生产审批，production 使用 [CI/CD](ci-cd.md)。出错看 [排障](troubleshooting.md)，日常维护看 [部署后的操作](deployment.md)。

`demo` 配置使用当前用户安装，跳过系统用户和 systemd 管理；安装入口为 `/home/quandy/workspace/demo`，完整版本在 `demo-releases/`，运行配置在 `demo.config/`。`autonomy_system_install: false` 要求关闭依赖安装，适用于已准备运行依赖的宿主机。

`release-demo.yml` 的 `bundle_runtime: true` 会捆绑可执行文件链接的第三方动态库；目标仍需兼容的 libc/C++ 运行时与 GPU 驱动，动态加载的可选插件需单独验证。普通 release 默认使用目标平台已有的依赖基线。
