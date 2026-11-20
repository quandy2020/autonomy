# 一条命令部署

在宿主机的 `ansible/` 目录执行：

```bash
./deploy.sh release robots -l robot01 -K
```

自动完成：准备控制机工具 → SpaceHero 编译 → 打包与校验 → SSH 部署 → 更新配置 → 重启并检查服务。任何一步失败都会停止。`-K` 提示输入目标机 sudo 密码；已有免密 sudo 时可省略。

向 ARM 板卡发布请使用 [一键交叉编译与分发](cross-release.md)。

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

制品默认保存在控制机的 `~/.autonomy/artifacts/`，验证日志在 `~/.autonomy/log/`，不在源码目录生成 `dist`。`release.yml` 的 `artifact_path` 支持 `~` 和绝对路径；相对路径以 `ansible/` 为基准。构建参数是 YAML 字符串列表。`--release-config` 只影响控制机编译，目标配置仍由所选 inventory 管理。临时覆盖目标参数使用 `-e`，长期设置写入 inventory。

`failed=0`、`unreachable=0` 表示部署完成，服务 active 仍需结合机器人实际功能验收。`release` 不代替生产审批，production 使用 [CI/CD](ci-cd.md)。出错看 [排障](troubleshooting.md)，日常维护看 [部署后的操作](deployment.md)。

`demo` 配置使用当前用户安装，跳过系统用户和 systemd 管理；安装入口为 `/home/quandy/workspace/demo`，完整版本在 `demo-releases/`，运行配置在 `demo.config/`。`autonomy_system_install: false` 要求关闭依赖安装，适用于已准备运行依赖的宿主机。

`bundle_runtime: true` 会捆绑可执行文件链接的第三方动态库，以及 Autoviz 的 Qt/Ogre 插件依赖；编译机需要 `patchelf`、Qt 6 的 `qmake6` 和插件。`bin/` 中直接安装真实程序，通过 ELF 的 `$ORIGIN` 相对路径加载依赖，程序从自身安装位置查找资源，不生成隐藏 `.elf` 或启动包装脚本。安装后可直接执行 `./bin/autoviz`，无需先设置 `LD_LIBRARY_PATH`。目标仍需兼容的 libc/C++ 运行时与 GPU 驱动，其他可选插件需单独验证。

运行库打包时会按内容去重：完全相同的动态库用硬链接共享磁盘数据，所有加载路径保留。`.so → .so.0 → .so.0.x` 是版本软链接，只存一份库数据；OpenCV、PCL 等名称相近的模块是不同依赖，不能按名字删除。去重节省的字节数记录在 `MANIFEST.json` 的 `deduplicated_runtime_bytes` 中。

## 安装后测试能否运行

在 `ansible/` 目录执行：

```bash
python3 scripts/smoke_runtime.py --prefix /home/quandy/workspace/demo --config-dir /home/quandy/workspace/demo.config

# 本机有 X11 桌面时，额外验证 Autoviz 窗口、渲染初始化和自动退出
python3 scripts/smoke_runtime.py --prefix /home/quandy/workspace/demo --config-dir /home/quandy/workspace/demo.config --gui
```

自动验证 Autoviz 无额外库路径配置启动、task/localization/bridge 启动、RPC 心跳/身份/状态和退出；任何失败返回非零。日志目录会打印在首行。测试使用独立网络、IPC、共享内存和临时目录，不发送运动指令；需要宿主机允许非特权用户命名空间并安装 `unshare`、`mount`、`ip`。窗口检查使用空通信环境；实时话题显示、传感器输入、实际定位精度和机器人导航需单独验收。

已知限制：Autoviz 与 task/localization 联跑时，自动退出测试曾超时；这项联合运行检查尚未通过。
