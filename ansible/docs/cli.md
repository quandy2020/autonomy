# deploy.sh 常用命令行参数

在 `ansible/` 目录执行以下命令。`robot01` 和 `rk3588_robots` 是主机和组的示例，使用前换成 inventory 中的名称。归档路径使用控制机上的绝对路径。

```bash
./deploy.sh [模式] [环境] [参数]
./deploy.sh --help
./deploy.sh --help --color=always
```

帮助按场景列出示例。终端里的标题用青色、命令用绿色，管道或重定向默认使用纯文本。

省略模式时使用 `full`，按 inventory 执行源码构建或制品部署。可用环境包括 `robots`、`staging`、`production`、`sim` 和 `demo`。未指定环境时，`deploy`、`release`、`cross-release` 默认选 `robots`，其他 Ansible 模式默认选本机。也可设置 `AUTONOMY_DEPLOY_ENV`，位置参数优先。生产制品发布走 [CI 审批和晋级](ci-cd.md#推广到生产或回滚)。

## 参数可以和哪些模式搭配

| 参数 | 可搭配的模式 | 用法 |
| --- | --- | --- |
| `-l PATTERN`、`--limit PATTERN` | 使用 inventory 的模式 | 选主机或组。`cross-release` 实际发布时必填。 |
| `-K`、`--ask-become-pass` | 使用 inventory 的模式 | 提示输入目标机 sudo 密码。模块 `plan` 在系统安装模式下需要 sudo 读取健康检查点。 |
| `-R FILE`、`--release-config FILE` | `release`、`cross-build`、`cross-release`、`plan` | 指定控制机构建配置。默认原生配置为 `vars/release.yml`，交叉配置为 `vars/release-cross.yml`。 |
| `-I`、`--incremental` | `build`、`cross-build`、`release`、`cross-release`、`deploy` | 交叉模式复用构建缓存。`deploy`、`release`、`cross-release` 尝试差分传输，必要时仍发送完整制品。原生 `build` 默认复用构建目录。 |
| `-m NAME`、`--module NAME` | `build`、`cross-build`、`cross-release`、`plan` | 可重复。选一级项目或 `autonomy/<领域>`，CMake 自动加入依赖。原生 `build` 加 `-m` 只编译，`cross-build` 同时提供 `-m` 和 `-B` 才打包模块更新。 |
| `-B FILE`、`--base-artifact FILE` | `cross-build`、`cross-release`、`plan` | 与至少一个 `--module` 同用。模块 `cross-release` 和模块 `plan` 必须提供基线归档。 |
| `-S`、`--test` | `build` | 编译后、完整构建安装前运行当前 CMake 配置注册的 CTest。测试失败或没有注册测试时停止。可与模块选择同用。 |
| `-o FILE`、`--report FILE` | `style` | 把完整格式检查报告写入指定文件。 |
| `-L MODE`、`--color MODE` | 所有模式 | `auto` 在终端显示颜色，`always` 强制开启，`never` 关闭。默认 `auto`。 |
| `-e VARS`、`--extra-vars VARS` | 使用 inventory 的模式 | 临时覆盖变量。标量用 `KEY=VALUE`，列表等结构化值用 JSON。 |
| `-C`、`--check` | Ansible 模式 | 预览支持检查模式的任务。交叉发布的检查模式使用 `deploy --check` 加已有制品。 |
| `-D`、`--diff` | 使用 inventory 的模式 | 显示文件差异。常与 `-C` 一起预览配置变更。 |
| `--list-hosts`、`--list-tasks`、`--list-tags`、`--syntax-check` | 使用 inventory 的模式 | 列出目标、任务、标签或检查语法。列表和语法检查会跳过 release 的编译准备。 |
| `--private-key FILE`、`-u USER` / `--user USER`、`-v` / `--verbose` | 使用 inventory 的模式 | 指定 SSH 私钥、临时指定 SSH 用户或提高日志详细度。 |

环境名放在模式后，通过 `-l` 缩小目标。`build`、`deploy`、`release`、`cross-release`、`plan`、`verify`、`push` 和 `restart` 按完整流程执行，禁用 `--tags`、`--skip-tags`、`--start-at-task` 等任务筛选。`cross-build` 在控制机执行，支持构建和颜色参数。`style` 接收仓库相对路径和报告参数。

`check` 执行基础目标机检查。`plan` 还检查构建或发布目录所在文件系统的空间，加 `-R` 可核对原生构建 ABI 或交叉 Profile。`cross-release` 要求所选目标声明同一个交叉 Profile，并与发布配置一致。

两种写法可以混用。`--release-config`、`--module` 和 `--base-artifact` 还支持 `--名称=值`。带值的短选项用空格分隔。

```bash
./deploy.sh build -m autonomy/control -S
./deploy.sh build --module=autonomy/control --test
```

阶段提示用青色，错误用红色，格式检查警告用黄色、通过用绿色。Ansible 任务沿用自身的状态颜色。`auto` 在管道和文件输出中保持纯文本，并遵循 `NO_COLOR`。显式 `always` 或 `never` 优先，`style` 的报告文件保持纯文本。

```bash
./deploy.sh plan robots -l robot01 -L always
./deploy.sh style autonomy/control --color=never -o /tmp/autonomy-style.log
```

## 先查再执行

```bash
# 列出单机或组，确认范围
./deploy.sh deploy robots -l robot01 --list-hosts
./deploy.sh cross-release staging -l rk3588_robots --list-hosts

# 检查 playbook 语法
./deploy.sh deploy staging --syntax-check

# 常规预检、原生 ABI 预检、交叉 Profile 预检
./deploy.sh check robots -l robot01
./deploy.sh plan robots -l robot01 --release-config vars/release.yml
./deploy.sh plan robots -l robot01 --release-config vars/release-cross.yml

# 临时指定连接凭据
./deploy.sh check staging --limit robot01 -u deployer --private-key "$HOME/.ssh/id_ed25519"
```

## 本机与远程开发

```bash
# 检查全仓格式，或检查模块并输出完整报告
./deploy.sh style
./deploy.sh style autonomy/control --report /tmp/autonomy-style.log

# 本机增量编译、测试并安装到独立开发路径
./deploy.sh build -K --test \
  -e autonomy_install_prefix=/opt/autonomy-dev -e autonomy_cmake_jobs=4

# 只编译指定模块。使用该构建目录的产物进行开发
./deploy.sh build --module autodriver --module autonomy/transform

# 在已准备依赖的 staging 主机编译和测试指定模块
./deploy.sh build staging -l robot01 -K --module autonomy/control --test \
  -e autonomy_install_dependencies=false -e autonomy_cmake_jobs=4

# 在 staging 现场编译、安装并配置服务
./deploy.sh full staging -l robot01 -K
```

远程 `build` 使用目标机检出的源码。一级项目可选 `automsgs`、`autolink`、`autodriver`、`autoviz`、`automanip` 和 `autonomy`。需要运行特定模块的 CTest 时，该模块及其依赖需在当前配置中注册测试。 [构建指南](getting-started.md)

## 编译、打包和发布

```bash
# 用现有原生 Docker 容器编译并发布
./deploy.sh release robots -l robot01 -K --incremental \
  --release-config vars/release.yml

# 单次原生发布只启用 task 服务
./deploy.sh release robots -l robot01 -K -e '{"autonomy_services":["task"]}'

# 安装到 demo 用户目录
./deploy.sh release demo --release-config vars/release-demo.yml

# 生成完整 ARM 制品，或只编译指定模块
./deploy.sh cross-build --incremental --release-config vars/release-cross.yml
./deploy.sh cross-build --incremental --module automsgs --module autonomy/control

# 一次交叉编译，增量发布到同 Profile 的目标组
./deploy.sh cross-release staging -l rk3588_robots -K --incremental \
  --release-config vars/release-cross.yml
```

## 模块增量发布

先用已验收归档做基线。模块构建和基线使用相同的 Profile、sysroot 和镜像，目标 inventory 需配置健康探针。`cross-release` 自动执行同样的预检，再开始编译。

```bash
base=/absolute/path/to/current-autonomy.tar.gz

./deploy.sh plan robots -l robot01 -K \
  --base-artifact "$base" --module automsgs --module autonomy/control

./deploy.sh cross-release robots -l robot01 -K \
  --base-artifact "$base" --module automsgs --module autonomy/control

# 只生成合并后的制品，之后用 deploy 分发
./deploy.sh cross-build --base-artifact "$base" \
  --module automsgs --module autonomy/control
```

指定基线时自动启用增量。后续模块发布可以使用这次验收通过的归档作为新基线。 [交叉编译与模块发布](cross-release.md#增量部署指定模块)

## 已有制品、配置更新和验证

```bash
# 从 cross-build 的输出中取归档路径，读取配套校验值
archive=/absolute/path/to/autonomy.tar.gz
sha=$(awk 'NR == 1 {print $1}' "$archive.sha256")

# 先用已有归档模拟部署任务，再执行增量分发
./deploy.sh deploy robots -l robot01 -K --check \
  -e "autonomy_artifact_path=$archive" -e "autonomy_artifact_sha256=$sha"

./deploy.sh deploy robots -l robot01 -K --incremental \
  -e "autonomy_artifact_path=$archive" -e "autonomy_artifact_sha256=$sha"

# 预览配置差异，再推送到同一台机器
./deploy.sh push robots -l robot01 -K --check --diff
./deploy.sh push robots -l robot01 -K

./deploy.sh restart robots -l robot01 -K
./deploy.sh verify robots -l robot01 -e "autonomy_artifact_sha256=$sha"
./deploy.sh verify robots -l robot01 -K -e autonomy_show_release_retention=true
./deploy.sh verify production --limit robot01
```

`push` 在配置有变更且允许自动重启时重启所选 systemd 服务。命令完成后查看 Ansible 汇总中的 `failed` 和 `unreachable`。配置差异可能包含现场配置值，在可信终端查看。`autonomy_show_release_retention=true` 只列候选和保护版本。回滚时用已验收的旧完整制品和对应 SHA-256 执行 `deploy`，再执行 `verify`。 [日常操作与回滚](deployment.md)
