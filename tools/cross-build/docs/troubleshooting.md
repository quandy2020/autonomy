# 故障处理

本页是后续运行时的排查指南；本轮尚未进行实际验证。

| 现象 | 处理 |
| --- | --- |
| profile.yaml JSON 读取失败 | 使用双引号和 JSON 对象语法；不支持普通 YAML 的注释和缩进格式。 |
| Profile/platform/toolchain 不一致 | 按 profiles.md 同步结构化字段、目录名及四行 CMake 入口。不要靠改一个目录名完成平台迁移。 |
| SSH 失败或 rsync exit 23 | 确认远端账号权限、rsync 安装及开发文件存在；按退出码和文件错误处理，不忽略失败继续构建。 |
| macOS 系统 rsync 不支持 --protect-args | 安装现代 rsync 并通过 PATH 选择；不通过删除参数规避远端路径保护。 |
| 缺少 stdio.h / crt1.o / libc / libstdc++ | 在板端安装匹配固件的开发包再重新采集；不要填入宿主 macOS 或 x86 库。 |
| wrong ELF / hard-float ABI required | 检查目标架构和固件 ABI；aarch64、armhf、soft-float、musl 不是同一个目标。 |
| OS mismatch / requires R36 | 选择正确的 Profile；JetPack 5/7 不能套用 JetPack 6。需为其他环境单独建配置。 |
| symlink escapes sysroot | 不要运行不受限制的 ln/cp 修补到宿主文件；修正板端布局或采集范围。绝对链接只在暂存采集时转换。 |
| SDK 路径有断链 | 使用 --extra-dir 同时采集链接目标；verify 仅强制要求核心文件完整，不代表 SDK 依赖齐全。 |
| Docker 无法连接 | 启动 Docker Desktop/daemon；这与目标 ARM ABI 无关。 |
| 缺少目标 gcc/g++ | 运行仓库的 docker/build_docker_x86_64.py 更新共享镜像，或通过 IMAGE 指定带目标编译器的镜像。 |
| exec format error | 区分 builder 镜像架构与目标 ELF 架构；Apple Silicon 的 amd64 builder 需要仿真，目标产物应在目标板执行。 |
| CMake 试图运行目标程序 | 为交叉构建显式提供项目需要的探测结果/宿主生成工具；不要把所有链接检查改成 static library。 |
| 源码目录 permission denied | 源码只读挂载；项目需支持 out-of-source build。构建和安装只写 /build、/install。 |
| 切换 SOURCE 后复用旧缓存或产物 | 同一 Profile 在容器内总是使用 /src。切换工程前移开 work/build/<profile> 和 work/install/<profile>，然后重新构建。 |
| GLIBCXX / GLIBC version not found | 目标运行时比编译器/依赖要求旧；用匹配 SDK/工具链重建，检查符号版本，不能仅看 ELF 架构。 |
| 找不到业务依赖 | 补齐 sysroot 内的目标头文件、库和 CMake/pkg-config 配置；基础镜像不预装 Autonomy 全部依赖。 |
| nvcc/protoc 无法执行 | 从板端复制的是 ARM 工具；builder 需要自己架构的宿主工具。 |

## 锁与采集恢复

同一 Profile 发生并发操作会报错并给出 `work/.<profile>.lock` 路径。确认没有相关进程或其他终端操作后，才可手动清理遗留锁。锁内 owner 文件仅记录 PID，同一 PID 可能被复用，不能单凭进程号判断。

发布 sysroot 时，先把已有目录重命名为 `.previous-<uuid>`，再发布暂存目录；第二步失败则尝试恢复。异常退出后，先检查目标目录和备份的存在状态，再决定恢复哪个版本。备份、暂存目录和锁不自动跨崩溃恢复。不要在构建进行时清理任何一个目录。

每个 Profile 的旧备份由用户管理；清理前先确认业务产物不再需要该 sysroot。工作区目录不纳入 git，不要在未确认前执行 `git clean -fdx`。

## 可重复构建范围

默认镜像使用发行版 tag 和在线 apt，包补丁版本可能变化；基础实现不提供软件供应链锁文件或二进制级可重复性。正式发布应准备固定摘要的 builder 镜像及 sysroot 快照，记录编译器版本和具体 SDK/L4T 信息。更换 sysroot 或镜像后不要复用旧 CMake 缓存。
