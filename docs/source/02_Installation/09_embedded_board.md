(installation-embedded-board)=
# 9. 嵌入式板端构建与安装

面向 **Firefly / RK3588 等 aarch64 板子**：开发机改代码，经 **NFS** 同步源码，在板子 **本地盘** 编译与运行。  
详细脚本参数亦可参考仓库内 [`scripts/README.md`](../../../scripts/README.md)。

### 9.1 适用场景与拓扑

| 角色 | 典型地址 | 职责 |
|------|----------|------|
| 开发机（x86） | `192.168.234.50`（可自动探测） | NFS Server，编辑源码 |
| 嵌入式板 | `firefly@192.168.234.1` 等 | NFS Client，本机 `build` |

```
[开发机]  autonomy 仓库根 (EXPORT_PATH)
    │  nfs-kernel-server
    ▼
[板子]  ~/autonomy              ← NFS 源码（勿在此目录 cmake build）
        ~/autonomy_ws/src/autonomy → 软链到 NFS
        ~/autonomy_ws/build        ← 本地盘产物（推荐）
        /usr/local                 ← 第三方库（install_dependency）
```

**原则**：源码走 NFS；`build` / `install` 放板子本地路径，避免 NFS 编译慢与文件锁问题。

### 9.2 推荐流程总览

```text
开发机: SSH 免密 +（可选）NAT
    → share_nfs_workspace.sh all --board-ip <板IP>
板子:  install_dependencies.py --profile board
    → cmake -S src/autonomy -B ~/autonomy_ws/build …
    → cmake --build / source setup_environment.bash
```

---

### 9.3 开发机：一键 NFS

在 **开发机**、autonomy 仓库根执行：

```bash
cd /path/to/autonomy

# 按板子 IP（最常用）
bash scripts/share_nfs_workspace.sh all --board-ip 192.168.234.1

# 或使用配置档 scripts/nfs_boards/<name>.env
bash scripts/share_nfs_workspace.sh all --board-profile lab-default

bash scripts/share_nfs_workspace.sh status --board-ip 192.168.234.1
```

#### 参数如何按板子动态加载

优先级：**命令行 > 环境变量 > `--board-profile` > 内置默认**。

| 参数 | 环境变量 | 含义 |
|------|----------|------|
| `--board-ip IP` | `BOARD_IP` | 板子 IP（合成 `FIREFLY_HOST=user@IP`） |
| `--board-user USER` | `BOARD_USER` | SSH 用户（默认 `firefly`） |
| `--board-host USER@IP` | `FIREFLY_HOST` | 完整 SSH 目标 |
| `--host-ip IP` | `HOST_IP` | 开发机对板子可见的 NFS 地址 |
| `--client-net CIDR` | `CLIENT_NET` | `/etc/exports` 允许网段 |
| `--mount PATH` | `MOUNT_POINT` | 板端挂载点（默认 `/home/firefly/autonomy`） |
| `--ws PATH` | `WS_LOCAL` | 板端工作区（默认 `/home/firefly/autonomy_ws`） |
| `--board-profile NAME` | — | 加载 `scripts/nfs_boards/NAME.env` |

示例（另一块板 / 另一网段）：

```bash
bash scripts/share_nfs_workspace.sh all \
  --board-ip 192.168.234.20 \
  --board-user firefly \
  --host-ip 192.168.234.50

# 非 234 网段
bash scripts/share_nfs_workspace.sh all \
  --board-host firefly@10.0.0.5 \
  --host-ip 10.0.0.1 \
  --client-net 10.0.0.0/24
```

新建板配置档：

```bash
cp scripts/nfs_boards/lab-default.env scripts/nfs_boards/my-rk3588.env
# 编辑 BOARD_IP / BOARD_USER …
bash scripts/share_nfs_workspace.sh all --board-profile my-rk3588
```

#### 前置：SSH 与出网（装依赖需要）

```bash
# 开发机 → 板子免密
ssh-copy-id -i ~/.ssh/id_ed25519.pub firefly@192.168.234.1

# 开发机开启转发/NAT（换电脑时做一次）
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING \
  -s 192.168.234.0/24 ! -d 192.168.234.0/24 -j MASQUERADE

# 板子默认路由走开发机（HOST_IP）
ssh firefly@192.168.234.1 \
  'sudo ip route replace default via 192.168.234.50'
```

成功标志：板子上 `df -h ~/autonomy` 为 NFS；`~/autonomy_ws/src/autonomy` 指向该挂载。

子命令：`all` / `server` / `client` / `status` / `down`（`status`/`down` 也需带 `--board-ip`）。

---

### 9.4 板端：安装依赖

SSH 登录板子后：

```bash
# 确认出网（apt / git clone）
ping -c1 8.8.8.8

cd ~/autonomy    # NFS 源码
python3 scripts/install_dependencies.py --profile board --skip-installed
```

| 要点 | 说明 |
|------|------|
| `--profile board` | 对齐 `autonomy.aarch64.dockerfile` + CMake；无 Qt/Ogre/Sphinx |
| 第三方 | 强制走 `docker/install/*.sh` → **`/usr/local`**（glog 0.6、protobuf 3.19、ceres…） |
| 冲突 apt | 会 purge apt 版 glog/gflags/ceres/protobuf/grpc，避免 ABI 混用 |
| 缓存目录 | 优先 `~/.cache/autonomy/thirdparty`（避免 root 占用 `/thirdparty`） |
| 续装 | `--thirdparty-only --resume-from install_protobuf.sh` |
| 权限 | `make install` / `ldconfig` 需板子 `sudo` |

查看计划：

```bash
python3 scripts/install_dependencies.py --list --profile board
```

---

### 9.5 板端：CMake 构建

**务必**在本地工作区构建：

```bash
cd ~/autonomy_ws

cmake -S src/autonomy -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/usr/local \
  -DBUILD_AUTOVIZ=OFF \
  -DBUILD_ORBISVIEW=OFF \
  -DBUILD_DOCS=OFF \
  -DBUILD_TEST=OFF \
  -DBUILD_ONNXRUNTIME=OFF \
  -DAUTONOMY_BUILD_MANIPULATION=OFF

cmake --build build -j$(nproc)
# 内存紧张时：cmake --build build -j4
```

| 路径 | 说明 |
|------|------|
| `~/autonomy_ws/build/lib/` | `libautonomy.so`、`libautolink.so` 等 |
| `~/autonomy_ws/build/bin/` | `mainboard`、`autolink` 等可执行文件 |

开发机改代码后 NFS 即时可见，板子只需再次 `cmake --build build`。

> **不要**使用 `~/autonomy/build`（NFS 上）作为主构建目录。

---

### 9.6 板端：运行环境与可选 install

```bash
cd ~/autonomy_ws
export AUTONOMY_BUILD_DIR=$PWD/build
source src/autonomy/scripts/setup_environment.bash

# 验证
ls build/lib/libautonomy.so build/bin/mainboard 2>/dev/null
# 运行时优先 /usr/local 动态库
# echo /usr/local/lib | sudo tee /etc/ld.so.conf.d/usr-local.conf && sudo ldconfig
```

若项目配置了 `install` 规则且需要装到前缀：

```bash
cmake --install build --prefix $HOME/autonomy_ws/install
# 或
cmake --build build --target install
```

日常开发一般直接使用 `build/lib` + `build/bin`，无需系统级 install。

安装到 `/usr/local` 后推荐：

```bash
sudo cmake --install build    # 在 build 目录内也可：sudo make install
source /usr/local/share/autonomy/setup.bash
autolink launch start task.launch
```

板端 `task.launch` 的 **CPU / 内存 / 启动时延** 实测见运行指南：[板端 task.launch 资源报告](../04_Running/09_board_task_launch_benchmark.md)。

---

### 9.7 切换板子 / 换开发机

```bash
# 开发机：卸旧板、挂新板
bash scripts/share_nfs_workspace.sh down --board-ip 192.168.234.1
bash scripts/share_nfs_workspace.sh all  --board-ip 192.168.234.20
```

换开发机后，若板子仍挂旧 IP：

```bash
ssh firefly@<板IP>
sudo umount ~/autonomy || sudo umount -l ~/autonomy
sudo sed -i '\#/home/firefly/autonomy#d' /etc/fstab
```

然后在新电脑仓库根重新 `share_nfs_workspace.sh all --board-ip …`。

---

### 9.8 板端常见问题

| 现象 | 处理 |
|------|------|
| `Temporary failure resolving` / apt 找不到包 | 板子无默认路由；经开发机 NAT + `nameserver 8.8.8.8` |
| `ldconfig: Permission denied` | 使用已修复的 `docker/install`（`autonomy_ldconfig` + sudo） |
| git `dubious ownership` under `/thirdparty` | 使用用户缓存目录，或 `chown` / `safe.directory` |
| 链接找不到 glog 符号 / `libglog.so.0` | 勿用 apt glog；确认 `/usr/local/lib/libglog.so`（0.6）与 `CMAKE_PREFIX_PATH=/usr/local` |
| 编译极慢 | 确认 `-B ~/autonomy_ws/build`，不是 NFS 路径 |
| OOM | `-j2` / `-j4` |

更多通用排错见 [§8 故障排查](08_troubleshooting.md)。

### 9.9 相关文档

- [§4 依赖安装](04_dependencies.md)
- [§6 编译构建](06_build.md)（桌面/CI 通用流程）
- [§7 环境配置](07_environment.md)
- 仓库 [`scripts/README.md`](../../../scripts/README.md)（NFS 参数全文）
