# scripts/

开发与 CI **工作流入口**（环境 / 依赖 / NFS）。

通用小工具（格式化、打包、板端 Swap、CMake 检查）已迁到 [`tools/`](../tools/)。

| 路径 | 用途 |
|------|------|
| **`setup_environment.bash`** | **统一环境变量**；`make install` 后为 `$PREFIX/share/autonomy/setup.bash` |
| **`install_dependencies.py`** | **一键依赖安装** |
| `install_deps/` | 模块化实现（`python3 -m install_deps`） |
| **`share_nfs_workspace.sh`** | **一键 NFS + 板端源码挂载**（多板 IP 可参数化） |
| `nfs_boards/*.env` | 板子配置档（按名字加载） |

```bash
source scripts/setup_environment.bash
python3 scripts/install_dependencies.py --skip-installed
python3 scripts/install_dependencies.py --profile board --skip-installed
python3 tools/clang_format_sources.py --check
```

依赖数据：`install_deps/data/*.json`；第三方安装脚本：`docker/install/`。

---

## 指定某个库 / 包安装

第三方库在 `docker/install/install_*.sh`，由 `install_dependencies.py` 按 Dockerfile 顺序调用。  
**已装到 `/usr/local` 的库，带 `--skip-installed` 时会跳过，不会再 clone。**

### 查看列表

```bash
python3 scripts/install_dependencies.py --list --profile board
# 输出 apt 包 + thirdparty 脚本名（如 install_ceres_solver.sh）
```

### 只装某一个第三方库（推荐）

直接跑对应脚本（最快，不跑整条依赖链）：

```bash
cd /path/to/autonomy   # 或板子上 ~/autonomy

# 示例：只装 Ceres / glog / protobuf
bash docker/install/install_ceres_solver.sh
bash docker/install/install_glog.sh
bash docker/install/install_protobuf.sh

# 板子内存紧时限制并行（Ceres 易 OOM）
AUTONOMY_MAKE_JOBS=2 bash docker/install/install_ceres_solver.sh
```

脚本若检测到 `/usr/local` 已有产物会直接 `[OK] … skipping`。

### 从某个库开始续装（后面的也会装）

```bash
# 从 Ceres 起装到 board 列表末尾（跳过已装）
python3 scripts/install_dependencies.py --profile board \
  --thirdparty-only --skip-installed \
  --resume-from install_ceres_solver.sh
```

| 参数 | 作用 |
|------|------|
| `--thirdparty-only` | 不跑 apt |
| `--resume-from install_XXX.sh` | 从该脚本起往后装 |
| `--skip-installed` | `/usr/local`（等）已有则跳过，**不 clone** |
| `--force-thirdparty` | 强制全部重编（忽略 skip） |

### 脚本名 ↔ 库

| 脚本 | 库 |
|------|-----|
| `install_gtest.sh` | GoogleTest |
| `install_glog.sh` | glog 0.6 |
| `install_gflags.sh` | gflags |
| `install_protobuf.sh` | Protobuf 3.19 |
| `install_grpc.sh` | gRPC |
| `install_ceres_solver.sh` | Ceres |
| `install_opencv.sh` | OpenCV（板端可跳过，用 apt） |
| `install_osqp.sh` | OSQP |
| `install_g2o.sh` | g2o |
| `install_fbow.sh` | FBoW |
| `install_nlohmann.sh` | nlohmann/json |
| `install_behaviortree_cpp.sh` | BehaviorTree.CPP |
| `install_gperftools.sh` | tcmalloc（系统有则可跳过） |
| `install_adolc.sh` / `install_ipopt.sh` | ADOL-C / Ipopt（多为 apt） |
| `install_fastdds.sh` | Fast DDS **v3.6.2**（可选跨机 RTPS，SECURITY=ON） |

### 只要某个 apt 包

```bash
sudo apt-get install -y libeigen3-dev   # 示例
# 或只跑依赖脚本里的 apt 段：
python3 scripts/install_dependencies.py --profile board --apt-only
```

---

## 一键 NFS + 板端 Build（完整流程）

目标：开发机改代码 → NFS 实时同步到板子 → 板子本地盘编译（`autonomy_ws/build`）。

### 1. 参数怎么传（不同板子 IP）

脚本支持三种等价方式，**优先级：命令行 > 当前 shell 环境变量 > `--board-profile` 文件 > 内置默认**。

#### 方式 A：命令行参数（推荐，临时切板）

```bash
cd /path/to/autonomy   # 仓库根（含 CMakeLists.txt / scripts/）

# 板子 A（默认实验室）
bash scripts/share_nfs_workspace.sh all --board-ip 192.168.234.1

# 板子 B（另一个 IP）
bash scripts/share_nfs_workspace.sh all --board-ip 192.168.234.20

# 指定用户 + 本机 NFS 地址 + 网段
bash scripts/share_nfs_workspace.sh all \
  --board-ip 10.0.0.5 \
  --board-user firefly \
  --host-ip 10.0.0.1 \
  --client-net 10.0.0.0/24

# 完整 SSH 目标（等价于 user@ip）
bash scripts/share_nfs_workspace.sh all --board-host firefly@192.168.234.20

# 只查状态 / 卸载时也要带同一块板的参数
bash scripts/share_nfs_workspace.sh status --board-ip 192.168.234.20
bash scripts/share_nfs_workspace.sh down   --board-ip 192.168.234.20
```

| 参数 | 对应变量 | 含义 |
|------|----------|------|
| `--board-ip IP` | `BOARD_IP` → 合成 `FIREFLY_HOST` | **板子 IP**（最常用） |
| `--board-user USER` | `BOARD_USER`（默认 `firefly`） | SSH 用户名 |
| `--board-host USER@IP` | `FIREFLY_HOST` | 完整 SSH 目标（覆盖上面两项） |
| `--host-ip IP` | `HOST_IP` | **开发机**在板子眼里的地址（NFS server） |
| `--client-net CIDR` | `CLIENT_NET` | `/etc/exports` 允许的客户端网段 |
| `--export PATH` | `EXPORT_PATH` | 开发机导出目录（默认=仓库根） |
| `--mount PATH` | `MOUNT_POINT` | 板子挂载点（默认 `/home/firefly/autonomy`） |
| `--ws PATH` | `WS_LOCAL` | 板子本地工作区（默认 `/home/firefly/autonomy_ws`） |
| `--ssh-key PATH` | `FIREFLY_SSH_KEY` | 私钥 |
| `--board-profile NAME` | （加载文件） | 见方式 C |

`--board-ip` + `--board-user` 会自动得到 `FIREFLY_HOST=user@ip`，无需改脚本。

#### 方式 B：环境变量（适合写进 `~/.bashrc` 或当前终端）

```bash
export BOARD_IP=192.168.234.20
export BOARD_USER=firefly
# 或直接：
# export FIREFLY_HOST=firefly@192.168.234.20

export HOST_IP=192.168.234.50          # 可选；不设则自动探测本机 234.x
export CLIENT_NET=192.168.234.0/24
export MOUNT_POINT=/home/firefly/autonomy
export WS_LOCAL=/home/firefly/autonomy_ws

bash scripts/share_nfs_workspace.sh all
bash scripts/share_nfs_workspace.sh status
```

#### 方式 C：板子配置档（多板长期使用）

目录：`scripts/nfs_boards/<名字>.env`

已提供示例：

- `scripts/nfs_boards/lab-default.env` → `192.168.234.1`
- `scripts/nfs_boards/lab-board-b.env` → `192.168.234.20`

```bash
# 使用配置档
bash scripts/share_nfs_workspace.sh all --board-profile lab-default
bash scripts/share_nfs_workspace.sh all --board-profile lab-board-b

# 配置档 + 命令行覆盖（命令行优先）
bash scripts/share_nfs_workspace.sh all \
  --board-profile lab-default \
  --board-ip 192.168.234.30
```

新建一块板时：

```bash
cp scripts/nfs_boards/lab-default.env scripts/nfs_boards/my-rk3588.env
# 编辑 my-rk3588.env 里的 BOARD_IP / BOARD_USER / CLIENT_NET …
bash scripts/share_nfs_workspace.sh all --board-profile my-rk3588
```

`*.env` 示例内容：

```bash
BOARD_USER=firefly
BOARD_IP=192.168.234.20
CLIENT_NET=192.168.234.0/24
MOUNT_POINT=/home/firefly/autonomy
WS_LOCAL=/home/firefly/autonomy_ws
# HOST_IP=192.168.234.50          # 可选
# FIREFLY_SSH_KEY=$HOME/.ssh/id_ed25519
```

#### 参数与拓扑对照

```
开发机 (HOST_IP, 自动或 --host-ip)
  EXPORT_PATH ──NFS──► 板子 (BOARD_IP / --board-ip)
                         MOUNT_POINT   = NFS 源码
                         WS_LOCAL/build = 本地编译目录
                         SSH = BOARD_USER@BOARD_IP
```

---

### 2. 从零到编译（推荐步骤）

以下以「板子 IP = `192.168.234.20`」为例；换成你的 IP 即可。

#### 步骤 0：网络与 SSH

```bash
# 开发机：应能 ping 到板子
ping -c1 192.168.234.20

# 免密（每块板做一次）
ssh-copy-id -i ~/.ssh/id_ed25519.pub firefly@192.168.234.20
ssh firefly@192.168.234.20 'echo ok'
```

板子上网（装依赖需要）：默认路由走开发机，开发机需开启转发/NAT（换电脑时再做一次）：

```bash
# 开发机（需 sudo）
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -s 192.168.234.0/24 ! -d 192.168.234.0/24 -j MASQUERADE

# 板子
ssh firefly@192.168.234.20 \
  'sudo ip route replace default via 192.168.234.50'   # via = 你的 HOST_IP
```

#### 步骤 1：一键 NFS

```bash
cd /path/to/autonomy

bash scripts/share_nfs_workspace.sh all --board-ip 192.168.234.20
# 或
bash scripts/share_nfs_workspace.sh all --board-profile lab-board-b

bash scripts/share_nfs_workspace.sh status --board-ip 192.168.234.20
```

成功标志：

- 开发机：`showmount -e <HOST_IP>` 能看到导出路径  
- 板子：`df -h ~/autonomy` 为 NFS；`~/autonomy_ws/src/autonomy` → 该挂载点  

#### 步骤 2：板端装依赖（首次 / 缺库时）

```bash
ssh firefly@192.168.234.20
# 确认出网
ping -c1 8.8.8.8

cd ~/autonomy   # 已是 NFS 上的源码
python3 scripts/install_dependencies.py --profile board --skip-installed
```

说明：`--profile board` 对齐 aarch64 Dockerfile + CMake；glog/protobuf/ceres 等走 `docker/install` → `/usr/local`。

#### 步骤 3：板端 Build（产物在本地盘）

**务必**在 `~/autonomy_ws` 构建，不要用 NFS 上的 `~/autonomy/build`。

```bash
ssh firefly@192.168.234.20
cd ~/autonomy_ws

cmake -S src/autonomy -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/usr/local \
  -DBUILD_AUTOVIZ=OFF \
  -DBUILD_ORBISVIEW=OFF \
  -DBUILD_DOCS=OFF \
  -DBUILD_TEST=OFF \
  -DBUILD_ONNXRUNTIME=OFF

cmake --build build -j$(nproc)
# 二进制一般在 build/bin、库在 build/lib
```

开发机改代码后，板子 NFS 立即可见，直接重新 `cmake --build` 即可。

#### 步骤 4：切换另一块板

```bash
# 先卸当前板（带上旧 IP）
bash scripts/share_nfs_workspace.sh down --board-ip 192.168.234.20

# 挂新板
bash scripts/share_nfs_workspace.sh all --board-ip 192.168.234.30
```

同一开发机可 export 一次，多块板先后 mount；`down` / `status` / `all` 都通过 `--board-ip` 指定操作哪一块。

---

### 3. 子命令

| 命令 | 在哪跑 | 作用 |
|------|--------|------|
| `all`（默认） | 开发机 | export + SSH 到板子 mount + 建 `autonomy_ws` |
| `server` | 开发机 | 只配置 NFS 导出 |
| `client` | 板子本机 | 只挂载（无 SSH） |
| `status` | 开发机 | 看 export + 远端挂载（需带 `--board-ip`） |
| `down` | 开发机 | 远端 umount + 本机 unexport |

```bash
bash scripts/share_nfs_workspace.sh --help
```

---

### 4. 换开发机电脑

1. 网卡进同一网段；`ping` 通板子。  
2. `HOST_IP` 可变：不设则自动探测本机 `192.168.234.x`，或 `--host-ip`。  
3. 重新 `ssh-copy-id`；重新 `all --board-ip …`。  
4. 若板子还挂着旧电脑 IP：先在板子 `umount` 并清 `/etc/fstab` 里旧 nfs 行，再一键。  

```bash
ssh firefly@192.168.234.20
sudo umount /home/firefly/autonomy || sudo umount -l /home/firefly/autonomy
sudo sed -i '\#/home/firefly/autonomy#d' /etc/fstab
```

---

### 5. 常见问题

| 现象 | 处理 |
|------|------|
| 不知道板子 IP | 路由/交换机查看；或串口/`ip addr`；确认后 `--board-ip` |
| `status` 连错板 | 忘了带 `--board-ip`，仍用默认 `.1` |
| mount 失败 | `--host-ip` 是否为本机地址；`CLIENT_NET` 是否覆盖板子 |
| apt 解析失败 | 板子无默认路由；经开发机 NAT（见步骤 0） |
| glog 链接错误 | 用 `install_dependencies.py --profile board`，勿混用 apt glog |
| 编译很慢/锁文件 | 确认 `-B ~/autonomy_ws/build`，不是 NFS 上的 `build` |

---

### `setup.bash` 要点

- 设置 `AUTONOMY_ROOT` / `AUTONOMY_PATH` / `AUTONOMY_BUILD_DIR`
- 注入 `build/bin`、`build/lib` 到 `PATH` / `LD_LIBRARY_PATH`
- Autolink / Autonomy / Autodriver / glog 相关变量

```bash
export AUTONOMY_BUILD_DIR=$PWD/build/autonomy-minimal
export AUTONOMY_SETUP_QUIET=1
source scripts/setup_environment.bash
```
