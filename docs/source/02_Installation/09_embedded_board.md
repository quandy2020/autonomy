(installation-embedded-board)=
# 9. 嵌入式板端（逐步操作）

面向 **Firefly / RK3588 等 aarch64**：开发机改代码，经 **NFS** 同步源码，在板子 **本地盘** 编译运行。

**原则**：源码可 NFS；**`build` 必须在板子本地盘**（勿用 `~/autonomy/build` 当主产物目录）。

脚本参数全文见仓库 [`scripts/README.md`](../../../scripts/README.md)。

```text
[开发机]  autonomy 源码根  ──NFS──►  [板] ~/autonomy（源码）
                                      [板] ~/autonomy_ws/build（本地产物）
                                      [板] /usr/local（第三方库）
```

---

### Step 1 — 开发机：SSH 与出网（装依赖需要）

把下面 IP 换成你的板 / 开发机地址：

```bash
# 开发机 → 板子免密
ssh-copy-id -i ~/.ssh/id_ed25519.pub firefly@192.168.234.1

# 开发机开启转发（换电脑时做一次）
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING \
  -s 192.168.234.0/24 ! -d 192.168.234.0/24 -j MASQUERADE

# 板子默认路由走开发机
ssh firefly@192.168.234.1 \
  'sudo ip route replace default via 192.168.234.50'
```

---

### Step 2 — 开发机：一键 NFS

在 **开发机源码根**：

```bash
cd /path/to/autonomy
bash scripts/share_nfs_workspace.sh all --board-ip 192.168.234.1
bash scripts/share_nfs_workspace.sh status --board-ip 192.168.234.1
```

成功：板子上 `df -h ~/autonomy` 为 NFS；`~/autonomy_ws/src/autonomy` 指向该挂载。

换板 / 换网段：

```bash
bash scripts/share_nfs_workspace.sh all \
  --board-ip 10.0.0.5 --board-user firefly \
  --host-ip 10.0.0.1 --client-net 10.0.0.0/24
```

或用配置档：`scripts/nfs_boards/<name>.env` + `--board-profile <name>`。

卸挂载：`bash scripts/share_nfs_workspace.sh down --board-ip …`

---

### Step 3 — 板端：安装依赖

```bash
ssh firefly@192.168.234.1
ping -c1 8.8.8.8          # 确认出网

cd ~/autonomy
python3 scripts/install_dependencies.py --profile board --skip-installed
```

| 要点 | 说明 |
|------|------|
| `--profile board` | 对齐 aarch64 Dockerfile；无 Qt/Ogre/Sphinx |
| 安装前缀 | glog / protobuf **3.19** / gRPC / ceres 等 → **`/usr/local`** |
| 冲突 apt | 脚本会 purge apt 版 glog/protobuf/grpc，避免混 ABI |
| 续装 | `--thirdparty-only --resume-from install_protobuf.sh --skip-installed` |
| 内存 | `AUTONOMY_MAKE_JOBS=2 bash docker/install/install_ceres_solver.sh` |

```bash
python3 scripts/install_dependencies.py --list --profile board
/usr/local/bin/protoc --version    # 期望 3.19.x
```

---

### Step 4 — 板端：CMake 构建（本地盘）

```bash
cd ~/autonomy_ws

cmake -S src/autonomy -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/usr/local \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DBUILD_AUTOVIZ=OFF \
  -DBUILD_ORBISVIEW=OFF \
  -DBUILD_DOCS=OFF \
  -DBUILD_TEST=OFF

cmake --build build -j$(nproc)
# 内存紧：-j2 或 -j4
```

只需部分模块时，加上与本机相同的 `-DAUTONOMY_BUILD_*=` / `-DBUILD_*=`，见 [§6.2](06_build.md)。

开发机改代码后 NFS 即时可见，板子再执行 `cmake --build build` 即可。

---

### Step 5 — 板端：环境与验证

**方式 A — 直接用 build（开发）**

```bash
export AUTONOMY_BUILD_DIR=$HOME/autonomy_ws/build
source $HOME/autonomy/scripts/setup_environment.bash
ls "$AUTONOMY_BUILD_DIR/lib"/libautonomy_common.so
which autolink
```

**方式 B — 安装到 `/usr/local`（车上常用）**

```bash
sudo cmake --install ~/autonomy_ws/build --prefix /usr/local
source /usr/local/share/autonomy/setup.bash
# 勿再让旧 build/bin 排在 PATH 前面
autolink launch start task.launch
```

板端资源实测：[板端 task.launch 报告](../04_Running/09_board_task_launch_benchmark.md)。

---

### Step 6 — 换板 / 换开发机

```bash
# 开发机
bash scripts/share_nfs_workspace.sh down --board-ip 192.168.234.1
bash scripts/share_nfs_workspace.sh all  --board-ip 192.168.234.20
```

换开发机后若板子仍挂旧 IP：

```bash
ssh firefly@<板IP>
sudo umount ~/autonomy || sudo umount -l ~/autonomy
sudo sed -i '\#/home/firefly/autonomy#d' /etc/fstab
```

再在新电脑源码根执行 Step 2。

---

### 常见问题（板端）

| 现象 | 处理 |
|------|------|
| apt 无法解析域名 | NAT + 默认路由 + `nameserver 8.8.8.8` |
| 编译极慢 | 确认 `-B` 在本地盘，不是 NFS |
| glog / protobuf 链接错乱 | 只用 `/usr/local`；`--profile board` |
| OOM | `-j2`；Ceres 用 `AUTONOMY_MAKE_JOBS=2` |
| launch / 二进制版本不对 | install 后 `source /usr/local/share/autonomy/setup.bash`，检查 `which autolink` |

通用排错：[§8](08_troubleshooting.md)。
