# 7. 环境配置

编译完成后，**每个新终端**先加载环境，再运行 `autolink` / 各 `autonomy.*`。

---

### Step 1 — 选择哪份 setup

| 场景 | 命令 |
|------|------|
| 开发：直接用 `build/` | `source scripts/setup_environment.bash`（在源码根） |
| 已 `cmake --install` 到 `/usr/local` | `source /usr/local/share/autonomy/setup.bash` |
| build 不在默认 `源码根/build` | 先 `export AUTONOMY_BUILD_DIR=/你的/build` 再 source 源码版脚本 |

源码版脚本路径：[`scripts/setup_environment.bash`](../../../scripts/setup_environment.bash)。可重复 `source`，幂等。

Docker 内：

```bash
source /workspace/autonomy/scripts/setup_environment.bash
```

板端 NFS 工作区示例：

```bash
export AUTONOMY_BUILD_DIR=$HOME/autonomy_ws/build
source $HOME/autonomy/scripts/setup_environment.bash
# 或已 install：
# source /usr/local/share/autonomy/setup.bash
```

---

### Step 2 — 确认生效

```bash
echo "$AUTONOMY_PATH"
which autolink
echo "$AUTOLINK_LAUNCH_PATH" | tr ':' '\n' | head
```

`PATH` 应优先指向你打算用的 `bin`（install 后不要让旧的 `build/bin` 盖住 `/usr/local/bin`）。

---

### Step 3 — 常用覆盖（可选）

在 `source` **之前**设置：

| 变量 | 含义 |
|------|------|
| `AUTONOMY_BUILD_DIR` | 构建目录 |
| `AUTONOMY_INSTALL_PREFIX` | 安装前缀 |
| `AUTONOMY_SETUP_QUIET=1` | 少打印 |
| `AUTONOMY_SETUP_ROS=1` | 尝试 source ROS 与仓库 `install/setup.bash` |

日志（setup 已有默认，可再改）：

```bash
export GLOG_logtostderr=1
export GLOG_minloglevel=0
export GLOG_log_dir=${HOME}/.autonomy/log
```

跨机 RTPS 时再改 `AUTOLINK_IP` / `AUTOLINK_DISCOVERY_SERVER`（见 Communication · FastDDS）。

---

### Step 4 — 跑起来

```bash
source scripts/setup_environment.bash
autolink_launch autonomy.launch
# 或按 Running 文档启动 task.launch 等
```

配置根默认在仓库 `config/`（`AUTONOMY_CONFIG_DIR`）。运行细节见 [04 Running](../04_Running/00_guide.md)。
