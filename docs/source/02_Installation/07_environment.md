# 7. 环境配置

### 7.1 一键加载（推荐）

仓库根目录执行：

```bash
source scripts/setup.bash
```

会设置本工程运行/开发常用环境变量（`AUTONOMY_*` / `AUTOLINK_*` / `AUTODRIVER_*` / `GLOG_*` / `PATH` / `LD_LIBRARY_PATH` 等），幂等可重复 `source`。

| 覆盖变量（source 前） | 含义 |
|----------------------|------|
| `AUTONOMY_BUILD_DIR` | 构建目录，默认 `$ROOT/build` |
| `AUTONOMY_INSTALL_PREFIX` | 安装前缀；若存在则优先加入 `PATH`/`lib`，并作为 `AUTONOMY_PATH` |
| `AUTONOMY_SETUP_QUIET=1` | 不打印摘要 |
| `AUTONOMY_SETUP_ROS=1` | 尝试 `source /opt/ros/*/setup.bash` 与仓库 `install/setup.bash` |

脚本路径：[`scripts/setup.bash`](../../../scripts/setup.bash)。

### 7.2 日志（glog）

`setup.bash` 默认已设置；也可单独覆盖：

```bash
export GLOG_logtostderr=1           # 输出到 stderr
export GLOG_alsologtostderr=0       # 不同时写文件
export GLOG_colorlogtostderr=1      # 彩色日志
export GLOG_minloglevel=0           # 0=INFO, 1=WARNING, 2=ERROR, 3=FATAL
export GLOG_log_dir=${HOME}/.autonomy/log
```

| 变量 | 推荐值 | 说明 |
|------|--------|------|
| `GLOG_logtostderr` | `1` | 终端可见日志 |
| `GLOG_minloglevel` | `0` | 开发时用 INFO |
| `GLOG_log_dir` | `~/.autonomy/log` | 持久化日志目录 |

### 7.3 Autonomy 运行配置

| 变量 / 路径 | 说明 |
|-------------|------|
| `AUTONOMY_PATH` | 工作区/安装前缀（`LoadModuleConf` / bridge conf） |
| `AUTONOMY_CONF_PATH` | 冒号分隔的 conf 搜索根 |
| `AUTONOMY_CONFIG_DIR` | Lua 配置根（默认 `config/`） |
| `AUTONOMY_BT_PLUGIN_PATH` | BT 插件 `.so` 搜索路径 |
| `AUTONOMY_ENV` | Docker 挂载用仓库根路径 |
| `BRIDGE` / `AUTONOMY_BRIDGE_TARGET` | Bridge 客户端默认 `host:port` |

Navigator / Task BT 模式（已 `source scripts/setup.bash` 后通常无需再 export）：

```bash
source scripts/setup.bash
autolink_launch autonomy.launch
```

### 7.4 Autolink 路径

`setup.bash` 默认绑定本仓库：

```bash
# AUTOLINK_PATH → <repo>/autolink/autolink
# AUTOLINK_LAUNCH_PATH → system/task/perception/… launch 目录
# AUTOLINK_DOMAIN_ID=80  AUTOLINK_IP=127.0.0.1
```

跨机 RTPS 时再改 `AUTOLINK_IP` / `AUTOLINK_DISCOVERY_SERVER`（见 Communication · FastDDS）。

### 7.5 ROS 2 环境（可选）

```bash
export AUTONOMY_SETUP_ROS=1
source scripts/setup.bash
# 或手动：
# source /opt/ros/humble/setup.bash
# source /path/to/install/setup.bash
```

详见 [04 Running §6](../04_Running/06_ros2_integration.md)。

### 7.6 Docker 容器内

```bash
### Autonomy ###
source /workspace/autonomy/scripts/setup.bash
# 可选 ROS 2
# export AUTONOMY_SETUP_ROS=1
```

```bash
source ~/.bashrc   # 若已写入上述 source
```

### 7.7 配置目录结构

运行时通过 `config_directory` / `AUTONOMY_CONFIG_DIR` 指定配置根目录：

```
config/
├── autonomy.lua          # 顶层入口
├── common.lua            # 共享帧名、容差
├── planner/planner.lua
├── controller/controller.lua
└── navigator/navigator.lua
```

应用代码：

```cpp
auto options = autonomy::system::CreateOptions("config");
```

### 7.8 相关文档

- [16 Navigator · 使用指南](../16_Navigator/00_guide.md)
- [04 Running](../04_Running/06_ros2_integration.md)
- [`scripts/README.md`](../../../scripts/README.md)
