# 7. 环境配置

### 7.1 日志（glog）

建议在 `~/.bashrc` 或容器启动脚本中设置：

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

### 7.2 Autonomy 运行配置

| 变量 / 路径 | 说明 |
|-------------|------|
| `config/` | 默认配置目录（`autonomy.lua` 等） |
| `AUTONOMY_BT_PLUGIN_PATH` | BT 插件 `.so` 搜索路径（Navigator） |
| `AUTONOMY_ENV` | Docker 挂载用仓库根路径 |

Navigator BT 模式示例：

```bash
export AUTONOMY_BT_PLUGIN_PATH=/path/to/autonomy/build/lib
./bin/nav_test --config_directory=config --use_bt=true
```

### 7.3 Autolink 路径（ROS 2 集成场景）

若与 ROS 2 工具链共存，部分脚本引用：

```bash
export AUTOLINK_PATH=/path/to/autonomy/autolink/autolink
```

纯 CMake 构建通常**不需要**设置此项。

### 7.4 ROS 2 环境（可选）

仅在需要 `ros2 run` / RViz2 时加载：

```bash
source /opt/ros/humble/setup.bash
# 若有 overlay workspace
source /path/to/install/setup.bash
```

详见 [04 Running §6](../04_Running/06_ros2_integration.md)。

### 7.5 Docker 容器内 `.bashrc` 模板

```bash
### Autonomy ###
export GLOG_logtostderr=1
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0

# 若在容器内编译
export AUTONOMY_BT_PLUGIN_PATH=/workspace/autonomy/build/lib

# 可选 ROS 2
# source /opt/ros/humble/setup.bash
```

```bash
source ~/.bashrc
```

### 7.6 配置目录结构

运行时通过 `config_directory` 指定配置根目录：

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

### 7.7 相关文档

- [16 Navigator · 使用指南](../16_Navigator/04_usage.md)
- [04 Running](../04_Running/06_ros2_integration.md)
