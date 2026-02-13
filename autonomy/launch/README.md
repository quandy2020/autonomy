# Autonomy Launch（基于 mainboard 启动）

本目录提供基于 **autolink mainboard** 的启动方式说明与示例，使 autonomy 任务（含行为树导航等）通过 DAG 配置以组件形式运行。

## mainboard 启动流程简述

1. **解析命令行**：`-d` DAG 配置文件、`-p` 进程组名、`-s` 调度名、`--plugin` 插件描述等。
2. **初始化 autolink**：`autolink::Init(argv[0], dag_info)`。
3. **加载 DAG**：根据 `-d` 指定的 proto 配置文件，通过 `ModuleController` 加载 `module_library`（.so），并创建各 `ComponentBase` 实例、执行 `Initialize(config)`。
4. **阻塞直到退出**：`autolink::WaitForShutdown()`，收到 SIGTERM 后清理并退出。

环境变量（用于解析 DAG 与库路径）：

- `AUTOLINK_DAG_PATH`：DAG 配置文件搜索目录（可多路复用，用 `:` 分隔）。
- `AUTOLINK_LIB_PATH`：`module_library` 动态库搜索目录（可多路复用）。
- `AUTOLINK_LAUNCH_PATH` / `AUTONOMY_LAUNCH_PATH`：autolink_launch.py 查找 launch XML 的目录（默认 `/autolink`）。

## 方式一：直接运行 mainboard

确保 `mainboard` 已在 PATH 中（例如安装后 `install/autolink/bin` 或 `install/autonomy/bin`），并设置 DAG/库搜索路径后执行：

```bash
# 可选：设置 DAG 与库搜索路径（指向本仓库或安装目录下的 dag / lib）
export AUTOLINK_DAG_PATH="/path/to/autonomy/launch/dag:/path/to/autolink/examples"
export AUTOLINK_LIB_PATH="/path/to/install/lib"

# 启动一个 DAG
mainboard -d autonomy.dag -p autonomy_main

# 启动多个 DAG
mainboard -d dag1.dag -d dag2.dag -p process_group
```

帮助信息：

```bash
mainboard -h
```

## 方式二：使用 autolink_launch.py 启动

使用 autolink 提供的 Python 启动脚本，通过 **launch XML** 描述要启动的 mainboard 模块及 DAG 列表。

1. **设置 launch 搜索路径**（使脚本能找到本目录下的 `mainboard.launch`）：

   ```bash
   export AUTONOMY_LAUNCH_PATH="/path/to/autonomy/launch"
   # 或
   export AUTOLINK_LAUNCH_PATH="/path/to/autonomy/launch"
   ```

2. **启动**：

   ```bash
   python3 /path/to/autolink_launch/autolink_launch.py start mainboard.launch
   ```

   若不指定文件名，脚本会使用默认 launch 文件（依赖上述路径下是否存在 `autolink.launch` 等）。

3. **停止**：

   ```bash
   python3 /path/to/autolink_launch/autolink_launch.py stop mainboard.launch
   ```

launch XML 中每个 `<module type="library">` 会对应一个 mainboard 进程，其参数包括：

- `dag_conf`：DAG 配置文件路径（可多个），与 mainboard 的 `-d` 一一对应。
- `process_name`：进程组名，对应 mainboard 的 `-p`。
- `sched_name`：调度名，对应 mainboard 的 `-s`。
- `plugin`：可选，插件描述路径，对应 mainboard 的 `--plugin`。

## 各模块 launch 与脚本

以下 launch 文件与脚本分别用于启动 **tasks**、**system**、**bridge**、**planning**、**driver**、**control** 模块（可与 autolink_launch 或直接执行脚本使用）。

| 模块       | Launch 文件      | 脚本                  | 说明 |
|------------|------------------|-----------------------|------|
| **tasks**  | `tasks.launch`   | `scripts/run_tasks.sh` | 行为树导航、TaskManager；二进制 `autonomy.tasks.launcher`（若 CMake 未单独建 target，需在 CMake 中增加并安装）。 |
| **system** | `system.launch`  | `scripts/run_system.sh` | AutonomyNode（map、planner、controller、tasks）；二进制 `autonomy.system.launcher`。 |
| **bridge** | `bridge.launch`  | `scripts/run_bridge.sh` | gRPC/MQTT 桥接；二进制 `autonomy.bridge.launcher`。 |
| **planning** | `planning.launch` | `scripts/run_planning.sh` | 无独立进程，当前通过启动 system 带起 PlannerServer。 |
| **driver** | `driver.launch`  | `scripts/run_driver.sh` | 无独立进程，当前通过启动 system 带起；或由 mainboard + DAG 加载 driver 组件。 |
| **control** | `control.launch` | `scripts/run_control.sh` | 无独立进程，当前通过启动 system 带起 ControllerServer。 |

**使用方式：**

- **通过 autolink_launch**（需 `AUTONOMY_LAUNCH_PATH` 或 `AUTOLINK_LAUNCH_PATH` 指向本目录）：
  ```bash
  python3 /path/to/autolink_launch.py start system.launch
  python3 /path/to/autolink_launch.py start tasks.launch
  python3 /path/to/autolink_launch.py stop system.launch
  ```
- **直接运行脚本**（需安装后 `bin` 在 PATH 中，或先 `source install/setup.bash`）：
  ```bash
  ./scripts/run_system.sh
  ./scripts/run_tasks.sh
  export AUTONOMY_CONFIG_DIR=/path/to/configuration_files
  ./scripts/run_system.sh
  ```

launch XML 中 `process_name` 若包含配置路径（如 `/opt/autonomy/configuration_files`），可按部署环境修改；脚本默认使用 `AUTONOMY_CONFIG_DIR`（未设时相对本目录解析为 `../../../configuration_files`）。

## 本目录文件说明

| 文件/目录           | 说明 |
|---------------------|------|
| `README.md`         | 本说明（mainboard 如何启动、各模块 launch）。 |
| `mainboard.launch`  | 供 autolink_launch 使用的 XML，内含一个 mainboard 模块及 dag_conf。 |
| `tasks.launch` ~ `control.launch` | 各模块对应的 autolink_launch XML（type=binary）。 |
| `dag/`              | DAG 配置示例目录；可将自定义 `.dag` 放于此，并令 `AUTOLINK_DAG_PATH` 包含此目录。 |
| `dag/autonomy.dag`  | 示例 DAG（proto 文本），需根据实际 component 与 so 路径修改。 |
| `scripts/launch_mainboard.sh` | 可选脚本：设置环境并直接调用 mainboard 或 autolink_launch。 |
| `scripts/run_*.sh`  | 各模块直接启动脚本（使用环境变量或默认配置路径）。 |

## 自定义 DAG 与组件

- DAG 为 proto 文本格式，见 `autolink/proto/dag_conf.proto`（`DagConfig` / `ModuleConfig` / `ComponentInfo`）。
- 每个 `module_config` 指定 `module_library`（.so 路径，可由 `AUTOLINK_LIB_PATH` 解析）和若干 `components`（`class_name` + `config`）。
- 组件需继承 `autolink::ComponentBase` 并实现 `Initialize(config)`，并在该 so 中注册，mainboard 会按 DAG 加载 so 并创建实例。

将 autonomy 侧需要跑在 mainboard 里的功能封装为上述组件后，在 `dag/` 中编写对应 `.dag`，并在 `mainboard.launch` 的 `dag_conf` 中引用即可实现“根据 mainboard 启动”。
