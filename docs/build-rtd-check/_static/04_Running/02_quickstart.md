# 2. 快速运行

> 前提：已完成 [02 Installation](../02_Installation/02_quickstart.md) 编译。

### 2.1 最小命令（BT 单点导航）

```bash
export AUTONOMY_BT_PLUGIN_PATH=$(pwd)/build/lib
export GLOG_logtostderr=1

./build/bin/autonomy_nav_test \
  --configuration_directory=config \
  --start_x=1 --start_y=1 --start_yaw=0 \
  --goal_x=5 --goal_y=5 --goal_yaw=0 \
  --use_bt=true \
  --timeout_sec=120
```

成功时输出：`Navigation succeeded.` 及 `Last path poses: N`。

### 2.2 构建测试工具

若 `autonomy_nav_test` 不存在，启用 `BUILD_TOOLS` 后重新编译：

```bash
cmake -G Ninja .. -DBUILD_TOOLS=ON
ninja autonomy_nav_test
```

### 2.3 起终点选择

默认地图 `config/data/map.pgm` 为世界坐标系栅格。起终点须在**自由空间**内，例如 `(1,1) → (5,5)`。

### 2.4 下一步

| 目标 | 文档 |
|------|------|
| 参数详解 | [§4 离线导航测试](04_nav_test.md) |
| 常驻进程 | [§3 Autonomy 进程](03_autonomy_process.md) |
| Docker 内运行 | [§5 Docker 运行时](05_docker_runtime.md) |
