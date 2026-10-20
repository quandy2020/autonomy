# 7. 运行验证

## 背景

栈起来之后怎么确认进程、库、发令链路正常。不是自动化测试清单，而是联调时常用手顺。

## 思路

以 conf / launch / 环境变量为主线，命令可复制；坑放在步骤末尾表。


## 步骤

### 1. 多进程栈

```bash
source scripts/setup_environment.bash
autolink_launch autonomy.launch
```

成功：各进程无致命错误；经 Bridge / Action 发令后任务可推进。仓库里已无 `autonomy_nav_test` 聚合入口。

### 2. 产物

```bash
ls build/lib/libautonomy_common.so build/lib/libautolink.so
ls build/bin/autonomy.planning build/bin/autonomy.control build/bin/autonomy.task
ls build/bin/autolink_launch
ls build/lib/*bt* 2>/dev/null | head
```

### 3. ROS 2（若启用）

```bash
ros2 node list
ros2 topic list
ros2 topic echo /plan
```

### 4. 可视化

```bash
rviz2
# Foxglove：ws://localhost:8765（launch 可含 foxglove_bridge）
```

见 [13 Visualization](../13_Visualization/07_rviz2_ros2.md)。

### 5. 清单

| 检查项 | 方法 |
|--------|------|
| 配置 | 启动日志无 conf 路径错误 |
| BT 插件 | `AUTONOMY_BT_PLUGIN_PATH` 含 `.so` |
| 地图 | planning 日志含 map / costmap 初始化 |
| TF | 仿真 / Bridge 发布 `map→base_link` |
| 发令 | Bridge / Action 有 Goal / Feedback |
| 控制 | Follow 阶段有 `cmd_vel` |
