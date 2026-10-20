# 0. FAQ 指南

## 背景

已经按 Installation / Running 做过一遍仍卡住——FAQ 按 **症状** 索引，不是替代教程。完整步骤仍以 [02 Installation](../02_Installation/00_guide.md)、[04 Running](../04_Running/00_guide.md) 和各域指南为准。

## 思路

用法三步：

1. 判断问题在哪一层：依赖与编译、运行与 launch、Autolink、导航 / BT。  
2. 打开对应 FAQ 页；若页内指向域排错表，再跟过去。  
3. 仍无解：带上「命令、日志头尾、是否 Docker/板端」进域文档。

FAQ 写的是 **常见组合**，不是全量规范。板端优先 [Installation · 嵌入式](../02_Installation/09_embedded_board.md)。导航类问题先确认 **多进程都在**，再怀疑算法。

更长对照表：[Installation §8](../02_Installation/08_troubleshooting.md)、[Running §8](../04_Running/08_troubleshooting.md)。


## 步骤

### 1. 按症状选页

| 症状偏… | 打开 |
|---------|------|
| apt / 第三方库 / 版本 | [02 安装](02_installation.md) |
| CMake / ninja / 链接 | [03 编译](03_build.md) |
| launch、进程、环境变量 | [04 运行](04_running.md) |
| 容器 | [05 Docker](05_docker.md) |
| channel / node / AUTOLINK_PATH | [06 Autolink](06_autolink.md) |
| 有目标但不动、BT Recovery | [07 导航](07_navigation.md) |

总览索引：[01 概览](01_overview.md)。

### 2. 先问这句

| 你遇到 | 先确认 |
|--------|--------|
| 编不过 | 依赖 prefix 是否统一；日志里第一个 FATAL |
| 跑不起来 | 是否 `source scripts/setup_environment.bash` |
| 对端无数据 | `AUTOLINK_PATH` + channel 名 + 类型 |
| 车不动 | planning 有无 Path；control 有无插件；costmap/TF |
