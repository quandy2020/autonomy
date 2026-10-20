# 2. 快速运行

## 背景

前提：[Installation · 快速安装](../02_Installation/02_quickstart.md) 已编译并 `source scripts/setup_environment.bash`。

## 思路

按固定顺序执行：依赖/环境 → 构建或配置 → 启动 → 最小验证；分支放到文末链接。


## 步骤

### 1. 启动

```bash
source scripts/setup_environment.bash
autolink_launch autonomy.launch
```

拉起 monitor / planning / control / task / perception / bridge / foxglove 等。发令用 Bridge、Action Client 或上层业务。

### 2. 探索闭环（可选）

```bash
autolink_launch exploration.launch
```

launch 文件在 `autonomy/system/launch/exploration.launch`。与 `autonomy.launch` 进程重叠，**勿同时启动**。

### 3. 配置落点

| 类型 | 路径 |
|------|------|
| 共享快照 | `autonomy/system/conf/autonomy.pb.txt` |
| 地图 | `autonomy/map/conf/` |
| BT XML | `autonomy/task/conf/behavior_tree/` |
