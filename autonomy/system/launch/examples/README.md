# Autonomy system examples

基于 [`../autonomy.launch`](../autonomy.launch) 拆出的场景入口，互斥：不要与
`autonomy.launch` / `exploration.launch` / `task/launch/task.launch` 同时启动
（进程名重叠）。

| Example | Launch | 进程 |
|---------|--------|------|
| 1 SLAM 建图与定位 | `examples/slam.launch` | monitor + localization + task |
| 2 导航 | `examples/navigation.launch` | monitor + localization + planning + control + task + bridge |
| 3 跟随 | `examples/follow.launch` | monitor + localization + planning + control + task + perception |
| 4 智能遥操 | `examples/teleop.launch` | monitor + task + perception + bridge |

## 启动

```bash
# 仓库根目录
export PATH=$PWD/build/bin:$PATH
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/system/launch

mkdir -p data/maps

autolink_launch examples/slam.launch          # 1 建图
autolink_launch examples/navigation.launch    # 2 导航（需已有 pbstream）
autolink_launch examples/follow.launch        # 3 跟随
autolink_launch examples/teleop.launch        # 4 遥操（需 -DBUILD_GRPC=ON）
```

停止：`autolink launch stop examples/<name>.launch`（或与 start 时相同的路径）。

## 推荐顺序

1. `slam.launch` 建图 → `data/maps/cartographer.pbstream`
2. `navigation.launch` / `follow.launch` 加载该地图做导航或跟随
3. `teleop.launch` 独立用于桥接遥操（可不依赖地图）

定位后端默认 Cartographer；3D LIO 将对应 launch 内 `localization` 的
`process_name` 换成 `localization_argus.launch` 中的命令即可。

BT 与任务配置见 `autonomy/task/conf/behavior_tree/README.md`。
