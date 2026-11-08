# Mapping 模块总览

`mapping/` 拥有 Voxel-SLAM 地图表示、地图算法、持久化格式和传输无关的可视化数据视图。前端可更新活动地图；backend 通过只读快照运行全局算法；serializer 负责格式转换但不控制 session 生命周期；visualization 只生成展示数据，不承载地图真值。

## 子模块职责

| 子模块 | 输入 | 输出 | 状态所有者 | 明确不负责 |
|---|---|---|---|---|
| `voxel/` | 去畸变扫描、位姿、关键帧观测 | 自适应体素、平面/点簇统计、地图快照、HBA 优化结果 | 体素索引、点簇、层级结构 | session 文件读写、ROS/RViz 展示、后台任务生命周期 |
| `serialization/` | 明确版本的 session/map 值对象或只读快照 | 文件/字节格式、结构化加载结果 | 编解码状态和格式兼容规则 | 决定哪些 session 参与 SLAM、修改运行时地图 |
| `visualization/` | const 地图/轨迹快照 | 传输无关的点/线/体素可视化数据 | 不长期拥有地图状态 | 地图真值、优化、ROS publisher、渲染窗口 |

## 子模块设计文档

- [自适应体素地图与全局 HBA](mapping.md)
- [地图与会话持久化](mapping/serialization.md)
- [地图可视化数据视图](mapping/visualization.md)

## 数据流

```text
frontend scan / pose
  -> mapping: update active map and keyframe statistics
  -> immutable MapSnapshot
       ├── backend/global_mapping: asynchronous HBA request
       ├── mapping/serialization: save/load versioned data
       └── mapping/visualization: transport-neutral display view
```

`mapping/visualization` 是可选投影模块，不是 Voxel-SLAM 核心算法的前置依赖。当前没有地图可视化导出工具；若未来出现实际消费者，再按共享数据契约决定是否实现该模块，不需为目录完整而创建空模块。

## 现有 Atlas 与目标命名的关系

- 当前 C++ 路径是 `map/` 和 `map/serialization/`；目标树统一使用 `mapping/`、`mapping/serialization/`、`mapping/visualization/`。
- 原有 `frontend/visual/` 与 `map/visual/` 仅包含未进入 CMake、没有外部代码消费者的视觉 SLAM 占位文件，已从本次 Voxel-SLAM 移植工程中清理。若未来引入视觉 SLAM，须作为独立、有实际输入输出和构建目标的模块重新设计。
- `map/serialization/voxel_map_serializer.*` 当前还承担轨迹/session 文本和二进制点云 IO；目标应按数据类型拆分，不应继续用 `VoxelMapSerializer` 一类名称包办所有格式。
- 所有路径变更按依赖和 CMake target 增量迁移；上表不表示源码已经移动。
