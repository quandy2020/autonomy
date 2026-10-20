(sando-controller)=
# 6. SANDO Controller

地面平面 **滚动时域**：动态障碍聚类 → 热度 A* → 时空走廊 → Bézier + MIQP 最小急动度。文献：Kondo et al., arXiv:2604.07599。源码：`sando_controller/`。

在局部 costmap 上聚类动态障，建时空走廊并用 Bézier + MIQP 最小急动度。动态障碍明显、需要走廊约束的局部轨迹时考虑；纯静态地图或算力紧张时 MPPI/RPP 更合适。

## 何时用

局部 costmap 里 **动态障碍** 明显、需要走廊约束的轨迹；算力与调参成本高于 MPPI/RPP。纯静态地图不必上 SANDO。

## 流水线

1. 局部 costmap 聚类动态障（盒 + 速度）
2. 热度场推开几何路径
3. 按时间分层收走廊多面体
4. 三次 Bézier；MIQP（DAQ P）选控制点
5. 只执行已提交前缀；失败沿用旧轨迹或悬停

输出机体系速度；航向在写出前滤波。无高度通道。

## 主要模块

`obstacle_tracker`、`path_searcher`、`safe_corridor`、`trajectory_optimizer`、`sando_controller` 插件入口。

## 配置

`sando_defaults` 与 control conf 中 SANDO proto。需要局部 costmap、位姿、速度。

## 常见坑

| 现象 | 处理 |
|------|------|
| 频繁悬停 | 走廊不可行；动态盒过保守 |
| 延迟 | MIQP 规模；降 horizon |
| 与全局 Path 脱节 | 热度搜索窗口与 Path 裁剪 |

归属 [04](../04_controller_algorithms.md)。
