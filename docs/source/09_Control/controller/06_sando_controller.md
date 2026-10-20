(sando-controller)=
# 6. SANDO Controller

源码：`autonomy/control/controller/sando_controller/`。地面平面滚动时域规划：热度加权 A* → 时空多面体走廊 → 三次 Bézier / MIQP 最小急动度。文献：Kondo et al., arXiv:2604.07599。

### 流水线

1. 局部代价地图聚类动态障碍（中心、半轴、速度）
2. 热度场推开几何路径（不删起点可达边）
3. 按时间分层，自由空间收成多面体走廊
4. 每层三次曲线；Bézier 控制点把无限点约束变为有限点
5. MIQP（vendored DAQP）选多面体与急动度
6. 只执行已提交前缀，失败则沿用上一条或悬停

输出：机体系速度指令；航向在指令写出前滤波。无高度通道。

### 主要类

| 模块 | 作用 |
|------|------|
| `obstacle_tracker` | 动态盒跟踪 |
| `path_searcher` | 热度 A* |
| `safe_corridor` | 时空走廊 |
| `trajectory_optimizer` / `layered_*` | 多项式 + MIQP |
| `sando_controller` | 插件入口 |

### 配置

见 `sando_defaults` / control conf 中 SANDO 相关 proto。需要局部 costmap 与位姿速度。
