# Autonomy 完整区域覆盖导航

本模块参考 Open Navigation 的 `opennav_coverage`，固定上游提交为 `c6fbf083d120f3c2ce3635e02caa33b5fa632559`，将规划流水线、路线方法、预标注行模式、BT 节点和覆盖导航适配为 Autonomy C++ / Autolink 实现。移植源码保留 Apache-2.0 许可，见 [LICENSE](LICENSE)。底层使用本仓库 [Fields2Cover](../../../planning/planner/fields2cover/fields2cover.hpp)，相关算法文档见 [planning/docs](../../../planning/docs/README.md)。

## 接口与数据流

- `/compute_coverage_path`：`ComputeCoveragePathAction`，输入区域、孔洞、阶段开关及算法模式，输出全路径、条带、转弯、逐状态速度/方向和规划指标。
- `/navigate_complete_coverage`：`NavigateCompleteCoverageAction`，接收覆盖任务，计算覆盖计划、到达起点、分段执行，返回实际控制 Action 终态。任务参与 `TaskScheduler` 的移动互斥和撤销许可。
- `/coverage/plan`：全覆盖路径预览；`/coverage/field`、`/coverage/headland`、`/coverage/swaths` 和 `/coverage/markers`：田地、核心区和作业带调试几何。
- `/coverage/tool_command`、`/coverage/tool_ack`：工具启停与可选确认，按任务 ID 和序号关联。未确认的必需工具命令会中止执行。

`CoverageRequest → Field → Headland / Decomposition → SwathsByCells → Route → checked Path → bounded execution segments → native FollowPath`。

BT 默认顺序为 `ComputeCoveragePath → NavigateToCoverageStart → FollowCoveragePath`，见 [默认树](conf/navigate_complete_coverage.xml)。计算只执行一次；每一段在前一段收到成功终态后才提交。段间保留必要曲线点、方向、作业类型和速度信息，不以均匀抽样删除路径拐弯。

原生 Action 接受的目标必须显式进入 `EXECUTING` 后再发布成功、失败或取消结果。端点保留服务直至已接受的工作完成，客户端取消针对本目标，不调用全局取消。

## 构建

容器工作区 `/workspace/autonomy` 对应宿主机 `/home/quandy/workspace/github/autonomy`。

```bash
cd /workspace/autonomy
source /opt/ros/humble/setup.bash
colcon build --packages-up-to autonomy --cmake-args \
  -DAUTONOMY_BUILD_FIELDS2COVER=ON \
  -DAUTONOMY_BUILD_COVERAGE=ON \
  -DAUTONOMY_BUILD_COVERAGE_TESTS=ON

ctest --test-dir build/autonomy \
  -R '^(task\.coverage.*test|planning\.fields2cover_test)$' --output-on-failure
```

覆盖任务默认不编译，开启时必须同时开启 Fields2Cover。任务程序通过 `--enable_coverage` 启用服务，通过 `--coverage_config` 加载 [运行配置](conf/autosim_coverage.json)。专项覆盖算法测试、实际原生端点测试和底层 Fields2Cover 回归各自注册为 CTest。

## AutoSim 与 AutoViz

[覆盖仿真配置](../../../../autosim/config/coverage.yaml) 使用可重复的开放区域，发布 `/map`、TF、`/odom` 和 `/gt/pose`。运行配置使用车宽 0.35 m、作业宽 0.8 m、转弯半径 0.4 m、地头 1.2 m；控制配置为 [原生 RPP](conf/coverage_controller.pb.txt)。生产田地和真实机器人需独立配置边界、障碍和实际工具。

```bash
python3 src/autonomy/autonomy/task/navigation/coverage/scripts/demo_runtime.py start
python3 src/autonomy/autonomy/task/navigation/coverage/scripts/demo_runtime.py status
# 仅停止该脚本记录且身份仍一致的进程：
python3 src/autonomy/autonomy/task/navigation/coverage/scripts/demo_runtime.py stop
```

避免同时运行另一套发布相同地图、TF 或速度通道的仿真/导航服务。脚本的 `--headless` 选项只省略 AutoViz。

在 AutoViz 的 **Panels → Navigation → Coverage** 中：

1. 输入外环和孔洞。第一环是作业区域，其余环是排除区域；坐标使用米。
2. 点击 **Preview**，观察区域、地头边界、条带与完整路径。
3. 选择是否先到覆盖起点，点击 **Start**。页面显示覆盖进度和剩余距离。
4. **Cancel coverage** 取消本任务并等待控制动作的终态。暂停/恢复暂不开放，避免重复已完成的工具作业。
5. Run 区固定在面板底部，设置区独立滚动；**Preview**/**Start**/**Cancel** 不会因障碍列表过长而滚出视野。
6. 画障碍物多边形：按 **Ctrl+Alt+O**（或点 **Draw polygon**），
   面板会自动切到 Publish Point 工具；在 3D 视图依次点击至少 3 个点，
   **Enter** 完成，**Esc** 取消。点位实时显示为绿色折线和顶点，
   完成后转为橙色障碍环并进入请求的孔洞列表。
7. 动态障碍：勾选 **Capture from /map** 并设置 **Margin**，
   点 **Refresh** 或等待每秒刷新；占用栅格中的小连通块会转成障碍环，
   静态长墙会按面积阈值跳过。勾选 **Re-plan when obstacles change**
   后，障碍栅格变化会自动触发一次全路径预览。
8. **Robot radius** 控制条带间距，作业宽度等于半径的两倍；
   需要按附件实际扫幅执行时，可在配置文件显式设置 `operation_width`。

默认编辑区域为 `[-3,3] × [-2,2]`，见 [同一请求的文本协议示例](conf/demo_request.pb.txt)。区域坐标可以直接替换为实际地图中可通行的区域。

## 原生客户端与仿真记录

```bash
build/autonomy/bin/task.coverage_client plan \
  src/autonomy/autonomy/task/navigation/coverage/conf/demo_request.pb.txt \
  artifacts/coverage-runtime/plan

build/autonomy/bin/task.coverage_client navigate \
  src/autonomy/autonomy/task/navigation/coverage/conf/demo_request.pb.txt \
  artifacts/coverage-runtime/navigation 120

# 观察 AutoViz 发起的任务，并为仿真工具提供确认：
build/autonomy/bin/task.coverage_client observe \
  src/autonomy/autonomy/task/navigation/coverage/conf/demo_request.pb.txt \
  artifacts/coverage-runtime/gui-run 120
```

客户端记录终态协议、真实 `/gt/pose` 位姿和工具命令时间序列。`navigate` 示例要求工具确认；该客户端提供的确认仅表示仿真工具协议验证，不是实体工具反馈。

## 地图分区

Coverage 支持两种分区来源，并在 AutoViz 用不同颜色块显示：

- **From map**：直接读取订阅的 `/map` 占位栅格。
  `RegionSegmenter::MapSplit` 先清理 1 格宽的墙边碎片，按
  `min_passage_width`（默认 1.4 m）做腐蚀得到房间核心，再按 4 连通回填，
  最后沿栅格角点追踪出**轴对齐**轮廓。门洞窄于该值时两个房间会被分开，
  宽于该值时仍视为同一房间。
- **Split map**：按 `columns × rows` 网格切分工作区包围盒，适合没有地图时调试。

分区之后，每个房间**单独规划**（headland → swaths → route → path），
再按调度顺序拼接成一条全局路径。这样跨房间的转弯不会去穿越墙体。

### 房间调度（TSP）

`MapConnector` 把栅格转成"按机器人半径膨胀后的自由空间图"，
用 Dijkstra 求任意两点间的**可通行距离**。房间之间用这个距离矩阵做
最近邻 + 2-opt，得到访问顺序与总行程：

- 相邻房间的连接路径沿自由空间穿过门洞，不再画穿墙直线。
- 距离机器人当前位置不可达的房间会被标记为 sealed，只输出多边形与状态，
  并在 `warnings` 中说明，不会假装已覆盖。
- 结果字段：`regions[]`（含 polygon、覆盖率、swath 数、顺序）与
  `travel_distance`。

请求里的 `swath_spacing` 直接决定相邻清扫带的中心间距，单位米。
面板 **Swath spacing** 默认为 **0.15 m**，范围 0.05–5.0 m，步进 0.01 m；
服务端收到后覆盖 `operation_width`，即生成端真正使用的带宽。

0.15 m 在 16×16 m 的房间里对应约 0.15 m 的实际行距（实测 16 个间隔全部为
0.150 m），覆盖率从 0.8 m 间距时的 96.3% 提升到 99.2%，代价是行程变长。

### 弓字形掉头

每个房间的清扫带按行铺满整间，行与行之间用**半圆掉头**连接，形态与常见扫地机的弓字形清扫图一致：

- 条带在整间多边形上生成（不再先内缩 1.2 m 的 headland），两端各缩进 0.12 m，
  正好留出半圆掉头的空间，条带端部因此是圆角过渡而不是直角折返。
- 掉头半径取相邻两条带的间距的一半（0.15 m 行距对应 0.075 m 半径），
  弧段按 0.01 m 采样成折线，两个绕行方向都会尝试，取留在房间内的那一条。
- 只有弦长不超过 0.9 m 时才画半圆，避免出现大半径圆弧。跨条带的接缝先做直线
  可行性检查：线段全程落在房间自由空间内就直接连；落在凹多边形缺口里时，在房间
  自由空间上以 0.05 m 八邻域栅格跑 Dijkstra，再做可见性简化，得到沿墙折线，
  因此不会出现穿过墙体的斜线。
- 条带先按最近端点贪心排序，再用 2-opt 消除长距离回跳，保证相邻条带首尾相接；
  否则同一行被拆成左右两段时会出现 4 m 以上的跳转，只能退化成原地转向。
- 房间轮廓填充率不低于 70% 时直接取包围盒矩形，行端保持整齐；低于该值保留追踪出的轮廓。
- 结果里会写一条统计告警，例如 sweep turns: 256 arc, 0 pivot, 2 link，
  分别表示圆弧掉头、因空间不足退化的原地转向、以及跨条带切换（直线或沿墙折线）
  的数量。12 房间 0.15 m 行距的实测结果是 256 / 0 / 2，核心覆盖率 99.8%。


### 房间内不可行时的降级

小房间放不下 headland 带或有界曲率转弯时，按顺序降级：

1. headland 把整个房间吃掉 → 放弃 headland，扫全房间；
2. 生成的转弯弧线越出房间（`ValidPath` 校验失败）→ 改成每段一个状态、
   段间原地转向的回退路径，保证所有位姿都落在房间内。

所有这些降级都会写进 `warnings`，不会静默改变行为。

### 执行

**Clean all rooms** 逐个房间下发 `navigate_complete_coverage`：每个请求只带该房间的多边形，
`generate_regions=false`、`generate_headland=false`，一个房间完成后自动进入下一个，
面板显示当前房间与整体进度。

## 模式与适配范围

- 地头：`CONSTANT`；可关闭地头生成，可选择单圈、同心多圈，以及先/后覆盖地头。
- 扫地半径：`robot_radius`（米）决定条带间距，作业宽度等于半径的两倍；
  配置文件、任务请求和 AutoViz 的 **Robot radius** 均可设置。
  显式 `operation_width` 可覆盖为附件实际扫幅。
- 分解：`NONE`、`TRAPEZOIDAL`、`BOUSTROPHEDON`。忽略沿边界的数值退化射线，保留有效组件；共享边界可预留通行带。
- 条带：`BRUTE_FORCE`、`SET_ANGLE`；目标为 `LENGTH`、`NUMBER`、`NUMBER_MODIFIED`、`COVERAGE`。
- 路线：`BOUSTROPHEDON`、`SNAKE`、`SPIRAL`、`CUSTOM`、`TSP`。`CUSTOM` 要求单区域的完整排列；起终点指定用于 TSP。
- 转弯：`DUBIN`、`REEDS_SHEPP`，配合 `DISCONTINUOUS` / `CONTINUOUS`。
- 预标注行：`CENTER`、`OFFSET`、`ROWSARESWATHS`，支持按 ID 跳过行。

本地 TSP 名称映射到原生 Fields2Cover 的贪心与限时 2-opt 路线算法，不链接 OR-Tools，不承诺全局最优。路径连接使用原始田地和孔洞进行约束验证，并保留有界曲率搜索；不会沿用上游中的无约束方角连接。

默认执行配置拒绝倒车。要执行 Reeds–Shepp，必须同时配置支持倒车的控制器和 `controller_supports_reverse`；仅选择转弯算法不会改变底盘能力。原生 FollowPath 控制器使用其速度限制，路径中的名义速度及工具段信息仍保留。

GPS 规划仅支持本地坐标后端明确识别的 EPSG:4326/4258 与 UTM 转换，执行要求 Cartesian 请求。连续曲率候选无效时，本地库没有连续曲率约束搜索替代，会返回明确错误。实际全车体包络、动态障碍与工具扫掠需要应用层验收。

原生执行把每个覆盖状态按不超过 0.25 m 的纵向间距展开为 transport pose，局部代价地图裁剪始终保留至少两个位姿，因此两点长路径不会退化为单点位姿。差速底盘遇到路径反方向（相差约 180°）时锁定同一转向，避免 atan2 在 ±π 接缝反复切换而造成原地来回摆动。

## 文件职责

每个手写 C++ 类位于独立类文件；生成协议代码与框架要求的回调命名保持兼容。

- `robot_params`、`headland_generator`、`decomp_generator`、`swath_generator`、`route_generator`、`path_generator`：独立阶段。
- `route_method`、`swath_order_method`、`tsp_route_method`、`route_geometry`：上游路线方法与区域拼接。
- `coverage_server`：流水线；`coverage_visualizer`：调试几何；`action_endpoint` / `action_endpoint_state`：原生端点及交接生命周期。
- `native_execution`、`path_segments`、`segment`：执行与无损分段；`coverage_navigator`：任务、BT 与调度；`runtime`：通信组合。
- `bt/`：状态节点、取消节点与工厂注册；`proto/coverage.proto`：原生请求与结果。
- `tests/`：算法、模式、几何、BT，以及真正的端点成功/失败/取消交付测试。

命名采用 Google C++ 规范，注释采用 Doxygen；Protobuf 生成访问器和 BehaviorTree.CPP 覆盖函数保留框架要求的名称。
