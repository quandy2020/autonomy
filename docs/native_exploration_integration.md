# 多传感器探索的原生工程接入

日期：2026-11-17。目标为 Autonomy 原生 C++、Autolink 和 Protobuf；`autonomy/map` 保持零改动。

本文记录真实接入结构。编译及测试状态见 `implementation_validation.md`；接口存在不代表三模式、学习策略或实机任务已经验收。

## 配置与共享图

`PlannerOptions.vista` 由 `vista_planner.proto` 定义，采用有存在性标记的 optional 标量。未设置使用默认值，明确设置零水平/垂直裕度则保留零；容量必须大于零，几何参数须满足 GeometrySafety 的有限性及非负约束。机器人半径或外接半径来自实际 Costmap 足迹，不由 Vista 配置另造一个平台。

`planner_plugins: "vista_planner"` 使用原生别名构造 VistaPlanner。`planner_plugins: "observation_route:VistaPlanner"` 则以另一个实例 id 构造。PlannerServer 的构造路径传入真实 options 与 Costmap，避免通用插件工厂只创建一个未配置对象。

同进程组合可以使用 `PlannerServer::GetPlanner("vista_planner")` 返回的既有插件，经 `VistaPlanner::SharedRoutePlanner()` 注入 VistaPathProvider；本次服务器测试验证这一能力。真实 ExplorationTask 目前尚未接入该提供接口，不能把测试中的共享实例写成已部署行为。

原生规划与任务各有 main 入口。独立进程部署时，task 使用 Autolink 规划客户端，由 PlannerServer 调用它持有的 Vista 实例；不能解引用另一个进程的指针。初版从真实返回 Path 计算距离，批量查询仍在同一图所有者执行。请求与结果须绑定坐标/图版本，不在 task 复制一张独立全局图。

全局路径保存原始终点朝向。取消、错误坐标系、旧 epoch 和异常回调不能发布部分路径。回调在状态锁之外执行，提交前再次核验请求和配置版本。原生 2D Costmap 只提供路线几何；读取不制造新观测时间、上游融合确认或三维净空证明。

## 无环构建依赖

`autonomy/common/navigation` 创建 `autonomy_navigation_support`：包含共享 Opt 几何核心和轻量的 observation/map-reader 实现，依赖标准库与 Threads。公开观测头仍位于 `perception/data`；perception 的递归源文件收集排除已进入此支撑库的实现，防止重复定义。

planning 和 task 链接同一支撑库；task 额外链接 planning。支撑库不链接 task、DeFM 或控制策略，因此在 `AUTONOMY_BUILD_PERCEPTION=OFF` 的原生配置中仍可构建路线和任务接口。Opt 的 `opt::navigation` 暂作为内部几何实现保留，公共头随 include 目录导出；不是一套新的地图服务。

原生 CMake 在 `integration/native/` 中保存对真实工作树基线的合并版本。它用于同步原生工程；Opt 的独立顶层 CMake 和 autonomy/CMakeLists.txt 不覆盖远端入口。同步前逐文件核验原始 SHA256，已有文件备份；不删除未列入清单的文件。

## 探索 BT 与任务结果

探索专用节点沿用现有 bt_node_base.hpp 与单一原生注册入口。Session 是唯一导航提交者：选择和适配不发送运动命令，ObserveAtPose 才提交导航与观测阶段。返航与保存持有各自版本凭据，旧 halt 不得取消后来请求。

原生节点和算法核心进入 task target；不复制或编译第二份原生 BT registry。Condition 读取确定结果，UpdateExploration 做有界消费，StatefulAction 持有阶段凭据。任务结果分别记录探索覆盖、返航和保存，不把 BT 本轮 SUCCESS 自动当作整个任务成功。旧 ExplorationTask 和 TaskServer 的探索 waypoint/finished 路径尚未由 Session/Tree 接管，仍需关闭竞争提交入口并对真实任务生命周期验证。

## 保存与融合协议缺口

当前 MappingCommand 没有真实 SAVE 或融合屏障命令；旧探索 SAVE_MAP 发送 MAP_CMD_SWITCH，不能作为保存证据。MappingClient 的现有 LoadMap 也只是检查名字，并不证明远端地图已载入。这些行为必须接到真实建图服务。

扩展工作限定在 task/automsgs 任务协议与上游定位建图适配，不修改 map 的接口、消息和代码。回执必须对应实际保存动作、建图会话、地图版本和有限来源截面。上游能力尚未接入时，保持明确的未完成/失败结果，不发成功 ACK。

## 编译与验证

在 SpaceHero 使用现有 `/workspace/autonomy/build/autonomy` 配置和库。单独的 `AUTONOMY_BUILD_EXPLORATION_TESTS=ON` 打开新增原生回归，仅注册本次相关专项，不启用所有可选模块的旧测试。

v2.1 在 SpaceHero 编译 planning/task/control 与六个测试 target；CTest 六项 6/6 通过，共 1.93 秒。新增四项覆盖、Session、Phase 和原生 BT 回归，链接真实 task/planning/support 及单一 BT registry；基础覆盖程序含十组检查。回归检查原生快照、接口、门关闭/重开、旧 epoch、并发读取、回调重入，以及真实 PlannerServer 的别名、配置、同进程共享实例和路径输出。

这些测试仍不替代三模式定位/观测、真实融合保存、三模态模型训练、完整扫掠体安全和 Orin 实测。同步后记录文件清单、Docker 日志和 map SHA256 复核，逐项更新验收状态。

本次同步清单、configure/build/CTest 日志和 map 核验均保存在 `docs/validation/2026-11-17-native-*`。map 471 个文件与保护基线一致；完整实施目标保持未完成。Vista 核心已有文件/类/函数/成员 Doxygen，其他新模块及几何镜像的逐项注释仍需补齐。

当前 v2.1 证据见 `validation/2026-11-17-v2.1-native-validation.json`。覆盖收益已按 normalized quality improvement 修复并回归，文件/类/方法/成员 Doxygen 已补齐覆盖状态与融合；其他模块注释和真实任务接入仍待完成。
