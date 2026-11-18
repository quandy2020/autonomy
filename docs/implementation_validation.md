# 多传感器探索实现与验证记录

设计依据：`multisensor_exploration_design.md` v2.10 最终方案审查稿与 `multisensor_exploration_review.md` 总览（owner 十三类隔离验证；BT 十一类隔离验证；正式原生九项 CTest）。当前实施目标包括实现、测试和 Docker 编译。以下将文件存在、算法验证、原生集成和任务实验分别记录。文中按日期/版本排列的早期状态保留为历史；当前证据边界以最新记录为准。

工作区：Opt 保存可回放的原生代码及测试，`autonomy/` 下按目标工程布局组织。远端 autonomy 是集成目标。禁止改动 `autonomy/map`。远端既有未提交改动以工作区内容为基线。

## 基线

- 2026-11-17：Opt C++17 + BehaviorTree.CPP 本地编译成功。
- 2026-11-17：远端 `SpaceHero` Docker 内 `autonomy_planning`、`autonomy_task` 编译成功。
- 地图目录 SHA256 基线保存于 `/tmp/opt-autonomy-reference/map_baseline.json`，集成结束复核。

## 要求追踪

| 要求 | 实现位置 | 必须证明 | 状态 |
|---|---|---|---|
| 三种模式、标定、单位、时间、降级 | perception/data、observation | 单模式、联合、迟到/重复/无效输入 | Opt 基础实现及测试用例已有；原生观测接入待完成 |
| map 零改动、不可变快照、滚动、epoch | perception/data/map_reader、vista/native_costmap_reader | 内容版本、长时覆盖、回环失效 | Opt 核心与真实 Costmap 桥已测试；原生融合确认及三维执行证据接入待完善 |
| FAR 增量可见图 | planning/planner/vista | 长边失效、移障重连、稳定 ID、栅格备用 | 核心、真实桥、别名/配置及同进程共享已在 SpaceHero 专项通过；跨进程探索接入待完成 |
| TARE 分层覆盖与访问排序 | exploration/strategy、coverage、routing | 边际收益、低收益扫尾、公平重访 | 基础代码已有；质量正改善公式已在 v2.1 修复并回归，公平扫尾与完整任务待验证 |
| Opt 保持观测目标意图 | exploration/integration/goal_pose_adapter | 调整后重验可见性/必要覆盖 | 代码已有；专项测试及原生任务接入待完成 |
| 原生 BT 与任务生命周期 | exploration/plugins、公共 bt_node_base | 实时监测、取消、暂停、单提交者 | 八个节点与独立阶段已进入 v2.1 原生构建及专项回归；真实 ExplorationTask 接管与服务接入待完成 |
| 实际观测确认与完成协议 | coverage/completion_evaluator、exploration_session | 各传感器截面、健康、稳定轮、返航/保存分离 | Opt 评估核心及会话初版已有；上游同步、真实返航成本、阶段恢复及收尾闭环待验证 |
| 冻结 DeFM 与三模态条件 | perception/inference/defm、训练工具 | 官方预处理、缓存、掩码、独立模式验证 | 待实现 |
| B 样条扩散与连续执行 | policy_controller/sand_planner | 时间、位置/速度/加速度边界、动态限制 | 待实现 |
| 独立扫掠体安全与过期保护 | trajectory_validator、tracker | 未知拒绝、动态障碍、细分裕度、截止时间 | 待实现 |
| Docker 原生集成 | integration/native、远端实际业务工作树 | 实际规划/任务/控制目标及相关测试 | v2.1 原生 planning/task/control 与六个测试程序构建通过，相关 CTest 6/6；完整任务/学习执行未验收 |
| 端侧任务实验与学习模型效果 | 回放/仿真/实机记录 | SR/SPL、覆盖、效率、jerk、延迟 | 待验证 |

编译和确定性测试通过不能替代学习策略训练、仿真任务或 Orin 实测。记录各项真实证据，不将接口骨架称作算法已完成。

## 2026-11-17 审查材料更新

- 已重新只读核对远端规划目录、探索入口、GlobalPlanner::CreatePlan、原生 BT 基类与地图拷贝接口；HEAD 仍为 def0e9aa。
- 最终审查稿补充了逐单元执行证据年龄、分传感器 watermark、候选截断不得完成、BT 返回语义与 26 类情况。
- 当前测试文件 test/multisensor_exploration_test.cc 已存在。本轮审查整理未重新运行算法测试或业务编译，不能据此更新为全套通过。
- /tmp/opt-exploration-work/Testing/Temporary/LastTest.log 当前记录 opt_goal_bt_extension_demo 于 2026-11-17 17:54 CST 通过；这是单个既有目标适配示例的记录，不是完整探索验收。
- docs 中的 HTML 及 canvas 提供审查视图，视图检查不属于机器人运行验证。

## 2026-11-17 v1.2 最终审查更新

- 重新只读核对远端 planning/planner、ExplorationTask 和 bt_node_base；原生任务已有导航与建图提交入口。探索专用节点目录改为 exploration/plugins/action、condition，沿用 navigation/plugins 的现有职责分组。
- Opt 已新增 ExplorationSession 初版及分传感器 ObservationCut、分页与取消相关改动。新增核心代码尚无本次完整编译和回归证据，旧通过记录不能覆盖这些改动。
- 审查稿新增三模态共享头训练契约、按来源质量及多视角记账、制动空间、真实返航路线预算、完成撤销与阶段恢复要求。
- mapping_synchronized 的真实来源、表面提取与 required_views 兑现、分页公平性、Session 生命周期、Vista 注册与八个探索 BT 节点继续列为实施缺口。
- 本次只整理审查材料及状态，不构建业务代码、不修改远端业务文件，也不修改 autonomy/map。
- 审查材料静态检查通过：HTML 20 节目录锚点全部存在，26 类情况完整保留，无外部脚本或样式依赖；Canvas 仅导入 cursor/canvas。未进行浏览器渲染或完整 TypeScript 检查，这些检查也不属于机器人任务验证。

## 2026-11-17 v1.3 核对与会话回归

- 最新 opt_exploration、opt_multisensor_exploration_tests、opt_exploration_session_tests 本地编译通过。
- ctest 的两项探索测试均通过，总耗时 1.50 秒。基础程序包含 8 组检查，新会话程序包含 2 组场景；这不是整套任务、原生插件或学习控制验收。
- 会话修复：移动目标对象前保存观测方向数量；地图变化时保留仍有效的观测目标和阶段；拒绝旧 epoch/旧时间位姿；暂停恢复后重新累计未完成方向的观测停留；显式链接 Threads::Threads。
- 新会话测试使用真实 VistaPathProvider、GoalPoseAdapter 和 CameraVisibilityModel，以及可控导航回调。验证到达不算完成、多朝向实际观测、地图变化期间保留有效目标、旧 epoch 不覆盖运动证据、暂停不能兑现停留要求。
- 最新只读 SSH 核对 HEAD 为 def0e9aa，autonomy/map 共 471 个文件与本任务 SHA256 基线一致；本轮未向远端业务目录写入文件。
- Vista 原生继承/注册、八个探索 BT 节点、异常回调/超时/收尾闭环、DeFM/SanD 三模态训练和新增代码 Docker 编译仍待完成。
- 审查页已由 v1.3 Markdown 同步生成，20 个章节锚点和 26 类情况保留，没有外部脚本/样式依赖。Canvas 按当前本地 SDK 做 TypeScript 静态检查，0 个诊断；修复了 Row.justify 的枚举值。未做浏览器渲染和机器人任务运行验证。

- 随后执行现有本地全量 CTest：13/13 通过，总耗时 2.82 秒。包含既有导航、目标适配 BT、回放及两个探索测试程序；不包含尚未接入的八个探索专用 BT 节点或原生 Docker 验证。

## 2026-11-17 v1.4 最终方案审查

- 本轮交付最终设计和审查视图；未增加业务代码、未重新运行 CTest 或业务构建。实施目标仍包括原生集成、测试及新增代码 Docker 编译，先前通过记录不延伸为新要求通过。
- 固定三模式覆盖政策与任务要求版本，补充离散位置/方向扫尾、分阶段 BT 请求及取消所有权、真实结点导数边界、训练/部署版本契约、保存阶段完成撤销和 CompletionEvidence。
- Markdown、HTML 与 Canvas 同步到 v1.4。静态检查通过：20 节导航锚点、26 类情况、8 个节点全部保留，HTML 无外部脚本/样式依赖；收益与边界公式转换为可直接阅读的文本。
- Canvas 使用当前 cursor/canvas SDK 做严格 TypeScript 检查，0 个诊断。没有浏览器渲染或机器人任务运行证据；该项仅证明静态审查材料有效。
- 本轮通过只读 SSH 再次核对远端 HEAD def0e9aa；autonomy/map 471 个文件与 SHA256 基线一致，新增、删除、内容变化均为 0。
- FAR、开发环境论文、DeFM 与 SanD 的 arXiv 标题/元数据重新核对。2110.14573 仍按开发环境与算法集成文献引用，原 TARE 使用 RSS 2021 论文及官方实现。
- 未完成项保持不变：原生 Vista 注册、八个探索 BT 节点及任务接入、上游融合确认、完整会话异常/超时验证、DeFM/SanD 三模态训练、新增代码 Docker 集成和端侧任务验收。

## 2026-11-17 v1.5 实际代码与原生桥验证

- 共享 MapReader/MapReadView 移到 perception/data，task 的原入口保留兼容别名；Vista 不再依赖 task 的地图契约。修复迁移后测试缺少显式 include 的编译回归。
- VistaRoutePlanner 拒绝旧 epoch 回滚；本地增加该回归检查。修复局部覆盖选择与访问排序中 GCC 的 misleading-indentation 错误，没有放宽 -Werror。
- 本地最新探索核心和两项探索测试重新编译。全量 CTest 13/13 通过，总耗时 0.41 秒；日志为 validation/2026-11-17-local-ctest.log。这 13 项仍不包含尚未接入的八个探索专用 BT 节点。
- 新增 VistaPlanner 继承真实 GlobalPlanner，使用真实 Protobuf；NativeCostmapReader 锁内复制原生 Costmap，锁外做版本维护。缺少真实观测/融合/高度证据时保持 source_stamp=0、mapping_synchronized=false、has_height_evidence=false。
- Docker 容器 SpaceHero 中隔离源码为 /workspace/autonomy/build/opt-vista-review-haab6yp8/source，构建目录为同级 build。开启 OPT_BUILD_NATIVE_VISTA，引用真实 /workspace/autonomy/src/autonomy 头文件及 /workspace/autonomy/build/autonomy 生成头和库；OPT_BUILD_BEHAVIOR_TREE=OFF。
- opt_native_vista 与 opt_native_vista_tests 已在 Docker 编译、链接通过；该 CTest 1/1 通过，耗时 0.12 秒。程序包含不可变快照、真实全局规划接口/共享图、门关闭/重开与旧 epoch、原生插件工厂、地图并发读取五组检查。日志为 validation/2026-11-17-native-vista.log。
- 原生 SDK 配置未导出 Autolink target，桥接构建解析真实库/头，不制造替代实现；Automsgs/Autolink 选择同一原生 build 的库，显式传递 Eigen 与 glog 依赖，消除安装库与 build 库的混用。
- 原生 Costmap 的尺寸构造函数只有声明、没有定义；测试使用真实默认构造和 resizeMap 初始化，没有补写或改动 map 源码。
- 远端只读复核 HEAD def0e9aa；autonomy/map 471 个文件与 SHA256 基线一致，新增、删除、变化均为 0。源码只复制到 Docker build 下的隔离目录，没有向远端业务工作树写入文件。
- Docker 内再核对受保护 map 同样为 471 文件、零变化；隔离副本的 165 个代码/构建文件与当前 Opt SHA256 一致。ldd 确认 Automsgs、Autolink 和 Autonomy 库均来自真实原生 build。清单见 validation/2026-11-17-native-vista-manifest.json，五组 PASS 输出见 validation/2026-11-17-native-vista-test.log。
- 审查材料更新到 v1.5：20 节、26 类情况、8 个节点契约保留；HTML 锚点有效，无外部脚本/样式依赖；Canvas 严格 TypeScript 检查 0 个诊断。未做浏览器渲染或实机运行验证。

仍待完成：planner_server 的 vista_planner 别名/配置构造接入与接入后的 planning/task/control 原生目标重编；真正分阶段的八个探索 BT 节点；完整 Session 取消/超时/重启/保存 ACK 验证；原生观测与上游融合/保存确认；DeFM/SanD 三模式训练及执行适配、三维 Safety 与端侧任务验收。当前原生桥测试不是上述全套验收。

## 2026-11-17 最终审查补充

- 核对当前 Opt 源码与上述已保存日志，补充设计稿的六项默认决策及第 20.3 节七项具体修复要求。
- 源码核对发现：稳定确认轮没有要求有效来源截面推进；SAVING 分支早于 PoseReady 检查；Vista 持状态锁调用外部地图/取消回调；同 epoch 重启没有显式策略重置；工作线程与析构取消回调的异常保护不足；单次 Tick 整批消费输入；Session 尚无独立 BT 阶段 API。
- 这些是源码层面的缺口，尚未做新增故障注入或运行回归。本轮没有修改业务代码、构建业务目标、重跑 CTest 或写入远端工作树。
- 官方 arXiv 元数据及 FAR/TARE README 经网络重新读取，文献角色与 DeFM 2026-10-27 的 v3 修订记录吻合；未将论文的性能数字移用为本系统指标。
- Markdown、离线 HTML 与现有审查画布同步。HTML 静态检查通过：20 个章节锚点、26 类情况、8 个节点契约与 7 项修复要求保留，公式转换为可读文本，无外部脚本/样式依赖；修正目录的 aside 布局，使其使用已有侧栏样式。
- Canvas 使用当前 cursor/canvas SDK 做严格 TypeScript 检查，0 个诊断；只导入 cursor/canvas，无网络调用。三个交付文件存在且非空。未检查浏览器实际渲染，材料静态检查不等于算法、Docker 或机器人任务验收。

## 2026-11-17 v1.6 最终方案与新增故障回归

- 新增有效观测截面与所需来源检查，禁止重复旧帧/重复 tick 推进完成确认；单 watermark 兼容路径也要求推进。
- 保存期间复核健康与定位，ACK 提交时核验更晚输入与版本；定位过期不保留完整探索成功，新工作使旧请求失效。有效部分地图保存独立记录。
- 同 epoch 新会话在工作线程重置策略；工作线程、拥有者服务及析构取消路径捕获非标准异常，取消失败阻止额外隐式运动。
- 事件/观测有单轮预算，命令和取消优先；accepted、processed、discarded 截面分开，过期丢弃也计入处理名额。
- VistaRoutePlanner 和 VistaPlanner 改为锁外执行外部回调及搜索，提交时复核配置/请求/图版本；NativeCostmapReader 串行读取。新增第六组 native_reentrant_callbacks 及 30 秒原生测试超时；这些最新原生改动尚未 Docker 编译或运行。
- 两个最新本地探索目标重新编译通过，专项 CTest 2/2 通过，总耗时 0.60 秒。基础程序包含九组检查，会话程序包含六组场景。日志：validation/2026-11-17-exploration-fault-regression.log。
- 修正两处测试场景：bounded drain 输入把第三实参明确作为时间戳、单独设置 sequence；保存用例给出已观测房间任务边界，避免地图外未知区域被错误当作已覆盖。初次新增会话测试失败后完成修正并重跑，不能把这次回归写成零问题。
- 本轮未重跑全量 CTest；旧 13/13 和旧 Docker 五组结果仅适用于原先版本。没有同步远端业务代码、修改原生构建入口或训练学习模型。
- 设计稿和审查视图更新到 v1.6，新增原生构建依赖边界，修正第 20.3 节已修复项与未完成项。当前优先工作仍为独立 BT 阶段和八个真实节点、Vista 配置接入及正式原生构建。
- 论文元数据与官方仓库 README 重新读取，来源记录见 validation/2026-11-17-review-sources.json；没有将上游性能数值作为本系统指标。
- 本轮只读 SSH 复核远端 HEAD def0e9aa；autonomy/map 471 个文件与基线相比新增、删除、变化均为 0。清单见 validation/2026-11-17-final-review-map-check.json。
- v1.6 审查材料静态检查通过：20 个章节链接、26 类情况、8 个 BT 契约均保留；离线 HTML 无外部脚本/样式依赖；Canvas 按当前 SDK 严格 TypeScript 检查 0 个诊断。三个交付文件存在且非空，未做浏览器渲染验证。记录见 validation/2026-11-17-final-review-artifacts.json。

仍需独立验收：真实观测/定位、上游融合与保存协议、完整表面覆盖与公平扫尾、三模态 DeFM/SanD 训练、时间样条及三维 Safety、任务回放/仿真和端侧实时性。实施目标保持未完成。

## 2026-11-17 v1.7 最终审查与当前源码状态

- 本轮交付最终方案，核对本地阶段接口、测试文件与远端目录；不修改业务代码，不运行 build/CTest，不向远端业务目录写入代码。
- 当前 ExplorationSession 已写入 RequestPhase/PhaseState/CancelPhase、PhaseTicket、自动/BT 互斥驱动与 Select/Adapt/Observe/Return/Save 阶段；test/exploration_phase_test.cc 已有六组阶段测试代码。
- 上述最新阶段与测试修改尚无编译/运行证据；此前探索 2/2、全量 13/13 和 Docker Vista 五组通过记录只说明对应旧版本。不能把当前接口存在改标为当前系统已通过。
- 本地尚无八个探索专用原生节点；远端仍只有原探索入口，Vista 原生配置接入、任务同步与完整学习/安全执行链尚未完成。
- 最终审查稿补充第 17.1 节八项默认设计政策，并区分 FINISHING 收尾候选、探索完成、返航到达与保存确认。
- 本轮只读 SSH 确认远端 HEAD def0e9aa；autonomy/map 共 471 文件，SHA256 与原基线相比新增、删除、内容变化均为 0。记录：validation/2026-11-17-v1.7-map-check.json。
- Markdown、离线 HTML 和 Canvas 按同一 v1.7 基线交付；材料静态检查与业务/模型/端侧验收分别记录。
- 本轮材料静态检查通过：20 个章节及目录锚点、26 类情况、8 个 BT 契约保留；3 组公式在 HTML 中转换为可读文本；离线页无外部脚本/样式/网络资源，Canvas 只导入 cursor/canvas 且无网络调用。文件存在、大小与 SHA256 记录在 validation/2026-11-17-v1.7-review-artifacts.json。本轮未运行 TypeScript 完整类型检查或浏览器渲染检查。


## 2026-11-17 v1.8 最终审查与原生 BT 本地验证

- 审查文档、离线 HTML 和交互画布统一到 v1.8；保留 20 个章节、26 类情况与八个节点契约，新增 §12.3 当前原生 XML 和根状态/任务结果的边界。
- 修正 plugin_utils 的 Blackboard const 访问与 nodiscard 编译问题；四个相关本地目标重新编译。八个探索节点使用真实 Autonomy BtCondition/BtSyncAction/BtStatefulAction 与原生注册源，未使用替代基类。
- 首次专项 CTest 为 3/4：前五组原生节点场景通过，整树用例失败。用例原先把第二视角设为测试相机视野之外，并在动作开始前观测了目标；同时把新帧与每个 BT tick 强制同步。修正为视野内的两个不同朝向、动作前朝其他方向观测，以及独立采集频率。保留两次实际观测、显式返航、保存 ACK 和恰好三个导航提交的要求。
- 修正后四项相关 CTest 4/4 通过，耗时 0.19 秒。包含多传感器基础程序、会话程序、六组独立阶段场景与六组原生 BT 场景。构建及回归日志：validation/2026-11-17-v1.8-local-bt-build.log、validation/2026-11-17-v1.8-local-bt-tests.log。
- 上述导航/建图服务使用注入回执；不证明正式 ExplorationTask、planner_server、真实融合/保存服务或三模态学习执行链已接入。持续流入的双来源输入、全部阶段暂停恢复及任务级故障组合仍需验证。完整目标保持未完成。
- 只读 SSH 复核远端 HEAD def0e9aa40c4d43063a68fbd1d8516fecce93cdd，autonomy/map 471 个文件与基线相比新增、删除、内容变化均为 0。结果：validation/2026-11-17-v1.8-map-check.json。本轮没有同步正式业务代码或进行 SpaceHero 全工程构建。
- 六个官方论文/仓库来源经网络重读，元数据记录：validation/2026-11-17-v1.8-review-sources.json。论文/仓库性能未转作本系统承诺。
- 审查材料静态验证通过：20 个导航锚点、26 类情况、八个 BT 契约、三组公式与原生 XML，离线页无外部脚本/样式/网络资源，画布只导入 cursor/canvas。
- 画布源码依据公开 SDK 做严格 TypeScript 检查，0 个源码诊断；SDK 缺少 internal/canvas-action-dispatch.js 声明导致最初库检查失败，因此最终对宿主声明使用 skipLibCheck，未修改或伪造 SDK。此项不等于完整 SDK 或浏览器运行验证。日志：validation/2026-11-17-v1.8-canvas-typecheck.log。
- 文件存在、大小、SHA256 及检查边界记录：validation/2026-11-17-v1.8-review-artifacts.json。未检查浏览器实际渲染。

## 2026-11-17 v2.0 真实同步、原生构建与最终审查

- 115 个文件已经通过基线 SHA256 检查、备份后逐文件同步至真实远端业务工作树，不整目录覆盖或删除用户文件。首次清单为 validation/2026-11-17-native-sync-manifest.json；后续同步须以上次部署 SHA256 检查冲突。
- SpaceHero 原生 configure 和 build exit=0，autonomy_planning、autonomy_task、autonomy_control 与两个 Vista 测试程序编译成功。支撑 target 位于 common/navigation，保留无环依赖及原生可选模块设置。
- CTest autonomy.native.vista 和 autonomy.planning.vista_server 2/2 通过，分别 0.15/1.37 秒，共 1.52 秒。六组 native 场景包含最新回调重入检查；服务器测试验证真实 optional 配置、别名/id、同进程实例、Path 与终点朝向。
- 完整 configure/build/CTest 日志保存为 validation/2026-11-17-native-configure.log、2026-11-17-native-build.log、2026-11-17-native-tests.log；汇总为 validation/2026-11-17-v2.0-native-validation.json。不与本地 4/4 或旧隔离测试合并为全套验收。
- 构建后只读 SSH 检查 map 471 文件，新增、删除、内容变化均为 0，HEAD 为 def0e9aa40c4d43063a68fbd1d8516fecce93cdd；证据为 validation/2026-11-17-native-map-check.json。
- 真实 ExplorationTask 与 TaskServer 仍保留探索 waypoint 提交；旧 finished 消息可直接结束任务，SAVE_MAP 仍转发 MAP_CMD_SWITCH。八个节点进入 task 编译不等于真实任务已由 Session/Tree 驱动。
- 规划与任务各有独立 main 入口；GetPlanner 只证明同进程共享。最终方案规定 task 通过原生 Autolink 路径入口复用 PlannerServer 的图；跨进程提供接口尚未接入。
- 当前 CoverageFusion::MarginalGain 仍为旧乘法规则；v2.0 的质量正改善公式待实现。有限观测截面、真实融合/保存、三模态训练、三维执行链、完整 Doxygen 和任务/Orin 实验仍待完成。
- 本轮更新审查 Markdown、HTML 与 Canvas，核验此前正在运行的原生构建结果并执行两项 CTest，未再新增或同步业务代码。完整实施目标仍为 active，未标记完成。

## 2026-11-17 v2.1 质量改善与原生六项专项

CoverageFusion 的旧乘法算法已被回归复现并修复。原始质量保持 [0,1]；评分按 required_quality 归一化并饱和，相对实际与已选最大基线取正改善。ANY_SENSOR 按稳定单元去重，PER_SENSOR 按来源计算并平均，不以 LiDAR 预测填补实际 RGB-D 需求。预测不写入真实覆盖账本；非法类别、来源、质量或预选缓存拒绝整批处理。补充覆盖模块 Doxygen。

本地四项覆盖/Session/Phase/BT CTest 4/4 通过，共 2.81 秒。新增四个真实原生测试程序后，SpaceHero 的六项 Vista、PlannerServer、覆盖、Session、Phase、BT CTest 6/6 通过，共 1.93 秒。编译通过 planning/task/control 与六个测试 target；服务回执仍注入，不能据此宣称真实任务已接入。

同步以上次部署清单 SHA256 为 expected，120 项中更新 10 项，保留用户文件并备份已有内容。map 471 文件再次核验零变化。证据见 validation/2026-11-17-v2.1-native-validation.json 及同前缀 build/tests/map-check/sync-manifest 文件。完整目标未完成；不将阶段测试、BT SUCCESS 或提交保存请求当作完整探索与保存成功。

## 2026-11-17 v2.2 最终方案审查

本次更新方案、离线 HTML 及 Canvas，补齐真实 ExplorationTask 接管、TaskServer 旧入口收敛、版本化导航关联与 shutdown 约束；修正要求追踪表中质量收益和六项专项的过期状态。未新增业务代码、远端同步、业务 build、CTest 或机器人运行记录。

只读 SSH 核验：远端 HEAD 7b251c5e5b5a2deeace5b7558a750d6917bac23f；六个任务/规划入口与此前快照一致，最近部署清单 120 个文件均与其 SHA256 一致。map 471 文件与保护基线相比零新增/删除/内容变化。核验覆盖上述文件，不覆盖其余工作树；此前构建日志保持其对应版本的范围。

四篇 arXiv 页面和 FAR/TARE README 重新读取成功；当前来源记录为 validation/2026-11-17-v2.2-review-sources.json。远端与地图检查见同前缀 native-check 和 map-and-deployment-check JSON；审查材料检查见 artifacts JSON。真实任务接入、有限截面、融合/保存、三模态训练、三维执行与任务/Orin 验收仍待完成，完整目标保持 active。

## 2026-11-17 v2.3 最终方案审查与新本地接管原型

本轮核对实际 Opt 文件、v2.1 构建/测试记录及 v2.2 远端核验资料，更新最终方案、离线页和审查画布。此前续作已写入 ExplorationTask/TaskServer 所有权与入口收敛、TypedTask 版本化结果、NavigationTask 按 ID 取消及进程内唯一 Session ID；尚未构建、测试或同步。不把旧 4/4、6/6 延伸为这些代码通过。

源码审查明确取消后晚启动、旧 waiting_control 泄漏、thread 访问/关闭同步、StopSession 异常清理、延迟队列溢出、idle-only setter 与同步导航派发问题。第 12.5 节新增身份/运动许可、异步派发与原子交接、owner 清理、证据入队统计及跨重启请求身份契约。新增第 20.8 节标明当前本地能力、阻断项和实施优先级。

未修改业务代码、运行业务 build/CTest、写入远端或修改 map。材料静态验证另行记录；未做浏览器渲染或机器人运行。完整实施目标保持 active。

## 2026-11-17 v2.4 最终方案审查及最新源文件核验

补齐父探索/委托导航的调度、原生接受与取消责任交接、先停提交者再关闭导航依赖的契约。MotionGate/NavigationDispatcher 两个本地文件尚未接入 NavigationServices，未构建、测试或同步。源码存在 Submit 返回后锁外读取 permit 与锁内公布 accepted 之间的漏取消窗口；此次只报告发现，未进行业务修复或竞态测试。

只读 SSH 确认远端 HEAD bdfd9f0b8a2949f5b7fae08c5f5ec8e5c5243498，SpaceHero running。autonomy/map 471 文件与原保护基线零新增/删除/变化。检查的七个既有任务入口中，task_server.hpp/.cpp 不同于 v2.2 快照，须保留用户改动并合并后同步；其余五个入口保持快照内容。两个派发器文件远端不存在。证据见 validation/2026-11-17-v2.4-review-native-check.json。

四篇 arXiv 元数据及 FAR/TARE 官方 README 重新 HTTP 读取均为 200，锁定正文来源、字节数与 SHA256；未据此声明模型或性能完成。审查稿保留 20 节、26 类情况、8 节点、3 组公式，交互页提供 30 项审阅标记。HTML 与画布静态/类型检查另记 validation/2026-11-17-v2.4-review-artifacts.json；不等同浏览器渲染。

本次无业务 build/CTest、业务同步或机器人验证；v2.1 本地 4/4、Docker 6/6 仅覆盖其原版本。真实任务、观测/定位、跨进程路线、融合/保存、有限来源截面、公平扫尾、三模态训练、时间样条/3D Safety 及端侧任务仍需完成。实施目标保持 active。

## 2026-11-17 v2.5 派发器模块回归与最终总览

- 最新 NavigationDispatcher 与八类测试重新 configure、编译并运行 opt_navigation_dispatcher_tests，1/1 通过，测试时间 1.50 秒，包含 5000 次接受/取消责任交接。
- 修复提交返回后的同锁责任交接、取消去重和锁外调用；接受/结果异常按 ID 取消；Shutdown 永久 retire，回调重入不等待外部 join；取消无法确认可通过 Failed() 读取并禁止该代新运动。
- 该模块尚未接入真实 ExplorationTask、未同步远端、未进行新代码 Docker 编译。不要将本地 1/1 与旧原生 6/6 相加为完整任务验收。
- 旧实现交接回归退出失败的日志已保留；早期 CV 用例未复现，atomic 接受事件用例复现漏取消。当前源码通过与旧源码失败分别记载。
- 本轮只读远端核验 HEAD bdfd9f0b8a2949f5b7fae08c5f5ec8e5c5243498；SpaceHero running；map 471 文件零新增/删除/内容变化。TaskServer 两文件存在用户新修改，后续需逐文件合并，不覆盖。
- 新增 multisensor_exploration_review.md 审查总览，完整 Markdown、离线 HTML 与 canvas 更新至 v2.5，明确三模式、增量图、收益、Opt、BT、完成和执行的设计/证据边界。
- 日志和指纹：validation/2026-11-17-v2.5-dispatcher-local.log、2026-11-17-v2.5-dispatcher-local.json、2026-11-17-v2.5-review-native-check.json、2026-11-17-v2.5-review-sources.json。审查视图结构和 TypeScript 检查另记，不作为浏览器或机器人运行证明。

完整实现目标仍未完成：任务接管与父子调度、真实观测/定位及跨进程路线、有限完成截面和真实融合/保存、覆盖补全/公平扫尾、三模态训练与连续执行、仿真/实机/Orin 验收继续待完成。

## 2026-11-17 v2.5 后续派发器部署及原生七项回归

五项白名单文件已按 expected SHA256 核验、备份并同步。同步清单为 validation/2026-11-17-v2.5-dispatcher-sync-manifest.json。SpaceHero 中 autonomy_planning、autonomy_task、autonomy_control 与七个相关测试程序 configure/build 均成功；Vista、PlannerServer、multisensor、Session、Phase、native BT、NavigationDispatcher 七项 CTest 7/7 通过，总耗时 2.00 秒。证据为 validation/2026-11-17-v2.5-dispatcher-native-validation.json 及同前缀 configure/build/tests 日志。map 471 文件零新增/删除/内容变化。

该记录覆盖已部署派发器与相关原生回归，未包含随后本地 ExplorationTask/TaskServer 接入稿。编译日志有测试 lambda 隐式捕获 this 的 C++20 弃用警告；没有把它描述为零警告构建。旧 v2.5 审查段落的“尚未同步”是当时历史状态，当前以本段为准。

## 2026-11-17 v2.6 最终方案审查材料

本次更新总览、详细设计、离线 HTML 和画布，纠正派发器部署/原生验证的过期说明，明确新 ExplorationTask 的异步服务、运动许可和关闭处理已写入本地但尚未验证。不得将五项已部署文件或七项相关测试通过延伸为新 owner、guarded acceptance、pre-tick 门控和父子调度通过。

本次重新计算五项本地源文件与三份原生日志的 SHA256，均匹配该 v2.5 验证记录；核对主来源 HTTP 200 与内容指纹。补充消费者可扩展概率收益但当前采用质量改善代理的边界，以及 ObservationGoal、TimedTrajectory、CompletionEvidence 等目标接口语义。保留所有历史验证记录。

本次无业务代码修改、业务 build/CTest、远端同步或远端当前工作树检查。静态审查页与画布检查见 validation/2026-11-17-v2.6-review-artifacts.json；浏览器渲染与机器人运行未验证。真实接入、外层有限完成截面、融合/保存、三模式训练和任务级验收仍待完成，完整目标保持 active。

## 2026-11-17 v2.7 当前审查与只读核验

本次再次读取八项原始 HTTP 来源，全部 200，保存 DeFM/SanD v1 全文及来源指纹。只读 SSH 确认远端 HEAD bdfd9f0b8a2949f5b7fae08c5f5ec8e5c5243498、SpaceHero running；map 471 文件与保护基线完全一致。五项派发器文件的本地与远端指纹、三份原生 configure/build/CTest 日志均匹配 v2.5 保存记录。未重新构建或执行测试，旧 7/7 仍仅覆盖其原有范围。

新的 exploration_owner_test.cpp 草稿已存在但未编译、运行或同步。源码核验发现其将策略 Reset 当成 Session::Start 并要求它在 task owner 执行的计数假设需要修正；当前 Reset 实际在 Session 规划工作线程调用。该草稿不作为通过证据。本地 ExplorationTask 的初始化失败运行占用清理、建图准备前暂停后的恢复路径仍有源码发现；未宣称受控故障已复现或已修复。第 12.7、20.12 节补充契约和验证边界。

更新总览、详细设计、离线 HTML 与 Canvas 至 v2.7，保持 20 节、26 类情况、八个核心节点、30 项审阅清单。来源、远端/地图与静态检查分别为 validation/2026-11-17-v2.7-review-sources.json、2026-11-17-v2.7-review-native-check.json、2026-11-17-v2.7-map-and-deployment-check.json、2026-11-17-v2.7-review-artifacts.json。浏览器渲染未验证；审阅标记不表示验收通过。

本次无业务代码修改、业务 build/CTest、远端同步或机器人运行。完整实施目标保持 active，当前未验收能力以详细设计 §20.12 为准。

## 2026-11-18 v2.8 owner 故障复现、修复及最终审查

- 修正 owner 测试的证据：使用真实 Session 身份确认 Start；构造与释放线程由不延长 Session 所有权的注册表核验。策略 Reset 位于规划工作线程，不作为 task owner 或启动计数。
- baseline 6/9 通过，实际复现建图准备跳过及两条初始化失败清理问题；baseline-start-gate 6/11 通过，进一步复现 final-clock 期间取消/暂停仍晚 Start。旧失败日志保留。
- 修复按 generation 补齐建图准备、统一异常清理、最终 Start 许可交接、树推进前最新意图复核，fixed-initialization 11/11 通过。
- 扩展暂停预算与监测，fixed-owner-monitor 12/13 通过，定位真实待覆盖计数发布滞后；Session Publish 从账本刷新 pending_units 后，最终 13/13 通过。
- SpaceHero 独立 staging 编译新 exploration.cpp、exploration_session.cpp 与 exploration_owner_test.cpp，链接真实原生共享库，全部步骤返回 0。正式 checkout 未修改；未纳入正式全目标构建或 CTest。已有 task_interface.hpp:89 的 extra semicolon 警告仍在。
- 记录：validation/2026-11-18-v2.8-owner-final-owner.json 与同前缀 compile/link/owner-scenarios 日志。当前来源 SHA256 与各步骤日志 SHA256 在该记录中，不合并旧 7/7 与新 13/13 为全任务通过。
- 新增 docs/exploration_task_owner.md，更新最终方案入口、详细设计、离线审查页与交互审阅标记，保留历史记录。正式部署、NavigationTask 原生门控、父子调度及真实任务链继续待完成。

完整实施目标保持 active，最新当前能力与缺口见详细设计 §20.13。审查材料静态/类型检查独立记录，不能作为浏览器、机器人或任务完成验证。

## 2026-11-18 v2.9 正式 owner 部署、原生验证与最终审查

- 已完成的正式部署：六项白名单经过 expected SHA256 检查、原内容备份、逐文件同步。记录为 [同步清单](validation/2026-11-18-v2.8-owner-sync-manifest.json)，备份为 `/tmp/opt-owner-deploy-v2.8-86ef9ffc183b/backup`。
- 已完成的指定构建：SpaceHero 正式 checkout 配置成功，planning/task/control 与八个测试目标编译成功。八项正式 CTest **8/8 通过，2.13 秒**，包含新的 owner 测试。见 [原生验证](validation/2026-11-18-v2.8-owner-native-validation.json) 与 [CTest 日志](validation/2026-11-18-v2.8-owner-native-tests.log)。不是全目标构建、全工作树或机器人任务通过。
- 证据修复：第一次启动器 JSON 转义错误发生在 CTest 之前。其错误日志被重试覆盖，不能恢复原始内容；原路径与摘要及缺失原因保留在验证 JSON 的 historical_launcher_failures。原 manifest 另有备份。成功 configure/build/CTest 三份日志 SHA256 都已重新匹配。未为修复元数据而重复业务构建或测试。
- 部署后只读核验：[当前源文件记录](validation/2026-11-18-v2.9-review-native-postdeploy-check.json) 确认六项部署内容匹配指纹、map 471 文件与保护基线一致、TaskServer 用户修改保持原状、SpaceHero running。当前业务源文件未再次修改。
- 审查材料更新：总览、完整设计、owner 说明、离线页与 Canvas 更新为 v2.9，保留历史记录，纠正“owner 尚未部署”状态。26 类情况、八节点、30 项审阅检查及全部方案待验收项保留。文档更新的指纹独立于原部署指纹。
- 文献核对：[八项官方 HTTP 来源](validation/2026-11-18-v2.9-review-sources.json) 均返回 200，包含 DeFM/SanD v1 原文；联网工具未提供来源编号，未编造网页引用。来源刷新不验证预训练权重、模型推理或性能。
- 本轮不重复业务 build/CTest 或远端同步。审查页静态检查与 Canvas TypeScript 检查另记于 [审查产物记录](validation/2026-11-18-v2.9-review-artifacts.json)；浏览器渲染与机器人运行未验证。
- 剩余实施：真实 NavigationTask 许可、父子运动调度、依赖关闭；三模式原始生产者和独立定位；跨进程 Vista；公平扫尾/外层有限前缀及真实融合保存；任意多边形/部分覆盖/checkpoint；三模态训练、物理时间样条/三维 Safety/跟踪；回放、仿真、实机及 Orin 验收。完整目标保持未完成。

## 2026-11-18 v2.10 原生 BT 执行前检查、正式同步与回归

- BtRunnerHooks::BeforeTick 与 BtTaskApp::BeforeTreeTick 形成真实执行前入口；拒绝后不 tick 节点，返回后重新检查暂停/取消。默认入口保留兼容行为。
- 修复长周期取消等待；暂停/恢复在等待 mutex 下递增唤醒代数，避免等待边界丢通知。worker 回调 Cancel 不 self-join；Configure/Run 拒绝重入；旧 host 不在活跃 worker 中替换。
- setup、检查、监测、日志及 halt 的标准/非标准异常保留 FAILED，取消不掩盖故障。树及 logger 清理后才发布 IsRunning=false。回调必须有界，不能宣称通知可打断任意阻塞接口。
- 隔离 native 使用真实原生头、库、XML、Script/StatefulAction 及 typed host，十一类场景通过。详见 [隔离记录](validation/2026-11-18-v2.10-bt-stage.json) 和 [生命周期说明](native_bt_admission.md)。没有替换 runner 或伪造节点效果。
- 六项白名单文件在 expected SHA256 校验及备份后同步；[同步清单](validation/2026-11-18-v2.10-bt-sync-manifest.json)。map 471 个文件未改，TaskServer 内容保持原样。备份路径单独保留。
- SpaceHero 正式 checkout 配置、planning/task/control 及九个专项程序构建成功；相关 CTest **9/9 通过，2.28 秒**。见 [正式验证](validation/2026-11-18-v2.10-bt-native-validation.json)、[构建日志](validation/2026-11-18-v2.10-bt-native-build.log)、[CTest 日志](validation/2026-11-18-v2.10-bt-native-tests.log)。已有 RPATH/头文件警告仍在，没有新出现的编译失败。
- 十一类隔离验证不与九项 CTest 累加；指定目标通过不证明全工作树、运动派发许可或机器人任务通过。新增虚函数后的 native task ABI 已在正式目标重编；没有启动或热替换机器人进程。
- 实际 dispatch 许可、父子调度、三模式真实生产者/独立定位、跨进程路线、融合保存/有限截面/公平扫尾、三模态训练、时间轨迹/3D Safety/跟踪及整机实验仍待完成。当前完整实施目标仍为 active。

## 2026-11-18 v2.11 typed-task 接受保护、正式回归与最终设计审查

- 四项白名单文件已同步：typed_task.hpp、typed_task_test.cpp、ExplorationTests.cmake、typed_task_admission.md。expected SHA256 核验、备份与部署指纹见 [同步清单](validation/2026-11-18-v2.11-typed-sync-manifest.json)。
- 拒绝目标保留旧身份，真正接受后提交新身份；pending 阻止结果读取；接受票据阻止迟到旧请求覆盖新身份；异常释放 pending；Shutdown 使旧票据失效。领域通过 ShouldUpdateGoalIdentity 分类 START/REPLAN 与控制请求。
- 九类隔离原生场景通过，见 [专项记录](validation/2026-11-18-v2.11-typed-stage.json)。其测试领域已覆盖匿名与控制分类，真实 NavigationTask 尚未覆盖，不将测试分类视为生产导航已接入。
- 收回前轮运行中的正式验证：configure/build/CTest 返回码均为 0，planning/task/control 与十个指定测试程序构建成功，相关 CTest **10/10 通过，2.22 秒**。见 [原生记录](validation/2026-11-18-v2.11-typed-native-validation.json) 和 [CTest 日志](validation/2026-11-18-v2.11-typed-native-tests.log)。不为更新元数据重复业务构建。
- 只读部署核验见 [当前核验](validation/2026-11-18-v2.11-review-native-postdeploy-check.json)：四项部署指纹匹配，map 471 文件与保护基线一致，TaskServer 用户修改未变，SpaceHero running。日志指纹与各返回码在审查产物记录中复核。
- 新增 [最终设计审查稿](multisensor_exploration_final_plan.md)，更新总览、详细设计、离线页与交互清单。方案区分目标接口与当前代码，完整性限定任务分辨率与候选覆盖前提；Opt 保留观测要求，平滑要求物理时间与跨周期衔接。
- 剩余工作：生产导航身份/readiness/实际 dispatch 许可、父子调度和依赖关闭；三模式生产者及独立定位、跨进程 Vista、完整公平扫尾与有限截面、真实融合保存、多边形/checkpoint、三模态学习、时间样条/3D Safety/跟踪、整机与 Orin。完整实施目标仍未完成。


## 2026-11-18 v2.12 NavigationTask 完成、正式部署及 v2.13 审查更新

- 六项白名单：navigation.hpp/.cpp、navigation_task_test.cpp、bt_runner.hpp、ExplorationTests.cmake 和 navigation_task_admission.md。expected SHA256、原内容备份和逐文件同步见 [清单](validation/2026-11-18-v2.12-navigation-sync-manifest.json)；原备份为 `/tmp/opt-navigation-deploy-v2.12-7f3a086c1c19/backup`。
- NavigationTask 已完成身份分类、完整命名重试、匿名 START、可撤销 readiness、pending/active/tree/motion epoch、定向取消、即时终态和 worker 重入保护。XML 文件在接受前检查，XML 内容/节点仍在运行时验证。
- 真正 native 对象和执行链的 21 类隔离场景全部通过，见 [隔离记录](validation/2026-11-18-v2.12-navigation-stage.json) 与 [场景日志](validation/2026-11-18-v2.12-navigation-native-navigation-scenarios.log)。readiness、截止时间和取消依赖受控，不作为远端真实取消验证。
- SpaceHero 正式配置、planning/task/control 与十一个相关测试程序构建成功，相关 CTest **11/11 通过，2.67 秒**；新导航程序 0.41 秒。构建日志确认 navigation.cpp 与 task_server.cpp 消费者重编。[正式验证](validation/2026-11-18-v2.12-navigation-native-validation.json) 绑定三个成功日志及摘要。已有警告仍在，不等于整工作树或整机通过。
- 已收回部署后的核验结果：[部署核验](validation/2026-11-18-v2.12-navigation-postdeploy-check.json)。六项部署指纹匹配；map 471 文件不变；TaskServer 两文件用户内容保持；SpaceHero running。没有重复启动部署或构建。
- v2.13 更新最终方案、审查入口、详细设计、导航说明、离线页和 Canvas；保留历史验证，纠正旧 NavigationTask 草稿状态。文档和静态检查的证据单独记录，不将浏览器渲染标记为通过。
- 剩余首要缺口：实际 ActionSession 派发许可、父探索委托调度、晚接受取消与远端取消确认、依赖关闭。领域 tick 资格和异步 CancelActiveMotion 返回不构成物理停止 ACK。
- 真实三模式生产者/定位、跨进程路线、公平扫尾/有限截面、真实融合保存、多边形/checkpoint、三模态训练、时间样条/3D Safety/跟踪和机器人/Orin 验收仍未完成。完整实施目标保持 active。


## 2026-11-18 v2.15 最终方案审查与本地派发状态复核

最终设计与接口、三模式、增量图、收益、Opt、完成协议、平滑和验收阶段集中到 [最终审查稿](multisensor_exploration_final_plan.md)。当前正式业务构建/CTest 仍为 v2.12 的 11/11；本地 v2.15 派发门控已接入，隔离原生编译和原有导航 21 类通过，十一项源码与四步日志已复核。新增派发专项、真实消费者重编及业务部署未完成，不能以原有场景证明新增竞态正确。

远端十七个业务文件、map 471 文件和 TaskServer 用户修改在本轮只读核验中保持基线；八项官方来源重新获得 HTTP 200。记录分别见 [源码与日志复核](validation/2026-11-18-v2.15-review-stage-check.json)、[远端快照](validation/2026-11-18-v2.15-review-native-check.json)、[来源](validation/2026-11-18-v2.15-review-sources.json)。本轮未运行新的业务构建/CTest，机器人及三模式全任务仍未验证。
