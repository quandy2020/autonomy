# 08 · Navigation 验收矩阵

> 当前验收环境：2026-11-09，Linux SpaceHero。构建命令、日志与限制见
> [16-linux-validation.md](16-linux-validation.md)、[18-autosim-validation.md](18-autosim-validation.md) 与 [19](19-behavior-recovery-validation.md)，剩余工作见 [13-todo.md](13-todo.md)。
> ✅ 表示指定范围有实测证据；🟡 表示部分完成；⛔ 表示缺外部制品。

## 1. 当前模块验证

| 范围 | 验证内容 | 证据 |
| --- | --- | --- |
| common | pose/twist 转换、平台限幅、配置解析、线程与缓冲区 | 相应 `*_test` |
| perception | log-depth 预处理、encoder、Observation validity | `depth_preprocessor_test`、`defm_encoder_test`、`observation_builder_test` |
| memory | 关键帧、tokens、空间索引、快照、漂移校正 | memory 目录测试 |
| planning | chunk blending、场景、规则、优化、恢复原语 | planning 目录测试 |
| VNTPA 接口 | query/context、离散动作与 postprocess | `query_builder_test`、`action_decoder_test`、`postprocessor_test`、`vntpa_policy_test` |
| safety | TTC、CBF、AEB、可行性、看门狗 | `safety_shield_implementation_test` 等 |
| depth freshness | 仿真 epoch、重复缓存 stamp、超时与重启 | `depth_freshness_test` |
| behavior 默认树 | 内建注册、实际 XML、RecoveryNode/PipelineSequence、持久预算与请求校验 | `nodes_test`、`bt_cpp_engine_test`、`recovery_maneuver_test`、`runner_test` |
| guardian 核心 | 心跳、分类、降级、模块聚合 | `heartbeat_monitor_test`、`supervisor_implementation_test` 等 |
| Runner | 异步推理、取消/抢占、旧结果失效、到达停车 | `runner_test`、`runner_create_test` |
| 配置 | 三主配置、四平台与两份模型 manifest 解析 | `configuration_validator` |
| DeFM 制品 | hash、名字/dtype/形状、错误输入和失败加载 | `model_integrity_test`、`navigation_defm_validator` |
| 组件链路 | 五组件、三 DAG、Action、TwistStamped、断流和心跳 | `component_integration`；范围见下文 |
| 真实多进程 | Habitat 传感器/执行器、Python Action、障碍/时钟/抢占/恢复/进程退出 | 九项 `tools/run_autosim_acceptance.sh`；[19](19-behavior-recovery-validation.md) |

测试名以 `ctest --test-dir build/autonomy/autonomy/navigation -N` 的实际注册列表为准。
测试存在不表示论文模型或全部任务树已验证；默认恢复树的真实证据见 [19](19-behavior-recovery-validation.md)。

## 2. 组件集成范围

集成测试在独立 IPC/挂载命名空间和独立 `/dev/shm`、`/tmp` 中运行真实 Autolink 定时器/传输，
解析三个 DAG 后直接创建五个具体组件。传感器与差速运动来自测试夹具。
该范围包含：

- depth-only 输入时 raw 持续为零，可选 camera/odom 缺省与融合 validity；
- 三心跳进入 NORMAL；
- topic→Action、Action→Action、Action→topic 抢占，旧 Action 返回 CANCELLED；
- current_pose/navigation_time/distance_remaining feedback、计时重置与点 `(2,1)`；
- Cancel 后导航原始输出归零、不重放缓存 topic；Safety 减速按加速度/jerk 限幅；
- Runner 回归覆盖取消/抢占不消费旧异步推理结果；
- 探索前进、合成近障停车、移除恢复；
- depth 停发超过 200 ms 停车，零速仍发布 TwistStamped，frame=`base_link`、header 使用 depth 时间；
- 组件停用后的 Guardian 降级和 Behavior 重建恢复；等待范围包含 500 ms 检查周期及接收租约。

组件夹具不覆盖 mainboard 动态加载、真实独立进程 kill、autosim 场景碰撞与长期运行。
独立 Habitat 脚本已验证 mainboard 加载、多进程 Action、kill/重建与每步几何余量；
机器人采用运动学 set_pose，物理接触碰撞响应和长期运行仍待验收。
BehaviorComponent 已执行默认恢复 XML，真实自主绕行、恢复取消/重建和耗尽终态见 [19](19-behavior-recovery-validation.md)。

## 3. 闭环分级

| 等级 | 状态 | 当前证据及缺口 |
| --- | --- | --- |
| L0 构建/单测 | ✅ | umbrella colcon 构建、导航 CTest、配置校验与安装资源检查 |
| L1 传感器→执行器 | ✅ | Habitat RGB-D/odom/IMU→Observation、Safety→真实 autosim reader/plant |
| L2 点到点 | ✅ | 单场景 `(2,1)` 8.9384 s、误差 0.04349 m、无几何穿透 |
| L3 避障 | 🟡 | Habitat 单静态障碍停车/移除恢复、几何余量 0.18329 m；物理碰撞和更多场景待测 |
| L4 恢复 | 🟡 | 默认 XML 保留静态障碍自主绕行、取消/重建和墙体耗尽通过；复杂/动态场景与物理接触待测 |
| L5 探索 | 🟡 | 真实 initial_goal 前进已验证；长期/map frontier 闭环待测 |
| L6 抢占 | ✅ | 真实 Python/C++ Action/topic 抢占、cancel、计时重置及缓存不复活 |
| L7 守护 | ✅ | 三类进程 kill、Guardian 降级、实际 plant 停车和重建 NORMAL |
| L8 VNTPA | ⛔ | DeFM Engine 可运行；planner 权重缺失 |
| L9 任务 | 🟡 | 任务原语可单测；following/coverage/door/elevator/docking 未接完整组件任务 |

**九项真实脚本检查已通过；L1–L7 尚未全部完成，不能称为完整 autosim 闭环验收。**

## 4. 模型与工程证据

- DeFM CPU C++ Engine 与 Python ORT 1.23.2 的三组输入最大绝对误差为 0，满足 `<1e-4`。
- PyTorch/ORT 的 ramp tokens 最大误差为 3.5154819e-4；跨框架阈值为 5e-4，
  具体记录见 [06-models.md](06-models.md)。
- 当前实际构建为 C++20。Autolink 头文件仍产生 unused parameter、SHM 模板等 warning；
  未开启全仓 `-Werror`，不能宣称 0 warning。
- ASan/TSan 本轮未运行，不沿用旧 macOS/独立工程记录。
- 性能门限与实时硬截止未在目标硬件上测量，Watchdog 单测不等于时延验收。

## 5. 外部阻塞

- VNTPA planner checkpoint/ONNX 未提供，manifest 哈希仍是占位。
- 完整 autosim 验收仍需长期/map frontier 探索、更多障碍和物理接触响应。
- SR/SPL、coverage gate、K 扩展和真实部署入口仍待接入。

BehaviorTree.CPP、autonomy_common、Autolink 与 DeFM ORT 后端本轮已实际构建，
不再列为当前 Linux 环境缺失依赖。
