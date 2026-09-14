# OrbisView 分层迁移设计

**日期：** 2026-09-13

**状态：** 已确认

**范围：** 将 Apollo Dreamview 与 Dreamview Plus 的能力分层迁移到 `autonomy/orbisview/`，形成基于 CMake、Autolink RT 和 `automsgs` 的原生 Web 可视化与工程交互平台。

## 1. 背景

Apollo Dreamview 是基于 Cyber RT、Apollo protobuf、Bazel、C++ Web 后端和 Web 前端构建的自动驾驶 HMI。其后端聚合定位、底盘、感知、预测、规划、路由、地图、相机及点云等数据，并将其转换为浏览器可消费的二维或三维世界状态。

Dreamview Plus 在此基础上增加通道级数据流、可组合面板、资源管理、数据录制与回放、路由编辑、PNC 分析、插件以及更完整的前端基础设施。

目标工程使用 CMake、Autolink RT 和 `automsgs`，并已包含以下可视化能力：

- `autonomy/visualization/`：Foxglove 协议桥接；
- `autoviz/`：原生桌面可视化应用；
- 外部 RViz2、Foxglove 等工具的桥接能力。

因此，迁移不得直接把 Apollo 的 Cyber RT 和 Bazel 构建体系引入主工程，也不得替换现有可视化模块。

## 2. 目标

新建 `autonomy/orbisview/`，形成面向机器人系统的统一空间可视化、运行观测和工程交互平台。

OrbisView 应支持：

- 导航路径、规划轨迹和机器人运动状态；
- 定位结果、里程计和 TF 坐标关系；
- 二维地图、三维地图和自主探索过程；
- 激光扫描、点云、相机图像和深度图；
- 机器人模型、感知对象和 Marker；
- 通道发现、实时订阅、频率限制和连接状态；
- 可组合面板、布局保存和后续插件扩展；
- 数据录制、回放、诊断和运行分析的后续扩展。

## 3. 非目标

首期不包含：

- Apollo 云端资源下载；
- Apollo 车辆配置管理；
- V2X 资源管理；
- Apollo 专用 ScenarioSet；
- RTK Recorder 与 RTK Player；
- Teleop；
- Apollo 动态车辆模型；
- 完整 PNC 专用分析面板；
- 对 `autoviz` 或 `autonomy/visualization` 的重构；
- 使 Apollo 原始 Bazel 目标参与 autonomy 默认构建。

## 4. 总体架构

```text
Autonomy Modules
  │
  │ Autolink channels / automsgs
  ▼
adapters
  │  消息发现、订阅、转换、时间与坐标统一
  ▼
core
  │  统一世界模型、会话状态、数据源与协议
  ├──────────────► backend
  │                 WebSocket、流控制、回放、资源管理
  │
  └──────────────► rendering
                    地图、点云、轨迹、机器人及覆盖层
                          │
                          ▼
                       frontend
                    面板、布局、交互与状态管理
                          │
                          ▼
                       plugins
                    数据源、面板与工具扩展
```

## 5. 目录结构

顶层与 Apollo Dreamview 对齐（`backend` / `frontend` / `proto` / `conf` / `launch` / `main.cc`），用 CMake 替代 Bazel `BUILD`；源文件扩展名统一为 `.cc` / `.h`。

```text
autonomy/orbisview/
├── CMakeLists.txt
├── README.md
├── main.cc
├── backend/
│   ├── CMakeLists.txt
│   ├── orbisview.cc / orbisview.h     # 对标 dreamview.cc（类名 Orbisview）
│   ├── common/
│   │   ├── orbisview_gflags.cc / .h
│   │   ├── handlers/
│   │   ├── streaming/
│   │   ├── plugins/
│   │   ├── util/                      # placeholder
│   │   ├── map_service/
│   │   ├── vehicle_manager/
│   │   ├── teleop/
│   │   └── sim_control_manager/
│   ├── hmi/                           # hmi.cc / hmi.h
│   ├── simulation_world/              # simulation_world_service.*
│   ├── point_cloud/                   # placeholder
│   ├── teleop/
│   ├── perception_camera_updater/
│   ├── record_player/                 # 对标 DV+ record_player
│   ├── adapters/                      # mock / autolink / automsgs（OV 特有）
│   └── testdata/
├── frontend/
│   ├── assets/
│   ├── config/
│   ├── proto_bundle/
│   ├── setup.sh / gen_pbjs.sh
│   └── src/
│       ├── app.tsx                    # 对标 Dreamview app.js
│       ├── components/                # Orbisview.tsx + panels
│       ├── store/                     # zustand + websocket/
│       ├── renderer/
│       ├── styles/
│       ├── utils/
│       └── fonts/
├── proto/
├── conf/
├── launch/                            # orbisview.launch + dev.sh
└── thirdparty/
    └── civetweb/                      # 传输栈（进入构建）
```

### 5.1 `proto/`

仅存放 `.proto` 定义（对标 Dreamview `proto/`）：`stream_envelope`、`render`、`plugin_config`、`record`、`point_cloud`。C++ 运行时 envelope / schema 常量在 `backend/common/`（`stream_envelope.*`、`render_schemas.h`）；当前 WS 仍以 JSON 传输，proto 为契约与后续 codegen 来源。

### 5.2 `backend/`

提供 HTTP/WebSocket 服务、HMI、仿真世界聚合、适配器、节流、录制回放与插件宿主。子目录命名对齐 Dreamview（`common` / `hmi` / `simulation_world`）。后端不得在 Autolink 回调线程中执行阻塞式网络发送。

### 5.3 `frontend/`

以 Dreamview Plus 为主要能力参考，原生实现面板系统、布局管理、状态管理和工程交互界面。地图 / 点云 / 机器人等渲染在前端实现。

### 5.4 `conf/` / `launch/`

`conf/` 存放 gflags 与软 HMI 模式描述；`launch/` 存放一键启动脚本（对标 Dreamview `launch/`，不使用 Cyber）。

### 5.5 `thirdparty/`

`civetweb/` 进入 OrbisView 构建。Apollo Dreamview / Dreamview Plus **不**在本树 vendoring；需要对照时查上游 Apollo 仓库。

## 6. 与现有可视化模块的关系

| 模块 | 职责 | 本次处理 |
|---|---|---|
| `autonomy/visualization` | Foxglove 数据桥接 | 保持不变 |
| `autoviz` | 原生桌面可视化 | 保持不变 |
| `autonomy/orbisview` | Web 可视化、运行观测和工程交互 | 新建 |

三个模块可以共享 Autolink RT 和 `automsgs`，但不存在包含关系。OrbisView 不复制已有 Foxglove 转换器或 AutoViz 显示插件；需要共享的通用能力应通过稳定接口复用，而不是跨目录引用实现细节。

## 7. 上游参考策略

- Dreamview / Dreamview Plus 的目录与能力作为外部参考，**不** vendoring 源码；
- OrbisView 原生实现保持独立协议（CivetWeb + JSON envelope）；
- CivetWeb 许可证见 `thirdparty/civetweb/LICENSE.md`；
- 前端依赖由 `frontend/package.json` 解析，安装产物不得提交。

## 8. 首期功能范围

首期建立可独立运行、可持续扩展的最小主链路：

1. Autolink 通道发现、订阅与取消订阅；
2. `automsgs` 消息类型识别；
3. WebSocket 实时数据流；
4. 二维占据栅格地图；
5. 三维点云；
6. 图像与深度图；
7. TF 坐标树；
8. 机器人模型与实时位姿；
9. 路径、轨迹、里程计和定位结果；
10. Marker 与 MarkerArray；
11. 激光扫描；
12. 基础传感器状态；
13. 可组合面板和布局保存；
14. 数据频率、延迟和连接状态显示。

## 9. 内部数据协议

前端不得直接依赖全部 `automsgs` 具体类型。后端使用统一信封描述数据：

```text
StreamEnvelope
├── channel
├── schema
├── timestamp
├── frame_id
├── sequence
├── encoding
└── payload
```

字段语义：

- `channel`：数据来源通道；
- `schema`：载荷类型及版本；
- `timestamp`：数据产生时间，不使用发送时间隐式替代；
- `frame_id`：空间数据所属坐标系；
- `sequence`：流内顺序及丢帧统计依据；
- `encoding`：protobuf、JSON、图像编码或其他明确格式；
- `payload`：实际数据载荷。

消息分为两类：

- **通用流**：保留 protobuf 二进制载荷及 schema，由通用面板解析；
- **渲染流**：地图、点云、图像、轨迹和机器人状态转换为稳定的 OrbisView 渲染协议。

这一边界允许后续接入 ROS 2、MCAP 或其他通信系统，而不改变前端面板协议。

## 10. 数据流与并发

```text
Autolink Channel
      │
      ▼
Channel Discovery
      │
      ▼
automsgs Adapter
      │
      ▼
StreamEnvelope / World Model
      │
      ▼
Throttle and WebSocket Streaming
      │
      ▼
Frontend Store
      │
      ├── 2D/3D Rendering
      ├── Image Panel
      ├── Data Panel
      └── Diagnostics Panel
```

每个订阅支持独立频率限制、队列上限和丢帧统计。实时可视化采用最新数据优先策略。慢速客户端不得阻塞 Autolink 回调线程；网络发送和序列化必须进入有界异步队列。

## 11. 异常处理

- 无法识别的消息类型保留原始载荷，并标记为 `unsupported`；
- 单个适配器失败不得终止后端服务；
- WebSocket 客户端过慢时丢弃旧帧，保留最新帧并累计丢帧计数；
- 数据超时后可以保留最后状态，但必须显示 `stale` 标记；
- TF 缺失时停止对应空间对象更新，不使用隐式单位矩阵；
- 无效点云、图像或地图不得进入渲染线程；
- 前端插件加载失败不得影响基础面板；
- 上游 Dreamview 代码默认隔离，避免 Apollo 依赖污染主构建。

## 12. 迁移阶段

### 阶段一：上游归档与工程骨架

- 建立 OrbisView 目录；
- 引入两个上游源码快照；
- 记录来源版本和许可证；
- 建立各层 CMake 边界；
- 确保 `thirdparty/` 默认不参与构建；
- 建立前端独立构建入口。

### 阶段二：最小数据链路

- 实现 Autolink 通道发现；
- 定义 `StreamEnvelope`；
- 建立 WebSocket 服务；
- 实现订阅、取消订阅、节流和连接管理；
- 提供通道列表与基础状态接口；
- 使用模拟数据验证后端到浏览器的数据链路。

### 阶段三：基础可视化

- 迁移 Dreamview Plus 面板和布局框架；
- 接入地图、TF、机器人模型、路径和定位；
- 接入 PointCloud2、LaserScan、Image 和 Depth；
- 实现二维与三维视图；
- 支持图层开关、视角控制和配置持久化。

### 阶段四：工程分析能力

- 消息检查面板；
- 通道频率和延迟统计；
- 日志与诊断面板；
- 数据录制和回放；
- 插件注册机制；
- 自主探索、导航和建图专用面板。

### 阶段五–八：DV+ 能力对等（OrbisView 原生）

- WorldModel / Chassis / Obstacles + Dashboard；Autolink `cmd_vel` 发布钩子；
- react-mosaic 可拖拽布局与地面车 preset；
- RoutePath / VectorMap / 软 HMI 模式与 Components（无 cyber_launch）；
- Prediction / Charts / PlanningDebug 深 PNC；mock + Autolink converter 双轨。

详见 `autonomy/orbisview/README.md` 阶段表。

## 13. 验证策略

- `backend/common/`：envelope / schema 常量与节流等单元测试；
- `proto/`：`.proto` 契约（stream / render / record / plugin）；
- `backend/adapters/`：每类 `automsgs` 消息转换测试；
- `backend/`：连接、订阅、节流、断线重连和慢客户端测试；
- `frontend/`：面板注册、布局恢复和数据状态测试；
- `backend/common/plugins/`：注册冲突、加载失败及接口版本测试；
- 集成测试：Autolink 发布模拟数据，浏览器验证接收与渲染状态；
- 许可证检查：确认上游文件和前端资源的授权信息完整。

验收记录必须区分静态检查、构建测试、自动化测试和实际浏览器运行结果，不得以源码存在代替功能通过。

## 14. 完成条件

阶段一完成条件：

- 目录层次与本设计一致；
- 上游来源、提交版本和许可证可追溯；
- `thirdparty/` 不进入默认构建；
- OrbisView CMake 骨架可配置；
- 前端依赖清单完整且不包含安装产物。

首期主链路完成条件：

- 浏览器能够发现 Autolink 通道；
- 浏览器能够订阅和取消订阅；
- 地图、TF、机器人位姿、路径、点云和图像至少各有一个可验证数据源；
- 慢客户端不会阻塞通信回调；
- 断线重连、数据过期和不支持类型具有明确状态；
- 自动化测试与实际浏览器验证结果分别记录。

## 15. 风险与控制

| 风险 | 控制措施 |
|---|---|
| Apollo 与 autonomy 运行时不兼容 | 用 `backend/adapters/` 隔离，禁止核心层依赖 Cyber RT |
| 两代 Dreamview 重复代码过多 | Dreamview Plus 作为主线，Dreamview 仅作参考 |
| 上游快照显著增加仓库体积 | 排除构建产物和依赖缓存，记录精确来源版本 |
| 前端协议与 `automsgs` 强耦合 | 使用 `StreamEnvelope` 和稳定渲染协议 |
| 高频点云或图像阻塞通信线程 | 有界队列、节流和最新帧优先 |
| 缺失 TF 导致错误空间显示 | 拒绝更新并显式报告坐标变换缺失 |
| 许可证来源不清 | 版权头 + `thirdparty/civetweb/LICENSE.md` / ORIGIN |
| 与现有可视化功能重叠 | 保持模块边界，禁止首期重构现有模块 |

## 16. 源码分析边界

本设计依据以下源码的静态检查形成：

- `/Users/quandy/Workspace/github/apollo/modules/dreamview/`；
- `/Users/quandy/Workspace/github/apollo/modules/dreamview_plus/`；
- `autonomy/visualization/`；
- `autoviz/`；
- autonomy 顶层 CMake 配置。

本阶段未执行 Apollo Dreamview、Dreamview Plus 或 OrbisView 的构建和运行验证。
