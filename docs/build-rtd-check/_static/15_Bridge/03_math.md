(bridge-math)=
# 3. 数学原理

> 协议选型与工程对比见 [08_survey.md](08_survey.md)；gRPC 实现细节见 [06_grpc.md](06_grpc.md)。

### 3.1 桥接问题形式化

设机器人本地导航栈为进程 $\mathcal{R}$，外部客户端集合为 $\mathcal{C} = \{c_1, \ldots, c_n\}$。Bridge 模块求解**异构系统互操作**问题：在保持 $\mathcal{R}$ 内部语义不变的前提下，建立双向映射

$$
\Phi: \mathcal{M}_{\mathrm{ext}} \leftrightarrow \mathcal{M}_{\mathrm{int}}
$$

其中 $\mathcal{M}_{\mathrm{ext}}$ 为外部消息空间（gRPC Protobuf / MQTT JSON），$\mathcal{M}_{\mathrm{int}}$ 为内部消息空间（`commsgs`、Navigator Action 等）。

一次完整交互可建模为四元组：

$$
\langle t, \sigma, \mathbf{p}, \mathbf{r} \rangle
$$

| 符号 | 含义 |
|------|------|
| $t$ | 时间戳（`builtin_interfaces.Time`） |
| $\sigma$ | 指令类型（`NavigationCommand` / `ExplorationCommand`） |
| $\mathbf{p}$ | 参数载荷（位姿数组、多边形区域等） |
| $\mathbf{r}$ | 响应（`success`, `message`, `status`） |

### 3.2 坐标与位姿变换

导航指令 `NAV_CMD_START` 携带 `geometry_msgs.PoseArray`。设第 $k$ 个目标位姿在坐标系 $\{F\}$ 下为

$$
\mathbf{T}_F^{(k)} =
\begin{bmatrix}
R(\theta_k) & \mathbf{t}_k \\
\mathbf{0}^\top & 1
\end{bmatrix}, \quad
\mathbf{t}_k = \begin{bmatrix} x_k \\ y_k \\ 0 \end{bmatrix}
$$

若 Bridge 收到的位姿在地图坐标系 $\{M\}$，而 Navigator 要求全局坐标系 $\{G\}$，需经 TF 链变换：

$$
\mathbf{T}_G^{(k)} = \mathbf{T}_G^M \cdot \mathbf{T}_M^{(k)}
$$

其中 $\mathbf{T}_G^M$ 由 `autonomy/transform` 查询。二维旋转矩阵：

$$
R(\theta) = \begin{bmatrix}
\cos\theta & -\sin\theta & 0 \\
\sin\theta & \cos\theta & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

四元数表示（`geometry_msgs.Quaternion`）与欧拉角关系（绕 $z$ 轴）：

$$
\theta = 2 \arctan2(q_z, q_w), \quad
q = \bigl(0,\, 0,\, \sin\frac{\theta}{2},\, \cos\frac{\theta}{2}\bigr)
$$

### 3.3 Protobuf 序列化与消息大小

Protobuf 3 对字段 $f$ 的线长编码（wire format）近似为：

$$
L(f) = \mathrm{tag}(f) + \mathrm{payload}(f)
$$

变长整数（varint）编码长度上界：

$$
|\mathrm{varint}(v)| \leq \left\lceil \frac{64 - \log_2 v}{7} \right\rceil \quad (v > 0)
$$

`NavigationCommandRequest` 含 $N$ 个位姿时，载荷大小近似：

$$
S_{\text{nav}} \approx S_{\text{fixed}} + N \cdot S_{\text{pose}}
$$

其中 $S_{\text{pose}} \approx 7 \times 8 + 32 \approx 88$ 字节（7 个 `double` 字段 + 帧头开销），$S_{\text{fixed}} \approx 20$ 字节。当前 `GrpcBridgeServer` 设置：

$$
S_{\max} = 100 \times 2^{20} = 104{,}857{,}600 \,\mathrm{bytes}
$$

即单条 gRPC 消息上限 100 MB（`kMaxMessageSize`）。

### 3.4 指令状态机

#### 3.4.1 导航命令 FSM

状态集合 $\mathcal{S}_{\text{nav}} = \{\mathrm{IDLE}, \mathrm{ACTIVE}, \mathrm{PAUSED}\}$，事件为 `NavigationCommand` 枚举：

$$
\delta_{\text{nav}}: \mathcal{S}_{\text{nav}} \times \mathcal{E}_{\text{nav}} \to \mathcal{S}_{\text{nav}}
$$

| 当前状态 | 事件 | 下一状态 | 动作 |
|----------|------|----------|------|
| IDLE | `NAV_CMD_START` | ACTIVE | 下发目标位姿至 Navigator |
| ACTIVE | `NAV_CMD_PAUSE` | PAUSED | 暂停 FollowPath |
| PAUSED | `NAV_CMD_RESUME` | ACTIVE | 恢复 FollowPath |
| ACTIVE/PAUSED | `NAV_CMD_STOP` | IDLE | 清除目标，发布零速 |
| * | `NAV_CMD_CANCEL` | IDLE | 取消当前 Action |

非法转移应返回 `success=false`，不修改内部状态（**fail-safe** 原则）。

#### 3.4.2 探索命令 FSM

状态集合 $\mathcal{S}_{\text{exp}} = \{\mathrm{IDLE}, \mathrm{EXPLORING}, \mathrm{PAUSED}, \mathrm{COMPLETED}\}$，与 `ExplorationStatus` 枚举对齐：

$$
s = \mathrm{EXPLORING} \Leftrightarrow \mathrm{status} = \mathrm{EXPLORING}
$$

与 `ExplorationStatus::EXPLORATION_STATUS_EXPLORING` 枚举对齐。

`EXPLORATION_CMD_SET_AREA` 仅在 $s = \mathrm{IDLE}$ 时接受多边形参数 $\mathcal{P}$：

$$
\mathcal{P} = \{ (x_i, y_i) \}_{i=1}^{n}, \quad n \geq 3
$$

多边形面积（Shoelace 公式，用于合法性校验）：

$$
A(\mathcal{P}) = \frac{1}{2} \left| \sum_{i=1}^{n} (x_i y_{i+1} - x_{i+1} y_i) \right|, \quad x_{n+1}=x_1
$$

要求 $A(\mathcal{P}) > A_{\min}$（避免退化区域）。

### 3.5 gRPC 流式语义

`AutonomyService` 采用 **Server Streaming RPC**：

$$
\mathrm{Client} \xrightarrow{\mathrm{Unary\ Request}} \mathrm{Server} \xrightarrow{\mathrm{Stream}\ r_1, r_2, \ldots} \mathrm{Client}
$$

设响应流 $\{r_i\}_{i=1}^{\infty}$，客户端在时刻 $t$ 的已接收集合为 $\mathcal{R}(t)$。完整性条件：

$$
\forall i < j,\; t(r_i) \leq t(r_j)
$$

即响应按因果顺序到达客户端。

对于 `SendNavigationCommand`，每个中间响应可携带执行进度；最终响应应满足：

$$
r_{\text{final}}.\mathrm{success} =
\mathbf{1}_{\{\mathrm{Navigator\ accepted\ the\ command}\}}
$$

`async_grpc` 框架在 **Completion Queue** 上调度事件，事件到达率 $\lambda$ 与处理率 $\mu$ 需满足稳定性条件（M/M/1 近似）：

$$
\rho = \frac{\lambda}{\mu} < 1
$$

其中 $\mu \propto N_{\text{grpc}} + N_{\text{event}}$（`num_grpc_threads` + `num_event_threads`）。

### 3.6 MQTT QoS 与延迟模型（规划）

MQTT 服务质量等级定义消息传递语义：

| QoS | 语义 | 投递保证 | 典型延迟 |
|-----|------|----------|----------|
| 0 | At most once | 无确认，可能丢失 | 最低 |
| 1 | At least once | 确认后可能重复 | 中等 |
| 2 | Exactly once | 四步握手，无重复 | 最高 |

端到端延迟近似：

$$
T_{\text{e2e}} = T_{\text{pub}} + T_{\text{broker}} + T_{\text{sub}} + T_{\text{bridge}} + T_{\text{nav}}
$$

其中 $T_{\text{bridge}}$ 为 JSON/Protobuf 解析与 FSM 处理时间，$T_{\text{nav}}$ 为 Navigator 响应时间。弱网场景下选择 QoS 1 可在可靠性与延迟间折中。

### 3.7 时间同步

请求中的 `timestamp` 字段使用 ROS 风格时间：

$$
t = \mathrm{sec} + 10^{-9} \cdot \mathrm{nanosec}
$$

客户端—服务器时钟偏差 $\Delta t = t_{\text{client}} - t_{\text{server}}$。若 $|\Delta t| > \tau_{\max}$（建议 $\tau_{\max} = 500\,\mathrm{ms}$），Bridge 应拒绝指令或打警告日志，防止重放攻击与乱序执行。

### 3.8 吞吐与并发

设并发客户端数为 $n$，每客户端平均请求率为 $\lambda_c$，则总负载：

$$
\Lambda = n \cdot \lambda_c
$$

gRPC HTTP/2 多路复用下单连接可承载多 RPC 流，理论流数上限受 `SETTINGS_MAX_CONCURRENT_STREAMS` 约束（默认无限制，受内存与线程池限制）。推荐配置：

$$
N_{\text{grpc}} \geq \lceil \Lambda / \mu_{\text{rpc}} \rceil, \quad
N_{\text{event}} \geq N_{\text{grpc}}
$$

当前默认 $N_{\text{grpc}} = N_{\text{event}} = 4$（代码硬编码），`bridge.lua` 建议值为 5。

### 3.9 数据流概览

$$
\mathrm{External\ Client}
\xrightarrow{\mathrm{gRPC/MQTT}}
\mathrm{Bridge\ Handler}
\xrightarrow{\mathrm{FSM + TF}}
\mathrm{Navigator / Explorer}
\xrightarrow{\mathrm{state\ callback}}
\mathrm{Bridge\ upstream\ stream}
\xrightarrow{\mathrm{Stream}}
\mathrm{External\ Client}
$$
