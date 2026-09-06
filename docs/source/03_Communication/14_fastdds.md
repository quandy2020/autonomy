# 14. 可选 FastDDS（跨机 RTPS）

跨机 Channel 为**可选**能力：默认仍为本机 INTRA + SHM；打开 Fast DDS 后可用 RTPS 数据面。

| 本文 §14 | 相关文档 |
|---------|----------|
| 启用、双机、Discovery Server、升 3.x | [§0 指南](00_guide.md) · [§1 架构](01_architecture.md) · [§3 Channel](03_channel.md) |

---

## 14.1 编译启用

```bash
cmake -S autolink -B autolink/build-fastdds \
  -DAUTOLINK_BUILD_TEST=ON \
  -DAUTOLINK_ENABLE_FASTDDS=ON
cmake --build autolink/build-fastdds -j
```

- CMake option：`AUTOLINK_ENABLE_FASTDDS`（默认 **OFF**）。
- 优先 `find_package(fastdds|fastrtps 2.14)`；未安装则 FetchContent 钉 **v2.14.6**。若前缀路径上探测到 Fast DDS **major≥3**（或 `find_package` 直接命中 ≥3），CMake 发出 WARNING（已验证基线为 2.14；3.x 未测，默认仍钉 2.14 FetchContent）。
- 打开后库目标定义 `AUTOLINK_ENABLE_FASTDDS=1`，编译 `transport/rtps/` 与 Hybrid/Transport 的 RTPS 分支。

验证：

```bash
ctest --test-dir autolink/build-fastdds \
  -R 'rtps_transceiver|topology_backend_factory|rtps_topology_backend|payload_limit|rtps_stats' -V
```

说明：同进程 RTPS PubSub 在部分平台上可能只完成匹配、不触发 `DataReaderListener`；双机清单仍是正式验收路径。`autolink/conf/fastdds_profiles.xml` 将 intraprocess 设为 `OFF`，优先走 UDP。

---

## 14.2 环境变量

| 变量 | 默认 | 含义 |
|------|------|------|
| `AUTOLINK_ENABLE_FASTDDS` | CMake OFF | 编译开关（非运行时） |
| `AUTOLINK_TOPOLOGY_BACKEND` | `local` | `rtps` → 跨机 ChangeMsg；失败回退 local |
| `AUTOLINK_IP` | 本机探测 | Participant 宣告地址 |
| `AUTOLINK_DOMAIN_ID` | `80` | DDS Domain |
| `AUTOLINK_DISCOVERY_SERVER` | 空（SIMPLE） | `ip:port[,ip:port]` → CLIENT 发现 |
| `AUTOLINK_RTPS_MAX_PAYLOAD_BYTES` | `4194304`（4 MiB） | Underlay payload 软限；`0` = 不检查 |
| `AUTOLINK_RTPS_REJECT_OVERSIZE` | 未设（仅 WARN） | `1` → 超限拒发并计 `oversize` |

诊断：进程内 `RtpsStats::Instance().Dump()` 输出 `sent` / `recv` / `write_fail` / `oversize` / `matched_readers` / `matched_writers`。

---

## 14.3 双机清单（数据面 + 拓扑）

两端均需 `AUTOLINK_ENABLE_FASTDDS=ON` 构建，并：

| 项 | 要求 |
|----|------|
| `diff_host` | conf 中设为 `RTPS`（勿依赖默认 SHM） |
| `AUTOLINK_TOPOLOGY_BACKEND` | 设为 `rtps`（跨机 ChangeMsg；默认 `local` 仅本机文件总线） |
| `AUTOLINK_IP` | 对端可达的本机地址（勿用仅回环） |
| `AUTOLINK_DOMAIN_ID` | 两端相同（默认 `80`） |
| 网络 | 同 LAN；默认 **SIMPLE** 发现依赖多播可达（见 §14.4 改用 Server） |
| 防火墙 | 放行 DDS 多播与动态端口 |

- **数据面**：Channel Writer/Reader 在 `diff_host=RTPS` 时走 RTPS。
- **拓扑面**：`AUTOLINK_TOPOLOGY_BACKEND=rtps` 时，`ChangeMsg`（node/channel/service JOIN/LEAVE）经 `RtpsTopologyBackend` 在三路 broadcast topic 上传播；对端 `ChannelManager` / `NodeManager` 可见远端角色。
- **失败回退**：`BACKEND=rtps` 且 Hub/Backend `Start` 失败时，进程打 `AERROR` 并回退 `local`（仅同机发现），保证可启动。

### 双 Participant

`RtpsParticipantHub` 在同一 Domain 内持有两个 `DomainParticipant`：

| Participant | 用途 |
|-------------|------|
| topology | `RtpsTopologyBackend` 的 ChangeMsg Writer/Reader |
| transport | Channel RTPS 收发（`Transport`） |

二者独立 name/端口，避免拓扑流量与业务数据面争用同一端点集合。

### Security

DDS Security（身份认证、加密、权限）**当前未实现**。多机拓扑与数据面均依赖可信局域网；勿在不可信网络上默认开启跨机 RTPS。

---

## 14.4 Discovery Server（可选）

未设 `AUTOLINK_DISCOVERY_SERVER` 时保持 **SIMPLE** 多播发现。跨网段或禁多播时，可起 Fast DDS Discovery Server，各节点作 **CLIENT**：

```bash
# 服务器（任意可达主机；与 Fast DDS 安装一并提供）
fast-discovery-server -i 0.0.0.0 -p 11811

# 各 autolink 进程
export AUTOLINK_DISCOVERY_SERVER=192.168.1.10:11811
# 多 Server：逗号分隔
# export AUTOLINK_DISCOVERY_SERVER=192.168.1.10:11811,192.168.1.11:11811
```

`RtpsParticipantHub` 将 env 传给 topology/transport 两个 Participant；非空时 `discoveryProtocol=CLIENT` 并注入 server locator 列表。

---

## 14.5 升级到 Fast DDS 3.x 检查清单

当前实现钉 **2.14.x**。CMake 若 `find_package` 到 major≥3 会 WARNING。升 3.x 前核对：

| 项 | 2.14 | 3.x 预期 |
|----|------|----------|
| CMake 包名 / 链接目标 | 常见 `fastrtps`（亦试 `fastdds`） | 统一 `fastdds` |
| `DomainParticipant` / QoS API | `fastdds/dds/...` + 部分 `fastrtps` 类型 | 核对头路径与 `ReturnCode_t` 命名空间 |
| Discovery Server API | `load_environment_server_info` / `m_DiscoveryServers` | 按 3.x 迁移指南核对 CLIENT 配置 |
| Underlay / TypeSupport | 2.14 `TypeSupport` / Topic 创建方式 | 按 3.x 迁移指南改 `create_topic` / 类型注册 |
| FetchContent / pin | `GIT_TAG v2.14.6` | 换 3.x tag 并重跑 RTPS 集成测 |
| Security | 未实现 | 仍不引入；勿默认假设 3.x Security 可用 |

升版后至少跑：`topology_backend_factory_test`（OFF/ON）、`rtps_topology_backend_test`（ON）、`rtps_transceiver_test`（ON）、`payload_limit_test` / `rtps_stats_test`。
