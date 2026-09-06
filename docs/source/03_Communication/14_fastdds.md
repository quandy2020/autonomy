# 14. 可选 FastDDS（跨机 RTPS）

跨机 Channel 为**可选**能力：默认仍为本机 INTRA + SHM；打开 Fast DDS 后可用 RTPS 数据面。

| 本文 §14 | 相关文档 |
|---------|----------|
| 启用、双机、升 3.x | [§0 指南](00_guide.md) · [§1 架构](01_architecture.md) · [§3 Channel](03_channel.md) |

---

## 14.1 编译启用

```bash
cmake -S autolink -B autolink/build-fastdds \
  -DAUTOLINK_BUILD_TEST=ON \
  -DAUTOLINK_ENABLE_FASTDDS=ON
cmake --build autolink/build-fastdds -j
```

- CMake option：`AUTOLINK_ENABLE_FASTDDS`（默认 **OFF**）。
- 优先 `find_package(fastdds|fastrtps 2.14)`；未安装则 FetchContent 钉 **v2.14.6**。
- 打开后库目标定义 `AUTOLINK_ENABLE_FASTDDS=1`，编译 `transport/rtps/` 与 Hybrid/Transport 的 RTPS 分支。

验证：

```bash
ctest --test-dir autolink/build-fastdds \
  -R 'rtps_transceiver|topology_backend_factory|rtps_topology_backend' -V
```

说明：同进程 RTPS PubSub 在部分平台上可能只完成匹配、不触发 `DataReaderListener`；双机清单仍是正式验收路径。`autolink/conf/fastdds_profiles.xml` 将 intraprocess 设为 `OFF`，优先走 UDP。

---

## 14.2 双机清单（数据面 + 拓扑）

两端均需 `AUTOLINK_ENABLE_FASTDDS=ON` 构建，并：

| 项 | 要求 |
|----|------|
| `diff_host` | conf 中设为 `RTPS`（勿依赖默认 SHM） |
| `AUTOLINK_TOPOLOGY_BACKEND` | 设为 `rtps`（跨机 ChangeMsg；默认 `local` 仅本机文件总线） |
| `AUTOLINK_IP` | 对端可达的本机地址（勿用仅回环） |
| `AUTOLINK_DOMAIN_ID` | 两端相同（默认 `80`） |
| 网络 | 同 LAN；Fast DDS **SIMPLE** 发现依赖多播可达 |
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

## 14.3 升级到 Fast DDS 3.x 检查清单

当前实现钉 **2.14.x**。升 3.x 前核对：

| 项 | 2.14 | 3.x 预期 |
|----|------|----------|
| CMake 包名 / 链接目标 | 常见 `fastrtps`（亦试 `fastdds`） | 统一 `fastdds` |
| `DomainParticipant` / QoS API | `fastdds/dds/...` + 部分 `fastrtps` 类型 | 核对头路径与 `ReturnCode_t` 命名空间 |
| Underlay / TypeSupport | 2.14 `TypeSupport` / Topic 创建方式 | 按 3.x 迁移指南改 `create_topic` / 类型注册 |
| FetchContent / pin | `GIT_TAG v2.14.6` | 换 3.x tag 并重跑 RTPS 集成测 |

升版后至少跑：`topology_backend_factory_test`（OFF/ON）、`rtps_topology_backend_test`（ON）与 `rtps_transceiver_test`（ON）。
