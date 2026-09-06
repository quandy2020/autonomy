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
ctest --test-dir autolink/build-fastdds -R rtps_transceiver -V
```

说明：同进程 RTPS PubSub 在部分平台上可能只完成匹配、不触发 `DataReaderListener`；双机清单仍是正式验收路径。`autolink/conf/fastdds_profiles.xml` 将 intraprocess 设为 `OFF`，优先走 UDP。
---

## 14.2 双机清单

两端均需 `AUTOLINK_ENABLE_FASTDDS=ON` 构建，并：

| 项 | 要求 |
|----|------|
| `diff_host` | conf 中设为 `RTPS`（勿依赖默认 SHM） |
| `AUTOLINK_IP` | 对端可达的本机地址（勿用仅回环） |
| `AUTOLINK_DOMAIN_ID` | 两端相同（默认 `80`） |
| 网络 | 同 LAN；Fast DDS **SIMPLE** 发现依赖多播可达 |
| 防火墙 | 放行 DDS 多播与动态端口 |

拓扑图仍为本机文件总线；完整跨机 ChangeMsg over RTPS 当前未提供。DDS 端点匹配与业务数据走 RTPS，不属于 Autolink L2 发现。

---

## 14.3 升级到 Fast DDS 3.x 检查清单

当前实现钉 **2.14.x**。升 3.x 前核对：

| 项 | 2.14 | 3.x 预期 |
|----|------|----------|
| CMake 包名 / 链接目标 | 常见 `fastrtps`（亦试 `fastdds`） | 统一 `fastdds` |
| `DomainParticipant` / QoS API | `fastdds/dds/...` + 部分 `fastrtps` 类型 | 核对头路径与 `ReturnCode_t` 命名空间 |
| Underlay / TypeSupport | 2.14 `TypeSupport` / Topic 创建方式 | 按 3.x 迁移指南改 `create_topic` / 类型注册 |
| FetchContent / pin | `GIT_TAG v2.14.6` | 换 3.x tag 并重跑 RTPS 集成测 |

升版后至少跑：`topology_backend_factory_test`（OFF 回归）与 `rtps_transceiver_test`（ON）。
