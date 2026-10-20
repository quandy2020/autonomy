# 14. 可选 FastDDS（跨机 RTPS）

默认本机 INTRA + SHM。打开 Fast DDS 后可用 RTPS 数据面。

## 14.1 编译

```bash
bash docker/install/install_fastdds.sh
cmake -S autolink -B autolink/build-fastdds \
-DAUTOLINK_BUILD_TEST=ON -DAUTOLINK_ENABLE_FASTDDS=ON
cmake --build autolink/build-fastdds -j
```

| 项 | 说明 |
|----|------|
| 开关 | `AUTOLINK_ENABLE_FASTDDS`（默认 OFF） |
| 基线 | Fast DDS **3.x**（pin **v3.6.2**）；不支持 2.x |
| 链接 | `fastdds` + `fastcdr`；FetchContent 时需 OpenSSL |

```bash
ctest --test-dir autolink/build-fastdds \
-R 'rtps_transceiver|topology_backend_factory|rtps_topology_backend' -V
```

## 14.2 环境变量

| 变量 | 默认 | 含义 |
|------|------|------|
| `AUTOLINK_TOPOLOGY_BACKEND` | `local` | `rtps` → 跨机 ChangeMsg |
| `AUTOLINK_IP` | 本机探测 | Participant 宣告地址 |
| `AUTOLINK_DOMAIN_ID` | `80` | DDS Domain |
| `AUTOLINK_DISCOVERY_SERVER` | 空（SIMPLE） | `ip:port` → CLIENT |
| `AUTOLINK_RTPS_MAX_PAYLOAD_BYTES` | 4 MiB | payload 软限 |
| `AUTOLINK_RTPS_SECURITY` | 关 | `1` → DDS Security |
| `AUTOLINK_RTPS_SECURITY_DIR` | — | 证书目录（Security 时必填） |

## 14.3 双机清单

两端均 `AUTOLINK_ENABLE_FASTDDS=ON`，且：

| 项 | 要求 |
|----|------|
| `diff_host` | conf 中 `RTPS` |
| `AUTOLINK_TOPOLOGY_BACKEND` | `rtps` |
| `AUTOLINK_IP` | 对端可达（非仅回环） |
| `AUTOLINK_DOMAIN_ID` | 两端相同 |
| 网络 | 同 LAN；SIMPLE 需多播；禁多播用 Discovery Server |

失败回退：`BACKEND=rtps` 且 Start 失败 → 回退 `local`（仅同机）。

Security（可选）：`AUTOLINK_RTPS_SECURITY=1` + 六文件证书目录（identity/permissions CA、cert/key、governance/permissions.smime）。一侧 ON 一侧 OFF 不应明文互通。

## 14.4 Discovery Server

```bash
fast-discovery-server -i 0.0.0.0 -p 11811
export AUTOLINK_DISCOVERY_SERVER=192.168.1.10:11811
```

## 14.5 构建注意

| 项 | 说明 |
|----|------|
| 系统仅 2.x | CMake FATAL |
| 首次 ON | wipe 旧 `build-fastdds` |
| 明文测例 | `unset AUTOLINK_RTPS_SECURITY AUTOLINK_RTPS_SECURITY_DIR` |
