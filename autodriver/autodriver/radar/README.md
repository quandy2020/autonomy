# radar

车载毫米波雷达后端（按厂商分包的雷达后端布局）。

| 路径 | 说明 |
|---|---|
| `backend_registry.*` | `backend` 名 → 工厂 |
| `backend_register.hpp` | `REGISTER_RADAR_BACKEND` |
| `conti/` | Conti ARS stub（`backend: conti` / alias `continental`）；当前 `Create`→`nullptr` |

落地后走 `canbus::CanReceiver` + Conti ProtocolData。样本暂为 `RadarSample`（`PointCloud2` 占位），待专用消息。

YAML 组：`radar` → `RadarModule`。详见 [backends](../../docs/source/guide/backends.md)。
