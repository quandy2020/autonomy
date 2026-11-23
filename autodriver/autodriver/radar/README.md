# radar

车载 / 评估板毫米波雷达后端（按厂商分包）。

| 路径 | 说明 |
|---|---|
| `backend_registry.*` | `backend` 名 → 工厂 |
| `backend_register.hpp` | `REGISTER_RADAR_BACKEND` |
| `conti/` | Conti ARS stub（`backend: conti` / `continental`）；`Create`→`nullptr` |
| `icl1122/` | 矽典微 ICL1122 / EVB1122：Demo 目标帧 + JoySpace RangeFFT |
| `../thirdparty/24GMmwave/` | JoySpace 上位机源码（`MmwaveController`） |

## 与 ICLM_TrafficRadar 一致（推荐）

上位机路条目标图（0–150 m、车标、ID）= **室外远距离 / Traffic 目标列表**，**不是** JoySpace 频谱：

| 项 | ICLM_TrafficRadar | autodriver |
|---|---|---|
| 波特率 | **256000** | `baud: 256000` |
| 数据 | 目标距离 / 角度 | `AA FF N 00 … 55 CC` → scan/tracks/points |
| 量程 | 0–150 m | autoviz **+Y**，格点 10 m |
| 配置 | — | **`host_bringup: false`** |

步骤：

1. Windows 用 **ICLM_TrafficRadar**（或 EVB1122 GUI「室外远距离」）连接并 Start。  
2. 拔掉 Type-C 再插（清掉 JoySpace FFT 残留）。  
3. Linux：
   ```bash
   autodriver -c /workspace/autonomy/src/autonomy/autodriver/config \
     --config-file autodriver_hardware.yaml
   /workspace/autonomy/src/autonomy/autoviz/scripts/run_radar_autoviz.sh
   ```
4. Fixed Frame = `radar_link`；**绿轴 Y = 距离**（格子 × 10 m）。

## JoySpace Range-FFT（调试用）

文档：[JoySpace 说明](https://joyspace.jd.com/pages/S3mxI9MQ2wDi56Qxu4C3)

- Type-C → `/dev/ttyACM0`，波特率 **921600**
- `host_bringup: true` + `register_config` → `ICLH` 频谱峰值  
- 与 Traffic 路条目标图 **不是同一模式**

```bash
./build/MmwaveController /dev/ttyACM0 config/RegisterConfig16_8_2db_840 16 8
```

## autodriver `icl1122`

- YAML：`backend: icl1122`（别名 `evb1122` / `um10034` / `iclegend`）
- 发布：`/radar/front/{scan,tracks,points}`
- 配置：`config/radar/icl1122.yaml`

详见 [backends](../../docs/source/guide/backends.md)。
