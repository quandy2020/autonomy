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

## UM10034P Rev.1.1 手册要点（本驱动据此校核）

| 项目 | 手册值 | 代码落点 |
|---|---|---|
| 频段 | 24 GHz ISM，扫频 24–25 GHz 可配 | `center_frequency_hz`，`wavelength_m_ = c/f` |
| 距离分辨率 | `c/(2B)`；GUI 示例 24000–24245 MHz（B=245 MHz）→ **0.63 m/bin** | `bandwidth_hz` → `range_resolution_m`（覆盖旧值并告警） |
| 测速 | Doppler FFT（§5.3）；示例 32 chirp、1270 µs → **v_max 2.45 m/s，Δv 0.15 m/s** | `EstimateIcl1122DopplerVelocity()`：chirp 间相位差，替代整 bin 帧差 |
| 距离细分 | 分辨率决定量化步长 | `InterpolateIcl1122PeakBin()`：三点抛物线插值，亚 bin 距离 |
| 天线 | 0° 合成增益 ≈20 dBi；6 dBi 波束 **86°×28°（H×E）**，H 面 **±43°** | `max_abs_azimuth_deg` 方位门控 |
| 芯片杂波抑制 | 数字部分含变系数杂波抑制，输出已抑制的 Range FFT | 慢时间再做一次 MTI（去每 RX 复均值） |
| 数据帧 | 0xAA dword：RX[23]、chirp[19:11]、FFT 点数[10:0]；尾部 checksum[31:16]、frame[15:12]、0x55 | `protocol.cpp` / `range_fft.cpp`（与 24GMmwave 参考实现一致） |
| MCU 封包 | `49 43 4C 48` + len(2) + type + channel + data + `49 43 4C 54` | `Icl1122McuParser` |
| 接口速率 | USB 2.0 High Speed，最大 34 Mbps；IAP 用 256000 | YAML `baud`（USB CDC 名义值） |

启动日志新增：

```
ICL1122 scale dR=0.612 m lambda=0.01243 m bandwidth_hz=2.45e+08 chirp_period_s=0 v_max=0 m/s v_res=0 m/s az_limit_rad=0.750492
```
