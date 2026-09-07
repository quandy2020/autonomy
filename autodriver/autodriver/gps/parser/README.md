# gps/parser

GNSS 报文解析工厂（「传输 × 解析」拆分）。

| 类型 | 说明 |
|---|---|
| `GnssParser` | `Consume(bytes)` → 可选 `ParsedFix` |
| `GnssParserRegistry` | 名 → 工厂；`Instance()` 首次注册 NMEA |
| `Nmea0183Parser` | 行缓冲 GGA/RMC |

内置名：`nmea`、`nmea0183`。`SerialGpsDriver` 经 `GnssParserRegistry` 使用本层。
