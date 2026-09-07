# 生命周期

`SensorManager` 管理传感器从配置加载到热插拔的全流程。发布由 `SampleSink`（通常是 `bridge::Publisher`）承接。

## 进程流程

```
LoadConfig(YAML)
  → Publisher.Initialize()          # 创建 Autolink Node
  → SensorManager(config)
  → SetSink(&publisher)
  → Initialize()                    # 查重 id
  → Start()
       ├─ alignment.enable → hub.Start()
       ├─ 对 autostart 传感器 Attach()
       └─ hotplug.udev → StartUdev()（Linux + libudev）
  → … 运行 …
  → Stop()                          # Detach 全部、停 Hub、停 udev
```

## enable 语义

typed YAML（`sensors.imu` 等）中：

| `enable` | 行为 |
|---|---|
| `true`（或旧别名 `attach_on_start: true`） | 条目进入 `Config.sensors`，且 `autostart=true`，`Start()` 时 Attach |
| `false` / 省略 | **不进入 Config**，之后也无法 `Attach` |

因此热插拔只作用于**已加载且带 `match` 的条目**：拔掉 → Detach；再插上 → Attach。若希望「平时 detach、插上再 attach」，仍需 `enable: true` 并配置 `match`。

## Attach / Detach

- **幂等**：已 Attach 再 Attach 返回成功；未 Attach 再 Detach 无操作。
- **Attach(id)**：
  1. 按 `Config::Sensor` 找配置
  2. `library` 空 → 进程内 `ClassLoaderManager` 创建内置 module
  3. `library` 非空 → 从 `plugin_dir` / `AUTODRIVER_PLUGIN_DIR` 加载外置 `.so`
  4. `Init` → sink `OnAttach`（开 Writer）→ `Start`
- **Detach(id)**：`Stop` → sink `OnDetach` → `hub.DropBuffer`；外置库无引用则 unload
- 未知 id 或加载失败返回 `false`，进程继续运行

## udev 热插拔

仅当 `hotplug.enable_udev: true` 且编译进 `AUTODRIVER_HAVE_UDEV` 时生效。

1. 收到 ADD / REMOVE
2. `Config::FindId(observed)`：用 `MatchDevice` 对照各传感器的 `match`
3. 命中则 Attach / Detach

`match` 规则：空规则永不匹配；非空字段须全部匹配（vendor/product 支持 `0x` 前缀、大小写不敏感）。

serial 且未写 `match` 时，若 `params.device`（或 YAML `port`）已填，会自动：

```text
match.subsystem = tty
match.device    = <port>
```

## 对齐旁路

`alignment.enable: true` 时，模块在写 sink 之外把同一份 `SensorSample`（`shared_ptr`）推入 `SensorHub`，按窗口对齐后回调。对齐关闭时热路径只有 stamp → sink。

## 相关

- [配置 · 热插拔匹配](configuration.md#热插拔匹配)
- [API · SensorManager](../api/overview.md)
