(running-board-task-launch-benchmark)=
# 9. 板端部署资源报告：`task.launch`

Firefly / RK3588 上空闲栈实测（**无 map / 无 NavigateToPose**）。

```bash
source /usr/local/share/autonomy/setup.bash
autolink launch start task.launch
```

| 项 | 值 |
|----|-----|
| 日期 | 2026-09-20 · Autonomy `feature/library` · `/usr/local` |
| 场景 | 空闲栈；有 map + 导航时按 §9.7 复测 |

```{admonition} 结论
:class: tip

空闲三进程合计约 **7–8% 单核 CPU**、**360–370 MiB RSS**（≈ 板载 **4.5–5%**），拉起 **亚秒级**；**无 Swap**。
```

| 指标 | 数值 | 备注 |
|------|------|------|
| 稳态 CPU | ≈ **7–8%** 单核 | ≈ 1% @ 8 核 |
| 启动 CPU | ≈ **28%** 单核 | ~6 s 回落 ≈ **3.7×** |
| RSS / PSS | ≈ **360–370** / 估 **120 MiB** | RSS 作 OOM；PSS 更真实 |
| 线程 | ≈ **110+** | 45 / 33 / 32 |
| 拉起 / Swap | **亚秒** / **0** | — |

## 9.0 图示九宫格

```{figure} ./images/benchmark_jiugongge.png
:alt: task.launch 3x3 gallery
:align: center
:width: 96%

**3×3 九宫格**：KPI · 进程树 · 部署 · CPU · vs Nav2 · RSS · 板载占比 · 线程 · 启动时序。
```

```{figure} ./images/benchmark_gallery.png
:alt: task.launch full gallery
:align: center
:width: 96%

**全量宫格（3×4）**：上图九格 + vs Nav2 CPU、RSS vs PSS（共 11 张）。
```

| # | 图 | 要点 |
|---|-----|------|
| 1 | KPI | 稳态/启动 CPU、RSS、线程、拉起、Swap |
| 2 | 进程树 | autolink + 三进程 + SHM |
| 3 | 部署 | NFS → build → `/usr/local` |
| 4 | CPU | 28% → 7.5%（≈3.7×） |
| 5–6 | vs Nav2 | CPU 约 15–20×；PSS 接近；3 vs 15 |
| 7–8 | 内存 | RSS 合计 ~365 MiB；占板载 4.7% |
| 9 | RSS/PSS | task 128 vs ~50 MiB |
| 10 | 线程 | 32 / 33 / **45** |
| 11 | 启动 | +0…+600 ms SHM ready |

---

## 9.1 被测对象

| 模块 | 二进制 | 作用 | RSS | 线程 |
|------|--------|------|-----|------|
| planning | `autonomy.planning` | 全局规划 | 111–113 MiB | 32 |
| control | `autonomy.control` | 局部跟踪 | 114–117 MiB | 33 |
| task | `autonomy.task` | TaskServer + BT | 124–128 MiB | 45 |
| 父进程 | `autolink launch` | 监督 / respawn | 10–11 MiB | 1 |

| 步骤 | 位置 | 说明 |
|------|------|------|
| 1–2 | 开发机 → 板端 | NFS 源码 → `~/autonomy_ws/src` |
| 3–4 | 板端 | 本地 `build` → `/usr/local` + `setup.bash` |

---

## 9.2 硬件与测量

| 项 | 值 |
|----|-----|
| 主机 | `firefly` · RK3588 · 8 核 · `aarch64` |
| 内核 / OS | `5.10.160-rt78-preempt` · Ubuntu 22.04 |
| 内存 | ≈ **7.7 GiB** · **Swap = 0** |
| CPU 含义 | 进程 **单核 %**（`top` 同义） |

| 指标 | 方法 |
|------|------|
| CPU / RSS 瞬时 | `ps …` |
| 稳态 CPU | `/proc/<pid>/stat` 10 s jiffies 差分 |
| RSS / HWM / PSS | `status` / `smaps_rollup` |
| 启动时序 | `autolink` 日志时间戳 |

```bash
source /usr/local/share/autonomy/setup.bash
autolink launch start task.launch
ps -eo pid,%cpu,%mem,rss,vsz,nlwp,etime,cmd | \
  grep -E 'autonomy\.(task|planning|control)|autolink launch'
autolink launch stop
```

---

## 9.3 CPU

| 进程 | 启动 ~6 s | 空闲稳态 |
|------|----------|----------|
| `autonomy.task` | ≈ 9.8% | ≈ 2.2–2.4% |
| `autonomy.control` | ≈ 9.3% | ≈ 2.5% |
| `autonomy.planning` | ≈ 9.0% | ≈ 2.5–2.8% |
| **合计** | ≈ **28%** | ≈ **7–8%**（≈ **3.7×**） |

| 解读 | |
|------|--|
| 空闲 / 启动 @ 8 核 | ≈ 1% / ≈ 3.5% |
| 有导航目标 | planning / control 升至数十 % 单核 |
| 整机 load | 含厂商后台 |

---

## 9.4 与 ROS 2 Nav2 对照

| 来源 | 平台 · 工况 | 文献 |
|------|-------------|------|
| Nav2 | Pi 4 · Gazebo 稳态导航 | [arXiv:2305.09933](https://arxiv.org/abs/2305.09933) Table I |
| Autonomy | Firefly · 空闲无 map | 本文 |
| 社区 | Pi 4 · Nav2 CPU 打满 | [Robotics SE](https://robotics.stackexchange.com/questions/114197/nav2-overloading-raspberry-pi-cpu) |

| 维度 | Nav2 多进程 | Nav2 组合 | Autonomy 空闲 |
|------|-------------|-----------|---------------|
| CPU % 单核 | **154.3** | **110.5** | **7–8** |
| PSS MiB | **116.6** | **77.8** | 估 **118** |
| RSS MiB | — | — | **360–370** |
| 节点/进程 | **15** | 单容器 | **3** |
| 拉起 | Lifecycle（秒级常见） | 同左 | **亚秒** |

> **非同机同工况**；正式 A/B 需同板 + 同图 + 同 `NavigateToPose`。

---

## 9.5 内存与线程

| 进程 | RSS | HWM | 线程 |
|------|-----|-----|------|
| `autonomy.task` | 124–128 MiB | 128 MiB | **45** |
| `autonomy.control` | 114–117 MiB | 117 MiB | **33** |
| `autonomy.planning` | 111–113 MiB | 113 MiB | **32** |
| `autolink launch` | 10–11 MiB | 11 MiB | 1 |
| **合计** | **360–370 MiB** | — | **110+** |

| 占比 | 数值 |
|------|------|
| MemTotal | ≈ **7.7 GiB** |
| task.launch RSS | ≈ **0.36 GiB**（**4.7%**） |
| Swap | **0** |

| 文件 | 约大小 |
|------|--------|
| `autolink` / `libautolink.so` | 2.4 / 10 MiB |
| `libautonomy_task.so` | ≈ 25 MiB |
| `ldd autonomy.task` | **260+**（需 `setup.bash`） |

| 风险 | 建议 |
|------|------|
| 无 Swap | 编译 `-j1/-j2` |
| RSS 爬升 | 查地图 / costmap / 泄漏 |

---

## 9.6 启动时序

| 相对时间 | 事件 |
|----------|------|
| +0 ms | 解析 launch |
| ~80 / 120 / 160 ms | planning / control / task Start OK |
| ~600 ms | SHM discovery（百余 channel） |

| 结论 | |
|------|--|
| 进程拉起 | **亚秒级**；导航就绪仍需 map + TF |
| `no map received` | costmap 告警，非 launch 卡顿 |
| 有图后补测 | Goal→path、控制周期、BT tick |

---

## 9.7 复测清单

| # | 步骤 | 记录 |
|---|------|------|
| 1 | `setup.bash` → `start task.launch`，≥30 s | — |
| 2 | 采集 | `ps` / `smaps` / 10 s CPU |
| 3 | map + TF，60 s | 稳态 |
| 4 | `NavigateToPose` | 峰值、goal→path |
| 5 | `launch stop` | 无残留（勿 `pkill -f`） |

---

## 9.8 相关文档

| 文档 | 链接 |
|------|------|
| 快速运行 / 多进程栈 | [§2](02_quickstart.md) · [§3](03_autonomy_process.md) |
| 环境 / 板端安装 | [安装 §7](../02_Installation/07_environment.md) · [§9](../02_Installation/09_embedded_board.md) |
