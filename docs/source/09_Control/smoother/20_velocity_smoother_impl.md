(velocity-smoother-impl)=
# 20. VelocitySmoother

> 归属 [§4 速度平滑器](../04_velocity_smoother.md) · Autonomy ✅ 算法已实现，pub/sub 待接线
>
> **VelocitySmoother**（Nav2 `nav2_velocity_smoother`）对控制器输出的 `cmd_vel` 施加**逐轴加速度界**与可选**三轴同步缩放**，在 OPEN/CLOSED_LOOP 反馈下输出可执行的平滑速度；属于 **time-scaling 的 $u$ 层实现**，不替代 TEB/NMPC 的轨迹时空优化。

---

## 1. 背景

控制器（尤其 DWB/MPPI）可在单周期给出大幅速度跳变，而电机与驱动器有 $a_{max}$、deadband 限制。Nav2 将平滑独立为 VelocitySmoother 节点或 Server 内组件，在 **FollowPath 流水线末端**（见 [§0.9](../00_guide.md#09-控制流水线)）保证 `cmd_vel` 物理可行；与 Graceful 内置 jerk 约束、TEB 优化 $\Delta T_k$ 互补。

---

## 2. 问题

**任务.** 将原始速度命令 $v^{cmd}$ 映射为满足加速度界与速度界的输出 $v^{out}$。

**输入 / 输出.** `inputCommandCallback(v^{cmd})` 缓存命令；`smootherTimer()` 按 `smoothing_frequency` 输出 $v^{out}$（三轴 $(v_x,v_y,\omega_z)$）。

**在线形式.** 定时器驱动；超时无新命令则渐近零速（`velocity_timeout`）。

---

## 3. 速度与离散时间模型

**状态.** 当前反馈速度 $v^{curr}$（OPEN_LOOP：上次输出；CLOSED_LOOP：`OdomSmoother` 滑动平均）。

**控制周期.** $f = \texttt{smoothing\_frequency}$（默认 20 Hz），$\Delta t = 1/f$。

**单轴加速度界.** 加速时 $a>0$，减速时减速度 $d<0$：

$$
\Delta v_{max} = \frac{a}{f}, \quad \Delta v_{min} = -\frac{a}{f} \quad \text{（加速模式）},
$$

$$
\Delta v_{max} = -\frac{d}{f}, \quad \Delta v_{min} = \frac{d}{f} \quad \text{（减速模式，$d<0$）}.
$$

加速/减速模式由 $|v^{cmd}|$ 与 $v^{curr}$ 符号关系判定（见 Nav2 实现）。

---

## 4. 数学问题定义

**步骤 1 — clamp 命令.**

$$
v^{cmd}_i \leftarrow \mathrm{clamp}(v^{cmd}_i,\; v_{min,i},\; v_{max,i}), \quad i \in \{x,y,\omega\}.
$$

**步骤 2 — 同步缩放（可选）.** `scale_velocities=true` 时求 $\eta \in (0,1]$ 使三轴 $\eta\,(v^{cmd}-v^{curr})$ 均落在各轴 $[\Delta v_{min},\Delta v_{max}]$ 内；取使 $|1-\eta|$ 最小的 $\eta$（`findEtaConstraint`）。

**步骤 3 — 应用约束.**

$$
v^{out} = v^{curr} + \mathrm{clamp}\bigl(\eta\,(v^{cmd}-v^{curr}),\; \Delta v_{min},\; \Delta v_{max}\bigr).
$$

**步骤 4 — deadband.** $|v^{out}_i| < v^{deadband,i} \Rightarrow v^{out}_i = 0$。

---

## 5. 平滑算法

<div class="algorithm-box-diagram">

<div class="algorithm-box algorithm-box-phase-a">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 1</span>
    <span class="algorithm-box-title">VelocitySmoother::smootherTimer</span>
  </div>
  <div class="algorithm-box-io" markdown="1">

**输入**：缓存的 $v^{cmd}$，$v^{curr}$（OPEN/CLOSED_LOOP）  
**输出**：$v^{out}$ → 底盘

  </div>
  <div class="algorithm-box-body" markdown="1">

1. **若** 距上次命令 $> T_{timeout}$ → 命令置零，平滑减速  
2. $v^{cmd} \leftarrow \mathrm{clamp}(v^{cmd}, v_{min}, v_{max})$  
3. 逐轴计算 $\Delta v_{max/min}$（加速/减速模式）  
4. **若** `scale_velocities` → $\eta \leftarrow \mathrm{FindEta}(v^{cmd}, v^{curr})$；**否则** $\eta \leftarrow 1$  
5. $v^{out} \leftarrow \mathrm{ApplyConstraints}(v^{curr}, v^{cmd}, \eta)$  
6. deadband 过滤；保存 `last_cmd_`  

  </div>
</div>

<div class="algorithm-box algorithm-box-phase-b algorithm-box-subroutine">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 2</span>
    <span class="algorithm-box-title">CLOSED_LOOP 反馈</span>
  </div>
  <div class="algorithm-box-body" markdown="1">

$v^{curr}$ 取自 `OdomSmoother`：窗口 $T_{odom}$ 内里程计算术平均  
$\bar{v}_x = \frac{1}{N}\sum_i v_{x,i}$（$v_y,\omega_z$ 同理）

  </div>
</div>

</div>

**数值例.** $f=20$ Hz，$a=2.5$ m/s²，$v^{curr}=0$，$v^{cmd}_x=1.0$ → $\Delta v_{max}=0.125$ m/s/周期，约 8 周期（0.4 s）爬升至 1.0 m/s。源码见 `velocity_smoother.cpp`；配置接线 [§4.12](../04_velocity_smoother.md#412-配置接线状态)。

---

## 6. 参考文献

1. Navigation2 velocity_smoother: [configuring-velocity-smoother](https://navigation.ros.org/configuration/packages/configuring-velocity-smoother.html)
