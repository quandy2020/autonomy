(simulation-math)=
# 3. 数学原理

理解 vehicle 限幅、差速积分或 Gazebo 侧运动学；实现见 [§8](08_vehicle_stage.md)。

公式与实现一一对应；调参时区分 network 后处理与 costmap/层参数。

### 差速运动学

输入 $(v,\omega)$：

$$
\dot x=v\cos\theta,\quad \dot y=v\sin\theta,\quad \dot\theta=\omega
$$

前向欧拉（步长 $\Delta t$）：

$$
x\leftarrow x+v\cos\theta\,\Delta t,\quad
y\leftarrow y+v\sin\theta\,\Delta t,\quad
\theta\leftarrow\mathrm{NormalizeAngle}(\theta+\omega\Delta t)
$$

### 轮速与 Ackermann

| 模型 | 关系 |
|------|------|
| 差速逆解 | $v=r(\omega_R+\omega_L)/2$，$\omega=r(\omega_R-\omega_L)/L$ |
| Ackermann | $\omega=v\tan\delta/L$ |

### 限幅

$v^{cmd}=\mathrm{clip}(v^{raw},-v_{rev},v_{max})$；加加速度限制见 `KinematicsControl`。

### 仿真时间

$t_{sim}$ 由 Gazebo `/clock` 或 Autolink `EnableSimulationMode()` 驱动。

### 符号

$v,\omega,\Delta t,L,\theta$ — 线/角速度、步长、轮距、航向。

### 延伸阅读

[§8 Vehicle](08_vehicle_stage.md) · [§7 Gazebo](07_gazebo_ros.md)
