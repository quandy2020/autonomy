(simulation-math)=
# 3. 数学原理

车辆限幅：[§8](08_vehicle_stage.md)；Gazebo：[§7](07_gazebo_ros.md)。

### 3.1 差速运动学

本体 $v_y=0$；输入 $(v,\omega)$：

$$
\dot x=v\cos\theta,\quad \dot y=v\sin\theta,\quad \dot\theta=\omega
$$

前向欧拉（`sim_dt`，默认 0.1 s）：

$$
x\leftarrow x+v\cos\theta\,\Delta t,\quad
y\leftarrow y+v\sin\theta\,\Delta t,\quad
\theta\leftarrow\mathrm{NormalizeAngle}(\theta+\omega\Delta t)
$$

姿态四元数：$q_z=\sin(\theta/2)$，$q_w=\cos(\theta/2)$。

### 3.2 轮速 / 全向 / Ackermann

| 模型 | 公式 |
|------|------|
| 差速逆解 | $v=r(\omega_R+\omega_L)/2$，$\omega=r(\omega_R-\omega_L)/L$ |
| 差速正解 | $\omega_R=(v+\omega L/2)/r$，$\omega_L=(v-\omega L/2)/r$ |
| 全向 | $[\dot x,\dot y,\dot\theta]^\top=R(\theta)[v_x,v_y,\omega]^\top$ |
| Ackermann | $\omega=v\tan\delta/L$，$R_{min}=L/\tan\delta_{max}$ |

### 3.3 里程计与限幅

$\mathbf z_k=[x,y,\theta]^\top+\mathbf n_k$，$\mathbf n\sim\mathcal N(0,\Sigma)$（理想仿真常无噪声）。

$$
v^{cmd}=\mathrm{clip}(v^{raw},-v_{rev},v_{max}),\quad
\omega^{cmd}=\mathrm{clip}(\omega^{raw},-\omega_{max},\omega_{max})
$$

加速度：$v_k=\mathrm{clip}(v_{k-1}+a_{max}\Delta t,v_{min},v_{max})$。

### 3.4 仿真时间

$t_{sim}=t_0+\sum\Delta t_{sim}$；由 Gazebo `/clock` 或 Autolink `MODE_SIMULATION` 驱动。

### 3.5 符号表

| 符号 | 含义 |
|------|------|
| $v,\omega,\Delta t,L,\theta,R_{min}$ | 线/角速度、步长、轮距/轴距、航向、最小转弯半径 |
