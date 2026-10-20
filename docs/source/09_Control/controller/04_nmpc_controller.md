(nmpc-controller)=
# 4. NMPC Controller

源码：`autonomy/control/controller/nmpc_controller/`。差速路径跟踪 + costmap 碰撞校验；求解器可选 DDP / FMPC / C-GMRES。

### 定义

有限时域 OCP，差速运动学 $f$，代价跟踪路径参考并惩罚控制：

$$
\min_{\mathbf{U}_{0:H-1}} J=\sum_{k=0}^{H-1}\ell(x_k,u_k)+V_f(x_H)
\quad\text{s.t.}\quad
x_{k+1}=f(x_k,u_k)
$$

执行 $u_0^*$；下一周期 warm-start。路径在 costmap 系下裁剪并投影。

### 结构

| 模块 | 作用 |
|------|------|
| `path_transformer` / `path_reference` | 全局 Path → 时域参考 |
| `optimizer` + `solver/cgmres` | OCP 求解 |
| `trajectory_collision_checker` | rollout 与 costmap 碰撞检查 |
| `controller` | `ControllerInterface` 插件 |

控制循环：变换/裁剪路径 → 投影位姿建参考 → 求解 → costmap 校验 → 发布 `[v,\omega]`。

### 用法

插件名见 `ControllerOptions` / `nmpc_controller.pb`；配置模板在 `autonomy/control/conf/`。需有效 costmap 与 TF。
