(nmpc-controller)=
# 4. NMPC Controller

有限时域 OCP：差速模型 + 路径参考 + costmap 碰撞校验。求解器可选 C-GMRES / DDP / FMPC。源码：`nmpc_controller/`。

把差速模型、路径参考与 costmap 碰撞检验放进有限时域 OCP，每周期解首控并 warm-start。适合需要显式状态/控制边界、比几何法更「硬约束」的场景；求解开销高于 RPP。

## 何时用

需要 **显式时域约束**、对控制量/状态有更硬边界时；比 RPP 重，比 SANDO 偏单轨 OCP 而非走廊 MIQP。

## 问题形式

$$
\min_{\mathbf{U}_{0:H-1}} \sum_{k}\ell(x_k,u_k)+V_f(x_H)
\quad\text{s.t.}\quad x_{k+1}=f(x_k,u_k)
$$

每周期解 $u_0^*$ 并 warm-start。路径在 costmap 系裁剪、投影为参考。

## 模块

| 模块 | 作用 |
|------|------|
| `path_transformer` / `path_reference` | Path → 时域参考 |
| `optimizer` + `solver/cgmres` | OCP 求解 |
| `trajectory_collision_checker` | rollout 碰障检查 |

## 配置

插件名与 `nmpc_controller.pb` / `autonomy/control/conf/` 模板；需有效 costmap 与 TF。

## 常见坑

| 现象 | 处理 |
|------|------|
| 无解 / 频繁失败 | horizon 与参考不匹配；障碍过近 |
| 延迟大 | 求解器迭代上限；降频率 |
| 与 NavFn 路径不一致 | 参考裁剪过短 |

归属 [04](../04_controller_algorithms.md)。
