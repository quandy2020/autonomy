(simple-progress-checker)=
# 4. SimpleProgressChecker

归属 [§3.5](../02_checkers.md)。 XY 位移进度；`movement_time_allowance` 时限 C++ 待完善。

### 定义

相对基准位姿，若位移 $\ge r_{\mathrm{req}}$ 则有进度并更新基准；超时未达 → `Check=false`。

### 参数

| — | 默认 |
|--|------|
| `required_movement_radius` | 0.5 m |
| `movement_time_allowance` | 10 s |

`Reset()` 清基准
