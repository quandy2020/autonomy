(pose-progress-checker)=
# 5. PoseProgressChecker

归属 [§3.6](../02_checkers.md)。 XY **或** 航向变化算进度。

### 定义

相对基准：位移 $\ge r$ **或** $|\Delta\theta|\ge\varepsilon_\theta$ → 有进度并刷新基准。

适用：原地旋转接近目标航向时 SimpleProgress 会误报卡住。

参数：位移半径 + 航向阈值 + 时限（同 SimpleProgress 族）
