# Trigger rules

配置：`robot/config/data_agent/*.yaml`。命中后拷贝 ring 窗口落盘。采集路径不得发布控制话题。

| 规则 | 条件 | 优先级 | 闭环去向 |
|------|------|--------|----------|
| estop | 安全闩置位 | P0 | 失败池 + 安全链 |
| task_fail | TaskResult / feedback FAILED | P0 | 失败池，根因卡片 |
| takeover | 遥控上升沿 | P0/P1 | 默认不进专家模仿 |
| loc_degrade | 定位质量低于阈值（边沿） | P1 | 可进回归 |
| path_deviate | odom vs plan | P2 | 可选人审 |
| manual | `trigger_snapshot.sh` | P2 | 测试 / 抽检 |
| periodic | 心跳 | P3 | 成功抽样 |

测试全量：会话 `--full-dump`，不靠触发截断。窗口长度与优先级下限分 conservative / aggressive / minimal 三套。
