# 多源融合与降级设计

## 1. 目标

在视觉、激光、IMU、GPS、气压、光流等多源下，提供：

1. **松耦合**：各前端独立，外层融合  
2. **紧耦合**（规划）：统一因子图  
3. **降级**：传感器失效时保持可用估计  

## 2. 模块

| 路径 | 类 | 职责 |
|------|-----|------|
| `fusion/ekf/` | `MultiSourceEkf` | 位姿/位置/高度更新 + 门控 |
| `fusion/factor_graph/` | `FusionGraph` | 小批量加权位姿融合 |
| `fusion/outlier_rejection/` | `OutlierRejection` | Mahalanobis / χ² |
| `fusion/degradation/` | `DegradationManager` | 健康等级与推荐模式 |

配置：`frontend.fusion: loose | tight`。

## 3. 松耦合数据流

```text
VIO odom ──┐
LIO odom ──┼─► gate ─► MultiSourceEkf / FusionGraph ─► 输出
GPS/baro ──┘
```

- Predict：前端 odom / IMU coast  
- Update：过门控的位姿或绝对位置  
- 气压：只更新 z  

## 4. 紧耦合（目标）

同一窗口内装配：

- IMU 预积分因子  
- 视觉重投影  
- 雷达点到面  
- GPS / 气压先验  

由 `backend/graph` 承担；`fusion` 层退化为残差权重与异常剔除策略。

## 5. 降级策略

| Health | 条件（简） | 推荐 |
|--------|------------|------|
| `kFull` | 图像+雷达 OK | LIVO |
| `kLioOnly` | 仅雷达 OK | LIO |
| `kVioOnly` | 仅图像 OK | VIO |
| `kImuCoast` | 视觉激光皆失效 | IMU 递推 +（可选）GPS/气压 |

协方差过大（`trace(cov) > 阈值`）视为模态失效，强制降级。

## 6. 与状态机

`SlamSystem` 在非 Full（且配置为 LIVO）或 IMU coast 时调用 `OnDegraded()`，供飞控侧切换控制律。

## 7. 后续工作

- [ ] 运行时真正切换 `Frontend` 实例  
- [ ] 紧耦合 LIVO 因子装配  
- [ ] GPS ENU 初始化与航向对齐  
- [ ] 门控阈值进 YAML  
