# LocalizationService 精细化

日期：2026-08-14  
状态：已落地  
范围：`localization.proto`、`common.proto`（803）、cli README

## 服务面

```text
GetPose         → GetPoseResponse          # PoseWithCovariance + confidence
GetStatus       → LocalizationStatus
SetInitialPose  → Status                   # PoseWithCovariance seed
```

**State：** UNKNOWN / LOCALIZED / LOST / INITIALIZING  

**增强：** 协方差进 `PoseWithCovariance`；`optional confidence`；`LocState` → `LocalizationState`；BUSY=803。
