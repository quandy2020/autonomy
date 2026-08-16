# NavigationService 流式 RPC 设计 — 完备客户端面

日期：2026-08-14  
状态：已批准（去冗余修订）  
范围：`navigation.proto`、`common.proto`（BUSY）、`rpcs/README.md`

## 服务面

```text
Navigate                              -> stream NavigateResponse
Pause / Resume / Replan / Cancel      -> GoalRequest -> Status
GetStatus                             -> NavigateResponse
```

**去冗余：** 删除 `active`（由 `state` 推导）；删除各控制 RPC 独立 Request/Response 与嵌入快照；`GetStatus` 直接返回 `NavigateResponse`。

**NavigateResponse：** `status`, `state`, `goal_id`, `current_pose`,  
`optional remaining_distance_meters`, `optional estimated_time_remaining_seconds`,  
`waypoint_index`, `number_of_waypoints`

**State：** UNKNOWN / IDLE / PLANNING / RUNNING / PAUSED / ARRIVED / FAILED / CANCELLED  

**Options：** `maximum_linear_speed`, `maximum_angular_speed`,  
`goal_position_tolerance_meters`, `goal_heading_tolerance_radians`, `timeout_seconds`

**Code：** `CODE_NAVIGATION_BUSY = 105`

**关流：** Cancel / FAILED / CANCELLED / 拒收 / 断开；ARRIVED 不关流。
