# autosim 验证

## 1. 启动 autosim

```bash
CONFIG=autosim/config/habitat_people.yaml \
  autolink launch start autosim/launch/autosim.launch
```

确认启用：

```text
camera.enabled: true
depth_channel: /camera/depth/image_raw
odom.channel: /odom
cmd_vel: /cmd_vel
```

## 2. 启动 bridge

```bash
export PYTHONPATH=\
/path/to/automsgs/proto/gen/python:\
/path/to/autolink/autolink/python:\
/path/to/autonomy/build/autonomy/navigation/generated/python:\
$PWD

python -m vntpa.runtime.sim.autosim_client \
  --planner exports/vntpa.onnx \
  --defm exports/defm_vit_s14.onnx \
  --goal-topic /navigation/goal
```

## 3. 发布目标

autosim 不发布导航目标，需要外部 goal sender：

```bash
navigation_goal_sender \
  --x 3.0 --y 1.0 --yaw 0.0 \
  --topic /navigation/goal
```

或由 navigation/behavior 发布 `navigation.GoalSpec`。

## 4. 数据流

```text
/camera/depth/image_raw -> DeFM -> tokens
/odom                   -> (x,y,yaw)
/navigation/goal        -> (x,y)
        │
        ▼
VNT-PA -> actions -> /cmd_vel
```

## 5. 坐标系

- `/odom` 得到 odom 帧 pose；
- goal 通常在 map 帧；
- 若 `map→odom` 非单位变换，必须做 TF 转换；
- 不要直接把 map 坐标当 odom 坐标使用。

## 6. 动作

| 动作 | 输出 |
| --- | --- |
| STOP | 0 |
| FORWARD | 0.5 m/s |
| TURN_LEFT | +1.047 rad/s |
| TURN_RIGHT | -1.047 rad/s |

持续 `action_dt_s=0.5`。

## 7. 验证指标

至少记录：

```text
success rate
SPL
stop distance
collision count
episode length
inference latency p50/p95
```

## 8. 已知差距

- 部署 keyframe 选择：距离/时间，最多 64 帧；
- 训练 coverage gate：最多 768 帧；
- 未接 C++ safety shield；
- 未实现 navigate_to_pose Action；
- 未做 autosim 闭环指标。

## 9. 故障排查

| 现象 | 原因 |
| --- | --- |
| 无 `/cmd_vel` | autolink/automsgs 未加载，或没有 goal |
| 不动 | depth encoding 不支持，检查 32FC1/16UC1 |
| 方向错误 | odom/map 帧不一致 |
| planner 不加载 | `vntpa.onnx` 缺失或 I/O 名称不匹配 |
| 不停止 | 模型未训练 STOP，或 goal 太远 |
