# Automanip

机械臂 HAL，包结构对齐 [`autodriver`](../autodriver/README.md)：YAML → Manager → Backend Registry → Driver，经 Autolink 收发。求解器在 `automanip/{core,oc,mpc,ddp,qp,slp,ipm,mpcnet,sqp,pinocchio,tools,perceptive}`。

## 一览

| 域 | 路径 | 做什么 |
|---|---|---|
| 运动学 | `automanip/arm/chain.*` | 串联旋转关节 FK + 几何雅可比 |
| 编排 | `automanip/arm/arm_manager.*` | 话题、看门狗、周期 `Step` |
| 示例 | `examples/` | cartpole、ballbot、double integrator、quadrotor，经 Autolink 给 autoviz |
| 启动 | `launch/` | 进程 |

## 通道

| 方向 | 类型 | 默认 |
|---|---|---|
| 末端目标 | `geometry_msgs.PoseStamped` | `/arm/target_pose`（进入 `track`） |
| 关节指令 | `control_msgs.JointCommand` | `/arm/joint_command`（`position` 或 `velocity`，切入 `hold`） |
| 模式 | `std_msgs.String` | `/arm/mode`：`hold` `home` `track` `estop` `clear_estop` |
| 关节状态 | `sensor_msgs.JointState` | `/arm/joint_states` |
| 末端位姿 | `geometry_msgs.PoseStamped` | `/arm/ee_pose` |
| 模式 / 能力 / 事件 | `std_msgs.String` | `/arm/mode_state` `/arm/capability` `/arm/event` |

`track` 下超过 `watchdog_ms` 没有新的末端目标会回到 `hold`（停速度，保持当前位置）。

## 运行

```bash
cmake --build build/autonomy -j"$(nproc)" --target automanip automanip_main test_automanip_config

export AUTOMANIP_PATH=$PWD/src/autonomy/automanip
export LD_LIBRARY_PATH=$PWD/build/autonomy/lib:$LD_LIBRARY_PATH
export PATH=$PWD/build/autonomy/bin:$PATH
automanip -n
automanip
```

默认 `arm.enable: false`。注册厂商驱动后把 `arm.enable` 设为 `true`，并把 `arm.backend` 设成注册名。

## 加厂商

1. `automanip/arm/<vendor>/driver.{hpp,cpp}` 实现 `ArmDriver`
2. `REGISTER_ARM_BACKEND(mybot, "mybot", CreateMyBotDriver, "");`
3. YAML：`enable: true`，`backend: mybot`

厂商读真实关节、写 `JointCommand`。

## 许可证

Apache-2.0。`automanip/{core,oc,mpc,ddp,qp,slp,ipm,mpcnet,sqp,pinocchio,tools,perceptive}` 保留上游 BSD 头。`automanip/ipm/lq_solver.*` 是本工程的离散 Riccati 后端，IPM 和 SQP 的二次子问题都用它，不依赖 HPIPM。MPC-Net 的 ONNX 控制器在找到 ONNX Runtime 头文件时才编进库。Pinocchio 接口、质心模型和碰撞球近似在找到 pinocchio 与 hpp-fcl 时才编进库。
