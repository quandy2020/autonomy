# Automanip

机械臂 HAL，包结构对齐 [`autodriver`](../autodriver/README.md)：YAML → Manager → Backend Registry → Driver，经 Autolink 收发。

控制律是 **osc2**：移植 [OCS2](https://github.com/leggedrobotics/ocs2) 固定基座机械臂的运动学 MPC（`ocs2` 是别名）。状态是关节位置，输入是关节速度，代价是末端位姿误差 + 输入正则 + 关节限位软约束。求解用短时域 Gauss-Newton，不依赖 Pinocchio / CppAD。

## 一览

| 域 | 路径 | 做什么 |
|---|---|---|
| 运动学 | `arm/chain.*` | 串联旋转关节 FK + 几何雅可比 |
| 控制 | `arm/osc2/` | 运动学 MPC，backend `osc2` / `ocs2` |
| 本体 | `arm/stub/` | 只积分关节指令，backend `stub` / `sim` |
| 编排 | `arm/arm_manager.*` | 话题、看门狗、周期 `Step` |
| 启动 | `launch/` · `dag/` | 进程，或 mainboard DAG |

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
cmake --build build/autonomy -j"$(nproc)" --target automanip automanip_main test_osc2 test_arm_driver test_automanip_config

export AUTOMANIP_PATH=$PWD/src/autonomy/automanip
export LD_LIBRARY_PATH=$PWD/build/autonomy/lib:$LD_LIBRARY_PATH
export PATH=$PWD/build/autonomy/bin:$PATH
automanip -n
automanip
```

只跑运动学核心（不链 Autolink）：

```bash
cd src/autonomy/automanip
bazel test //:test_osc2
```

DAG：`mainboard -d $AUTOMANIP_PATH/dag/arm_osc2.dag`。不要和 `automanip` 进程同时跑。

## 加厂商

1. `arm/<vendor>/driver.{hpp,cpp}` 实现 `ArmDriver`
2. `REGISTER_ARM_BACKEND(mybot, "mybot", CreateMyBotDriver, "");`
3. YAML：`backend: mybot`

厂商读真实关节、写 `JointCommand`。末端跟踪仍可用进程里的 `backend: osc2`（仿真链），或在厂商驱动里调用 `osc2::Osc2Controller`。

## 许可证

Apache-2.0。osc2 是 OCS2 运动学问题的独立实现，不是 OCS2 源码拷贝。
