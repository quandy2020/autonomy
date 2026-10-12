# Automanip

机械臂规划 / 执行栈（MoveIt 同构）+ OCP 子库 [`bolza/`](bolza/)（`libbolza`）。原 `autonomy/manipulation` 已收口至此。

## 一览

| 域 | 路径 | 做什么 |
|---|---|---|
| 规划栈 | `automanip/{model,planner,pipeline,motion,dispatch,…}` | MoveIt 同构；主进程 `automanip` |
| OCP 子库 | `bolza/`（`libbolza` + [使用指南](bolza/docs/README.md)） | DDP/SQP/IPM/SLP/MPC |
| OCP 示例 | `examples/` | cartpole / ballbot / … |
| 启动 | `launch/` | 进程 |

## 进程

| 二进制 | 说明 |
|---|---|
| `automanip` | ManipulationServer（`/automanip/move`） |
| `automanip.manipulation` | 同上（兼容别名入口） |
| `automanip.manipulation.setup` | Setup CLI lite |
| `*_demo` | bolza OCP 示例（见 `examples/`） |

规划配置：`share/automanip/conf/manipulation.pb.txt`（或 `--conf`）。

## 运行

```bash
cmake --build build/autonomy -j"$(nproc)" --target automanip_main bolza

export AUTOMANIP_PATH=$PWD/src/autonomy/automanip
export LD_LIBRARY_PATH=$PWD/build/autonomy/lib:$LD_LIBRARY_PATH
export PATH=$PWD/build/autonomy/bin:$PATH
automanip -n
automanip
```

## 许可证

Apache-2.0。`bolza/` 源码保留上游 BSD 头（见 `bolza/solver_license.txt`）。
