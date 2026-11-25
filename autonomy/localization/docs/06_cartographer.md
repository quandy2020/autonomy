# 6. Cartographer 2D

面向 AutoSim / Habitat 平面激光：输入 `/scan`、`/odom`、`/tf`、`/tf_static`。配置 `autonomy/localization/conf/cartographer/autosim_2d.lua`。停进程时把状态写到 `data/maps/cartographer.pbstream`（相对源码包根 `autonomy/` 的 `data/maps`）。

更完整的 Lua 与纯定位说明：[Cartographer 指南](../../../docs/source/06_Localization/07_cartographer/guide.md)。

## 启动

```bash
source /workspace/autonomy/install/setup.bash
cd /workspace/autonomy/src/autonomy
mkdir -p data/maps
export PATH=/workspace/autonomy/build/autonomy/bin:$PATH
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch

autolink launch start localization_cartographer.launch
```

或：

```bash
src/autonomy/autonomy/localization/scripts/start_cartographer.sh
src/autonomy/autonomy/localization/scripts/start_cartographer.sh stop
```

仿真侧使用带 `/scan` 的配置（例如默认 slam Habitat），不要用 Atlas RGB-D 场景冒充 2D 激光。

停止并写 pbstream：

```bash
autolink launch stop localization_cartographer.launch
```

加载已有地图可在 process 行增加 `--load_state_filename=data/maps/cartographer.pbstream`。换 Lua：`--configuration_basename=...`。
