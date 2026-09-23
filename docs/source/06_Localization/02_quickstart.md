# 2. 快速开始

### 2.1 Cartographer 2D SLAM（默认，推荐）

完整说明见 [Cartographer 使用指南](cartographer/guide.md)。

**三步流程：**

1. 转换 bag（Backpack 2D 数据集）：

```bash
python -m autonomy.tools.bag_convert \
  ~/Downloads/b2-2016-04-27-12-31-41.bag \
  -o ./data/records --backpack-2d
```

2. 启动 SLAM：

```bash
export PATH=$PWD/build/bin:$PATH
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
autolink launch start localization.launch
# 或：localization_cartographer.launch
```

3. 回放 record，观察 `map` 话题与地图输出。

保存地图状态：

```bash
autonomy.localization \
  --localization_mode=cartographer \
  --configuration_directory=autonomy/localization/conf/cartographer \
  --configuration_basename=autosim_2d.lua \
  --save_state_filename=data/maps/cartographer.pbstream
```

---

### 2.2 Lightning LIO

```bash
autonomy.localization \
  --localization_mode=lightning \
  --lightning_config=autonomy/localization/conf/lightning/autosim.yaml \
  --lightning_imu_topic=/imu \
  --lightning_lidar_topic=/points
```

或：`autolink launch start localization_lightning.launch`

---

### 2.3 AMCL 配置预览（待集成）

```lua
-- config/localization/localization.lua
AUTONOMY_LOCALIZATION = {
    default_algorithm = "amcl",
    enabled = true,
    amcl = AMCL_OPTIONS,  -- 见 config/localization/amcl/amcl.lua
}
```

AMCL 集成后典型启动顺序：`MapServer` 发布 `/map` → 里程计 + `/scan` → AMCL 发布 `map→odom` TF。详见 [§7 AMCL](07_amcl.md)。
