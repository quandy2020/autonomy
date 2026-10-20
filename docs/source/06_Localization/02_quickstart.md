# 2. 快速开始

### 2.1 Cartographer 2D（默认）

完整步骤：[Cartographer 指南](07_cartographer/guide.md)。

```bash
python -m autonomy.tools.bag_convert \
~/Downloads/b2-2016-04-27-12-31-41.bag \
-o ./data/records --backpack-2d

export PATH=$PWD/build/bin:$PATH
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
autolink launch start localization.launch
```

回放 record，观察 `map`。保存状态：

```bash
autonomy.localization \
--localization_mode=cartographer \
--configuration_directory=autonomy/localization/conf/cartographer \
--configuration_basename=autosim_2d.lua \
--save_state_filename=data/maps/cartographer.pbstream
```

### 2.2 Atlas

见 [Atlas](08_atlas/guide.md)。

### 2.3 Argus

见 [§6](06_argus.md)。离线回放：[Tools §8](../18_Tools/08_datasets.md)。
