# 2. 快速开始

## 背景

按传感器选一条后端；细节分别见 [Cartographer](07_cartographer/guide.md)、[Atlas](08_atlas/guide.md)、[Argus](06_argus.md)。

## 思路

按固定顺序执行：依赖/环境 → 构建或配置 → 启动 → 最小验证；分支放到文末链接。


## 步骤

### 1. Cartographer 2D（默认）

Backpack 示例：bag 转 record 再起 SLAM。

```bash
python -m autonomy.tools.bag_convert \
  ~/Downloads/b2-2016-04-27-12-31-41.bag \
  -o ./data/records --backpack-2d

export PATH=$PWD/build/bin:$PATH
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
autolink launch start localization.launch
```

回放 record，看 `map` 与 TF。退出时保存状态：

```bash
autonomy.localization \
  --localization_mode=cartographer \
  --configuration_directory=autonomy/localization/conf/cartographer \
  --configuration_basename=autosim_2d.lua \
  --save_state_filename=data/maps/cartographer.pbstream
```

### 2. Atlas（视觉 / VIO）

```bash
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
autolink launch start localization_atlas.launch
```

默认是单目模式；双目、RGB-D 或惯性模式必须同时修改模式、标定 YAML 和输入话题。详见 [Atlas 指南](08_atlas/guide.md)。

### 3. Argus（ESIKF LIO / LIVO）

```bash
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
autolink launch start localization_argus.launch
```

离线 Oxford 等序列：[Tools · 数据集](../18_Tools/08_datasets.md)、[06 Argus](06_argus.md)。
