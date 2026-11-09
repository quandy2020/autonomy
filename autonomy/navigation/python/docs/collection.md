# 数据采集

对齐 VNT-PA 的三阶段数据管线：

```text
explore -> tours
tokens  -> frozen DeFM patch tokens
bank    -> evaluation queries / action chunks
```

## 1. HM3D

HM3D 需要先在官网接受许可后手动下载，仓库不提供自动下载脚本：

<https://aihabitat.org/datasets/hm3d/>

```bash
export HM3D=/data/hm3d
export DATA=/data/vntpa
```

检查目录是否完整：

```bash
./scripts/check_hm3d.sh
```

HM3D v0.2 目录：

```text
$HM3D/train/
$HM3D/val/
$HM3D/hm3d_basis.scene_dataset_config.json
```

## 2. 探索生成 tours

```bash
python -m vntpa.cli.collect explore \
  --scene-root "$HM3D/train" \
  --dataset-config "$HM3D/hm3d_basis.scene_dataset_config.json" \
  --num-scenes 800 \
  --seeds-per-scene 5 \
  --seed-offset 100 \
  --out-root "$DATA/traj/train"
```

验证集：

```bash
python -m vntpa.cli.collect explore \
  --scene-root "$HM3D/val" \
  --dataset-config "$HM3D/hm3d_basis.scene_dataset_config.json" \
  --num-scenes 50 \
  --out-root "$DATA/traj/val"
```

## 3. 提取 DeFM tokens

```bash
python -m vntpa.cli.collect tokens \
  --corpus "$DATA/traj/train" \
  --out "$DATA/tokens/train" \
  --gpu 0

python -m vntpa.cli.collect tokens \
  --corpus "$DATA/traj/val" \
  --out "$DATA/tokens/val" \
  --gpu 0
```

每个 keyframe 保存：

```text
tokens  [256,384]
pose    [3]
```

## 4. 评估 bank

```bash
python -m vntpa.cli.collect bank \
  --token-root "$DATA/tokens/val" \
  --out "$DATA/banks/bank_val.npz" \
  --per-tour 5 --queries 16 --chunk 8 --seed 1
```

## 5. 数据可视化

深度预览：

```python
import numpy as np
from vntpa.utils.visual import save

depth = np.load("depth.npy")
save("logs/depth.png", depth, max_depth=10.0)
```

DeFM 三通道：

```python
from vntpa.data.transform import channels

channel_data = channels(depth)  # (3,H,W), [0,1]
```

TensorBoard 图像：

```python
from torch.utils.tensorboard import SummaryWriter
from vntpa.utils.visual import log

writer = SummaryWriter("logs/viz")
log(writer, "data/depth", depth, step=0)
writer.close()
```

查看 token store：

```bash
python -m vntpa.cli.validate --root "$DATA/tokens/val"
```

## 6. 结构

```text
$DATA/
├── traj/train/
├── traj/val/
├── tokens/train/
├── tokens/val/
└── banks/bank_val.npz
```
