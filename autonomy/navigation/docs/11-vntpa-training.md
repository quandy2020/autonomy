# 11 · VNTPA 离线训练

两条路径：

1. **冻结 bank（默认）**：不需要 Habitat。用已写好的 token store + `bank.npz` 做交叉熵模仿。合成数据可立刻跑通。
2. **论文分布式**：`thirdparty.pan.planner.train`，NCCL + CUDA，在线用 navmesh 专家打标签，默认 30000 步。

## 合成数据烟雾（无 HM3D）

```sh
cd src/autonomy/autonomy/navigation/python
python -m vntpa.cli.train --synthetic --tiny --run-name smoke --steps 20 --device cpu
```

产物：`dataset/synthetic/tokens/{train,val}`、`banks/bank_{train,val}.npz`、`logs/smoke/checkpoints/final.pt`、`logs/smoke/tb`、`logs/smoke/metrics.jsonl`。
这是契约/优化器验收，**不是**论文 SR 权重。

```sh
tensorboard --logdir logs --port 6006
python -m vntpa.cli.analyze --tokens dataset/synthetic/tokens/train \
  --bank dataset/synthetic/banks/bank_train.npz --tensorboard-dir logs/data/tb
```

## Matterport3D

可以。采集时把 `--scene-root` / `--dataset-config` 指到 Habitat 的 MP3D 即可
（`configs/dataset/habitat_mp3d.yaml`）。探索脚本已识别 `<id>/<id>.glb`。
tokens 之后的训练与 HM3D 相同；权重不能当论文 HM3D SR。

SpaceHero 镜像把 10 栋 MP3D 挂在仓库 `src/`（与 AutoSim `default.yaml` 同一套）：

```sh
export MP3D=/workspace/autonomy/src
export DATA=/workspace/autonomy/src/autonomy/autonomy/navigation/python/dataset/mp3d
# 10 栋：17DRP5sb8fy 29hnd4uzFmX 2azQ1b91cZZ S9hNv5qa7GM jtcxE69GiFV
#         kEZ7cmS4wCh mJXqzFtmKg4 q9vSo1VnCiC vyrNrziPKCB zsNo4HB9uLZ

cd /workspace/autonomy/src/autonomy/autonomy/navigation/python
PYTHONPATH=. /opt/venv/bin/python -m vntpa.cli.collect explore \
  --scene-root "$MP3D" \
  --dataset-config "$MP3D/mp3d.scene_dataset_config.json" \
  --num-scenes 10 --seeds-per-scene 5 --out-root "$DATA/traj/train"
```

完整 61 栋时把 `MP3D` 换成数据集根，`--num-scenes 61`。采集会占 GPU，先停 AutoSim 或换 `--gpu-ids`。

## 论文流程（HM3D）

```sh
export HM3D=/data/hm3d
export DATA=/data/vntpa

python -m vntpa.cli.collect explore \
  --scene-root "$HM3D/train" \
  --dataset-config "$HM3D/hm3d_basis.scene_dataset_config.json" \
  --num-scenes 800 --seeds-per-scene 5 --out-root "$DATA/traj/train"
python -m vntpa.cli.collect tokens --corpus "$DATA/traj/train" --out "$DATA/tokens/train" --gpu 0
python -m vntpa.cli.collect bank --token-root "$DATA/tokens/val" --out "$DATA/banks/bank_val.npz"

# 有冻结 bank 时，可先在单卡上训（不启 Habitat）
python -m vntpa.cli.train \
  --token-root "$DATA/tokens/train" --train-bank "$DATA/banks/bank_train.npz" \
  --val-token-root "$DATA/tokens/val" --val-bank "$DATA/banks/bank_val.npz" \
  --run-name pan --steps 30000 --accum 4

# 论文原版：在线专家 + DDP
python -m vntpa.cli.train --distributed --nproc 1 --accum 4 --wandb-project '' \
  --token-root "$DATA/tokens/train" \
  --val-token-root "$DATA/tokens/val" \
  --val-bank "$DATA/banks/bank_val.npz" \
  --run-name pan
```

导出：

```sh
python -m vntpa.cli.export --checkpoint logs/pan/checkpoints/final.pt \
  --out ../conf/models/vntpa.onnx --context-size 64
```

随机/合成权重不能计入导航 SR 验收。C++ 输入为 Habitat `(x, z, yaw)`。
