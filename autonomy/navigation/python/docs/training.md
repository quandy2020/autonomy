# 训练

对齐 [VNT-PA](https://github.com/beimingli0626/VNT-PA) 的训练流程。

## 1. 环境

Linux + NVIDIA GPU，Python 3.10：

```bash
conda create -n pan python=3.10 -y
conda activate pan

conda install -y -c conda-forge -c aihabitat-nightly \
  habitat-sim=0.3.3 withbullet headless

pip install torch==2.5.1 torchvision==0.20.1 \
  --index-url https://download.pytorch.org/whl/cu121

cd autonomy/navigation/python
pip install -e ".[train]"
```

`numpy-quaternion`、`h5py`、Habitat-Sim 是训练必需。

## 2. DeFM 权重

上游首次使用时从 Hugging Face 下载；本仓库已改为本地：

```bash
export VNT_PA_DEFM_WEIGHTS=/path/to/defm_vit_s14.pth
```

默认：

```text
autonomy/navigation/conf/models/defm_vit_s14.pth
```

## 3. 数据

```bash
export HM3D=/data/hm3d
export DATA=/data/vntpa
```

详细命令见 [collection.md](collection.md)。

## 4. 训练

```bash
python -m vntpa.cli.train --nproc 1 \
  --tensorboard-dir logs/pan/tb \
  --run-name pan \
  --accum 4 \
  --wandb-project "" \
  --token-root "$DATA/tokens/train" \
  --val-token-root "$DATA/tokens/val" \
  --val-bank "$DATA/banks/bank_val.npz"
```

等价上游：

```bash
torchrun --nproc_per_node=1 -m thirdparty.pan.planner.train ...
```

默认约 4 小时 / L40S。小显存：

```bash
--batch-scenes 6 --accum 8
```

## 5. TensorBoard

冻结-bank 训练默认写 `logs/<run-name>/tb`（`--tensorboard-dir` 可改；空字符串关闭）。

Time series 首页如果全是单点，那是 `data/` / `bank/` 的 step=0 快照。曲线在 **`train/loss`、`val/ce`**。横轴选 **Step**，左侧只勾一个 run（不要勾带时间戳的 hparams 子目录）。从 jsonl 重建曲线：

```bash
python -m vntpa.utils.tensorboard --jsonl logs/pan-mp3d/metrics.jsonl --out logs/pan-mp3d/curves
tensorboard --logdir logs/pan-mp3d/curves --bind_all --port 6006
```

```bash
python -m vntpa.cli.train --synthetic --tiny --run-name smoke --steps 20 --device cpu
tensorboard --logdir logs/pan-mp3d/curves --bind_all --port 6006
```

训练标量：

```text
setup/n_params, setup/n_train, setup/n_val
train/loss, train/loss_raw, train/acc0, train/lr, train/sps, train/grad_norm
train/kf_mean, train/live_targets
val/ce, val/acc0, val/acc_slots_*
```

语料分析（采集后、训练前）：

```bash
python -m vntpa.cli.analyze \
  --tokens "$DATA/tokens/train" \
  --bank "$DATA/banks/bank_train.npz" \
  --tensorboard-dir logs/data/tb \
  --out logs/data/report.json
```

```text
data/n_tours, data/frames/*, data/coverage_frac/*, data/path_length_m/*
data/hist/frames, data/hist/coverage_frac, data/tours
bank/n_rows, bank/geo0/*, bank/action_stop..action_turn_right, bank/pad_frac
```

论文 DDP 额外：

```text
train/step_ms
train/data_wait_ms
val/entry_rank
cl/success
cl/spl
```

实现位于 `thirdparty/pan/planner/train.py::make_logger`。

## 6. 恢复训练

```bash
python -m vntpa.cli.train --nproc 1 \
  --config logs/pan/config.json --resume
```

`init-from`：

```bash
python -m vntpa.cli.train --nproc 1 \
  --init-from logs/old/checkpoints/final.pt
```

## 7. Checkpoint

```text
logs/<run>/checkpoints/last.pt
logs/<run>/checkpoints/final.pt
logs/<run>/checkpoints/step_*.pt
logs/<run>/config.json
logs/<run>/metrics.jsonl
```

提升到产品目录：

```bash
python -m vntpa.cli.train --nproc 1 \
  --checkpoint-out checkpoints/vntpa_vit_s14/best.pt \
  --manifest checkpoints/vntpa_vit_s14/manifest.json ...
```

## 8. 常见问题

| 问题 | 处理 |
| --- | --- |
| `quaternion` 缺失 | `pip install numpy-quaternion` |
| `habitat_sim` 缺失 | 安装 headless habitat-sim 0.3.3 |
| DeFM 权重缺失 | 设置 `VNT_PA_DEFM_WEIGHTS` |
| CUDA OOM | 降 `--batch-scenes`，升 `--accum` |
| 无 TensorBoard | 安装 `tensorboard`，确认 `--tensorboard-dir` |
