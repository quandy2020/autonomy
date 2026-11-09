# vntpa

VNT-PA + DeFM 的单仓库、可离线训练/部署/autosim 工程。

## 结构

```text
vntpa/                      # project root
├── vntpa/
│   ├── cli/                # collect / train / evaluate / export / validate
│   ├── data/               # dataset / transform / sample / batch / collection
│   ├── engine/             # model / predictor / trainer / validator / metric
│   ├── models/             # encoder / vntpa / query / context / pose / action / head
│   ├── simulator/          # autolink / autosim bridge
│   └── utils/              # config / errors / logging / timing / visual
├── configs/                # dataset / model / train / eval / export
├── scripts/                # shell entry points
├── docs/                   # design docs
├── tests/
└── thirdparty/             # vendored VNT-PA / DeFM / frontier
```

`thirdparty/pan`、`thirdparty/defm`、`thirdparty/frontier_exploration` 已内置。

## 安装

```bash
pip install -e ".[train]"
```

Habitat-Sim 和 HM3D 需要单独准备。

## 环境

```bash
export VNT_PA_DATASET=/data/hm3d
export VNT_PA_DEFM_WEIGHTS=/path/to/defm_vit_s14.pth
```

默认 DeFM 权重：

```text
autonomy/navigation/conf/models/defm_vit_s14.pth
```

## 数据

```bash
export HM3D=/data/hm3d
export DATA=/data/vntpa

python -m vntpa.cli.collect explore \
  --scene-root "$HM3D/train" \
  --dataset-config "$HM3D/hm3d_basis.scene_dataset_config.json" \
  --num-scenes 800 --seeds-per-scene 5 --seed-offset 100 \
  --out-root "$DATA/traj/train"

python -m vntpa.cli.collect tokens \
  --corpus "$DATA/traj/train" --out "$DATA/tokens/train" --gpu 0

python -m vntpa.cli.collect bank \
  --token-root "$DATA/tokens/val" --out "$DATA/banks/bank_val.npz" \
  --per-tour 5 --queries 16 --chunk 8 --seed 1
```

## 训练

```bash
python -m vntpa.cli.train --nproc 1 \
  --tensorboard-dir logs/pan/tb \
  --run-name pan --accum 4 --wandb-project "" \
  --token-root "$DATA/tokens/train" \
  --val-token-root "$DATA/tokens/val" \
  --val-bank "$DATA/banks/bank_val.npz"
```

TensorBoard：

```bash
tensorboard --logdir logs/pan/tb --port 6006
```

恢复训练：

```bash
python -m vntpa.cli.train --nproc 1 \
  --config logs/pan/config.json --resume
```

## 评估

```bash
python -m vntpa.cli.evaluate \
  --ckpt logs/pan/checkpoints/final.pt \
  --token-root "$DATA/tokens/val" \
  --bank "$DATA/banks/bank_val.npz"
```

## 导出

```bash
python -m vntpa.cli.export \
  --checkpoint checkpoints/vntpa_vit_s14/best.pt \
  --out exports/vntpa.onnx
```

输出：

```text
exports/vntpa.onnx
exports/vntpa.onnx.json
```

## 统一 CLI

```bash
vntpa predict --model exports/vntpa.onnx --defm exports/defm_vit_s14.onnx \
  --depth depth.npy --pose 0 0 0 --goal 2 1
vntpa train --nproc 1 --run-name pan ...
vntpa evaluate --ckpt logs/pan/checkpoints/final.pt ...
vntpa export --checkpoint checkpoints/vntpa_vit_s14/best.pt --out exports/vntpa.onnx
vntpa autosim --planner exports/vntpa.onnx
vntpa benchmark --config configs/train/default.yaml \
  --depth depth.npy --pose 0 0 0 --goal 2 1
vntpa models --inspect exports/vntpa.onnx
vntpa dataset --root "$DATA/tokens/val"
vntpa info
```

## Python API

```python
from vntpa import VNT

model = VNT.load(
    "exports/vntpa.onnx",
    "exports/defm_vit_s14.onnx",
)
result = model.predict(depth, pose, goal)
print(result.actions)
```

训练/评估/导出：

```python
model.train(["--run-name", "pan", "--token-root", "..."])
model.benchmark(depth, pose, goal)
model.export("checkpoints/best.pt", "exports/vntpa.onnx")
```

低层：

```python
from vntpa import ContextMemory, OnnxPlanner, PlannerConfig

planner = OnnxPlanner("exports/vntpa.onnx", PlannerConfig())
memory = ContextMemory(max_keyframes=64)
result = planner.plan(pose, goal, memory)
```

## autosim

```bash
export PYTHONPATH=\
/path/to/automsgs/proto/gen/python:\
/path/to/autolink/autolink/python:\
/path/to/autonomy/build/autonomy/navigation/generated/python:\
$PWD

python -m vntpa.simulator.client \
  --planner exports/vntpa.onnx \
  --defm exports/defm_vit_s14.onnx \
  --goal-topic /navigation/goal
```

订阅：

```text
/camera/depth/image_raw
/odom
/navigation/goal
```

发布：

```text
/cmd_vel
```

## 测试

```bash
PYTHONPATH=. pytest -q tests
```

## 文档

- [architecture.md](docs/architecture.md)
- [training.md](docs/training.md)
- [collection.md](docs/collection.md)
- [deployment.md](docs/deployment.md)
- [autosim.md](docs/autosim.md)
- [naming.md](docs/naming.md)
