# vntpa

VNT-PA + DeFM 的**单仓库、可离线**训练/部署/autosim 桥接工程。

## 结构

```text
vntpa/                      # project root
├── vntpa/
│   ├── data/               # dataset / transforms + collection/
│   ├── models/             # encoder / vntpa / builders / heads
│   ├── engine/             # trainer / validation / inference / deployment
│   ├── simulator/          # autolink / autosim
│   ├── cli/                # collect / train / eval / export / validate
│   └── utils/              # config / errors / logging / geometry
├── configs/                # dataset / model / train / eval / export
├── scripts/                # shell entry points
├── docs/                   # six design docs
├── tests/
└── thirdparty/             # vendored VNT-PA / DeFM / frontier
```

`thirdparty/pan`、`thirdparty/defm`、`thirdparty/frontier_exploration` 已内置；运行时不再依赖
`$VNT_PA_ROOT` / `$DEFM_ROOT`。

## 安装

```bash
pip install -e ".[train]"
```

Habitat-Sim 需要单独安装（conda/预编译 wheel），HM3D 数据集也需本地准备。

## 离线资源

```bash
export VNT_PA_DATASET=/data/hm3d            # HM3D 数据根
export VNT_PA_DEFM_WEIGHTS=/path/to/defm_vit_s14.pth
```

默认权重路径为：

```text
autonomy/navigation/conf/models/defm_vit_s14.pth
```

`.pth`/`.onnx` 已 gitignore，部署时放入 `conf/models/`。

## 训练

```bash
python -m vntpa.cli.train --nproc 1 \
  --tensorboard-dir logs/pan/tb \
  --run-name pan \
  --token-root $VNT_PA_DATASET/tokens/train \
  --val-token-root $VNT_PA_DATASET/tokens/val \
  --val-bank $VNT_PA_DATASET/banks/bank_val.npz
```

数据采集：

```bash
python -m vntpa.cli.collect explore ...
python -m vntpa.cli.collect tokens ...
python -m vntpa.cli.collect bank ...
```

## 评估

```bash
python -m vntpa.cli.evaluate \
  --ckpt logs/pan/checkpoints/final.pt \
  --token-root $VNT_PA_DATASET/tokens/val \
  --bank $VNT_PA_DATASET/banks/bank_val.npz
```

TensorBoard：

```bash
tensorboard --logdir logs/pan/tb --port 6006
```

详细流程见 [docs/training.md](docs/training.md) 与 [docs/collection.md](docs/collection.md)。

## 导出

```bash
python -m vntpa.cli.export \
  --checkpoint checkpoints/vntpa_vit_s14/best.pt \
  --out exports/vntpa.onnx
```

## 统一 CLI

```bash
vntpa predict --model conf/models/vntpa.onnx --defm conf/models/defm_vit_s14.onnx \
  --depth depth.npy --pose 0 0 0 --goal 2 1
vntpa export --checkpoint logs/pan/checkpoints/final.pt --out conf/models/vntpa.onnx
vntpa train --nproc 1 --run-name pan ...
vntpa val --ckpt logs/pan/checkpoints/final.pt ...
vntpa autosim --planner conf/models/vntpa.onnx
vntpa dataset --root $VNT_PA_DATASET/tokens
vntpa benchmark --config configs/train/default.yaml --depth depth.npy --pose 0 0 0 --goal 2 1
vntpa models --inspect conf/models/vntpa.onnx
vntpa info   # version + runtime environment
```

## Python 高层 API

```python
from vntpa import VNT

model = VNT.load(
    "conf/models/vntpa.onnx", "conf/models/defm_vit_s14.onnx")
result = model.predict(depth, pose, goal)
print(result.actions)
```

低层部署 API：

```python
from vntpa import ContextMemory, OnnxPlanner, PlannerConfig

planner = OnnxPlanner("conf/models/vntpa.onnx", PlannerConfig())
memory = ContextMemory(max_keyframes=64)
result = planner.plan(pose, goal, memory)
```

## autosim

```bash
export AUTOMSGS_PYTHON=/path/to/automsgs/proto/gen/python
export AUTOLINK_PYTHON=/path/to/autolink/autolink/python
export NAVIGATION_PYTHON=/path/to/autonomy/build/autonomy/navigation/generated/python
export PYTHONPATH=$AUTOMSGS_PYTHON:$AUTOLINK_PYTHON:$NAVIGATION_PYTHON:$PWD

python -m vntpa.simulator.client \
  --planner conf/models/vntpa.onnx \
  --defm conf/models/defm_vit_s14.onnx \
  --goal-topic /navigation/goal
```

订阅：

- `/camera/depth/image_raw`
- `/odom`
- `/navigation/goal`

发布：

- `/cmd_vel` `TwistStamped`

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
