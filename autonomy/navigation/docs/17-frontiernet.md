# 17 · FrontierNet 式探索与 DeFM 深度编码器

> 目标：在现有 navigation 框架上做“FrontierNet 式探索 + 更强的深度编码器”。
> 落地顺序：**先跑通 FrontierNet 推理 → 换成 DeFM encoder、只重训 decoder 两个头 →
> 用官方 checkpoint 的输出做对齐基线**。
>
> 本文档对应第 3 步的代码骨架：`python/frontiernet/`。

## 1. 官方实现的关键事实

来源：`cvg/FrontierNet`（`main` 分支，2026-11 拉取）。仓库**没有训练代码**，
只有推理和探索逻辑，作者在 issue 中也确认过，所以“重训两个头”必须自己写。

| 项目 | 事实 |
| --- | --- |
| 模型 | `TwoHeadUnet`：ResNet-34 encoder + 5 段 UNet decoder + 两个头 |
| 输入 | `224×224`，`in_channels=4`（RGB + 归一化深度） |
| 分类头 | `classes=11`，输出 `[B,11,224,224]` logits |
| 回归头 | `[B,1,224,224]` 距离场，`regression_downsample_factor=1.0` |
| 权重 | `rgbd_11cls.pth`（Google Drive `11SugqEg3LR2voKdLvq9Xe_zch10ek006`） |
| 预处理 | 按 `scale` 缩放 → 中心裁剪到 224 → RGB/255，深度**逐帧 min-max** 归一化 |
| 后处理 | `D` 反归一化 `exp(-d)·10` 后 `< 0.5 m` 为 frontier；`Y` argmax 后查 log1p 增益 bin 中心再 `expm1` |
| 依赖 | 官方用 `segmentation_models_pytorch==0.3.3` 的 encoder/decoder 组件 |

关键点：**decoder 和两个头与 encoder 解耦**。官方 checkpoint 里
`decoder.*`、`segmentation_head.*`、`df_regression_head.*` 的键名与形状只取决于
`decoder_channels=(256,128,64,32,16)` 和 `classes`，与 encoder 是 ResNet 还是 ViT 无关。
这就是“换 encoder、只重训两个头”能成立的原因。

## 2. 为什么能直接搬 decoder

官方 decoder 的输入契约（`encoder_channels[1:]` 反转后）：

```text
head  512 ch @ stride 32   (7×7)
skip  256 ch @ stride 16   (14×14)
skip  128 ch @ stride 8    (28×28)
skip   64 ch @ stride 4    (56×56)
skip   64 ch @ stride 2    (112×112)
```

只要新 encoder 按 `[input, s2, s4, s8, s16, s32]` 吐出这 5 个特征图，
`UnetDecoder` 和两个头可以**原样**使用，权重逐位迁移。
`frontiernet/unet.py` 复刻了 smp 0.3.3 的模块命名（`decoder.blocks.0.conv1.0.weight` 等），
所以 `load_state_dict(..., strict=True)` 能直接吃官方 `rgbd_11cls.pth`。

## 3. DeFM 替换方案

DeFM ViT-S/14 在 224×224 输入下只有一种分辨率：`16×16` 个 token、宽度 384，
没有 CNN 的层级金字塔。`frontiernet/encoder_defm.py` 用两步补上：

```text
DeFM ViT-S/14
  └─ get_intermediate_layers(n=[2,4,6,8,11], reshape=True)
       → 5 × (B, 384, 16, 16)        浅 → 深，语义逐级增强
  └─ ViTToUnetAdapter
       → GroupNorm(1,384) → 1×1 Conv → 双线性重采样
       → (B,64,112,112) (B,64,56,56) (B,128,28,28) (B,256,14,14) (B,512,7,7)
  └─ 官方 UnetDecoder + SegmentationHead + DfRegressionHead
```

设计取舍：

- **adapter 必须训练**。它没有预训练权重，所以“冻结 encoder”只冻结 DeFM backbone，
  adapter 与 decoder/两个头一起训（`parameter_groups` 默认给 adapter 单独 lr）。
- **几何是近似的**。ViT patch=14 的网格是 stride 14，UNet 期望 stride 32，
  重采样是几何近似而非精确对齐。因为不复用任何官方 encoder 权重，这不影响
  checkpoint 迁移，但会影响 frontier 像素的空间精度，需要在验收时观察。
- **RGB 不再进入模型**。DeFM 是纯深度编码器，所以 DeFM 变体是 `in_channels=3`。
  如果后续发现 RGB 有用，再加一条 RGB 支路，而不是回退到 ResNet。

## 4. 代码结构

| 文件 | 作用 |
| --- | --- |
| `python/frontiernet/preprocess.py` | 两套预处理契约：DeFM 对数深度（与 C++ `DepthPreprocessor` 一致）和官方 RGB-D（逐帧 min-max） |
| `python/frontiernet/postprocess.py` | 两个头解码成 `FrontierMap`（region/gain/labels/df/logits），纯 numpy |
| `python/frontiernet/unet.py` | 官方 `TwoHeadUnet` 复刻（无 smp 依赖），键名与 smp 0.3.3 对齐 |
| `python/frontiernet/encoder_defm.py` | DeFM ViT 包装 + 五级特征 adapter + `DefmEncoder` |
| `python/frontiernet/model.py` | `FrontierNetDeFM`：encoder 替换、冻结策略、优化器分组 |
| `python/frontiernet/checkpoint.py` | 官方权重移植：只取 `decoder/segmentation_head/df_regression_head`，缺键/形状不符显式报错 |
| `python/frontiernet/infer.py` | `FrontierPredictor`：按 `model.n_channels` 自动选预处理 |
| `python/frontiernet/cli.py` | `smoke` / `baseline` / `align` 三个命令 |
| `python/tests/frontiernet/` | 预处理、后处理、UNet 形状与键名、adapter、权重移植、冻结策略 |

`frontiernet/__init__.py` 只直接导入 numpy 部分，torch 部分通过 `__getattr__` 懒加载，
所以在没有 torch 的环境里也能验证预处理/后处理契约。

## 5. 命令

```bash
cd autonomy/navigation/python

# 1) 形状自检：synthetic 不需要 DeFM 依赖，defm 会真的构建 ViT
python3 -m frontiernet.cli smoke --backbone synthetic
python3 -m frontiernet.cli smoke --backbone defm --defm-weights ../conf/models/defm_vit_s14.pth

# 2) 官方 checkpoint 基线（深度用 autosim 的 /camera/depth 帧导出成 .npy）
python3 -m frontiernet.cli baseline \
  --weights /path/to/rgbd_11cls.pth \
  --depth frame_0001.npy \
  --rgb frame_0001.png \
  --out frontiernet_out --name hm3d_1

# 3) DeFM 变体 + 官方 decoder/heads，与基线对比
python3 -m frontiernet.cli align \
  --weights /path/to/rgbd_11cls.pth \
  --defm-weights ../conf/models/defm_vit_s14.pth \
  --depth frame_0001.npy \
  --rgb frame_0001.png \
  --out frontiernet_out --name hm3d_1
```

`align` 输出 `*_align.json`：checkpoint 迁移统计、region IoU、label 一致率、
距离场平均绝对差。**换 encoder 之后 IoU 低是预期结果**，这个数字的意义是
“重训前的起点”，重训后再跑同一条命令看提升。

## 6. 已验证（2026-11-09，容器 SpaceHero `/opt/venv`）

与官方源码做了逐位对比：官方 `unet_resnet.py` / `unet_parts.py` 原样使用，
`segmentation_models_pytorch` 用 0.3.3 的真实 `base/modules.py`、
`base/initialization.py`、`encoders/resnet.py` 打薄壳。

| 检查 | 结果 |
| --- | --- |
| state_dict 键 | 官方 308 / 本实现 308，差异 0 |
| 张量形状 | 不一致 0 |
| 参数量 | 24,533,068（两边相同） |
| `load_state_dict(strict=True)` | 通过 |
| 前向输出 | `df` 与 `masks` 的 `max|diff|` 均为 `0.000e+00` |
| DeFM 权重移植 | `loaded=92 skipped=216 missing=0 unexpected=0` |
| 单测 | `42 passed`（`tests/frontiernet`） |
| `smoke --backbone defm` | 25,278,668 参数，其中可训练 3,643,340（adapter+decoder+heads） |
| `baseline` / `align` | 端到端跑通，输出 npz + json |

唯一行为偏差：输入边长不是 32 的倍数时，官方 `forward` 直接抛
`RuntimeError`（其 pad 分支是死代码），本实现改为 pad 到 32 的倍数再裁回原尺寸。

复现命令（容器内）：

```bash
docker exec SpaceHero bash -lc "cd /workspace/autonomy/src/autonomy/autonomy/navigation/python \
  && /opt/venv/bin/python -m pytest tests/frontiernet -q"
```

`tests/frontiernet/state_dict_keys.txt` 固化了这 308 个键名，任何改动导致键名漂移
都会让 `test_state_dict_contract.py` 失败。

## 7. 已知限制

- 官方仓库没有训练代码，训练循环、数据加载、增益标签生成都要自己补。
- 官方仓库**没有 LICENSE 文件**，`rgbd_11cls.pth` 的再分发许可需要确认；
  当前建议只在本地缓存，不提交进仓库。
- DeFM 变体丢掉了 RGB，纯深度输入在无纹理走廊里可能不如 RGB-D。
- adapter 的 stride 14→32 重采样是近似，frontier 像素精度需实测。
- 当前已有 `FrontierNetModel` / `FrontierNetFrontierSource` C++ 加载与深度投影源码，
  `simulation` 策略启用 `policy.frontiernet` 时加载 ONNX；map 有效时优先使用地图 frontier。
- adapter 已按第 9 节重训（`conf/models/frontiernet_defm.onnx`），但闭环里模型分支
  仍被地图分支挡住，见第 8 节第 2 条；训练质量验收用的是离线 IoU / df MAE / ONNX 对齐。

## 8. 下一步

1. 扩充数据：现在只有一间屋子、原地旋转 48 s 的 240 帧；换场景/换楼层后再看 IoU。
2. 闭环里让模型真正生效（已完成）：`policy.cpp` 的 `ExplorationFrontiers` 现在
   先取地图 frontier，地图没有候选时回退到 DeFM；DeFM 目标在地图 `PathTo` 失败时
   直接沿深度射线转向（`mode=model_direct`），不再被起点格膨胀判据丢掉。
3. 用 `frontiernet.explorer`（DeFM 变体）跑 600 s 闭环，用位移/覆盖面积验收。

## 9. 训练：冻结 DeFM，只训 adapter + decoder + 两个头

### 9.1 为什么必须训练

未训练的 DeFM 变体（官方 decoder/heads + 随机 adapter）在真实一帧上：

```text
df  min 0.000  max 0.969  mean 0.178      # 阈值 log(20)=2.996
frontier 像素 0
C++ 侧: [frontiernet] pixels=0 clusters=0 depth_ok=1 model_ready=1
```

预处理和推理都是通的（`depth_ok=1 model_ready=1`），空结果的原因是 adapter
没有把 ViT 特征对齐到 decoder 的输入分布。所以“换 encoder”必须配一次重训。

### 9.2 三步流水线

| 步骤 | 模块 | 产物 |
| --- | --- | --- |
| 采集 | `frontiernet.collect` | `frames/<idx>.npz`：`depth`(float32 米)、`rgb`(uint8)、`K`、`stamp`、`pose` |
| 打标签 | `frontiernet.label` | `labels/<idx>.npz`：`df`(float16)、`labels`(uint8)，另写 `labels.json` 直方图 |
| 训练 | `frontiernet.train` | `best.pth` / `last.pth` / `train_log.jsonl` / `summary.json` |

教师是官方 RGB-D checkpoint，只在训练主机上跑一次，**不进部署图**；
部署的仍然是 `depth[1,3,224,224] → df, masks` 的 DeFM 图。

```bash
cd autonomy/navigation/python

# 1) 采集（容器内，需要 autolink）
PYTHONPATH=.:$AUTOLINK_PY /opt/venv/bin/python -m frontiernet.collect \
  --out data/frontiernet/frames --count 240 --hz 5

# 2) 教师伪标签
/opt/venv/bin/python -m frontiernet.label \
  --frames data/frontiernet/frames --out data/frontiernet/labels \
  --weights conf/models/rgbd_11cls.pth --device cuda --batch-size 8

# 3) 训练
/opt/venv/bin/python -m frontiernet.train --config configs/train/frontiernet_defm.yaml
```

### 9.3 关键坑：标签必须落在 DeFM 的像素栅格上

官方预处理是 `scale 0.5 → 中心裁 224`，640×480 只取中间 448×448；
DeFM 分支是把整帧拉伸到 224×224。两者差一个固定的横向压缩，
拿 letterbox 标签训练时 adapter 学不动：

| 标签几何 | 8 epoch 后 val region IoU | 真实帧 `df max` | 真实帧 frontier 像素 |
| --- | --- | --- | --- |
| 官方 letterbox | 0.432 | 2.94（< 2.996） | 0 |
| `frontiernet_preprocess_aligned` | 0.582 | 13.87 | 2353 |

`frontiernet_preprocess_aligned` 用与 `defm_preprocess` 完全相同的 `resize`
把 RGB 和深度拉伸到 224，教师输出因此和学生对齐。教师本身对拉伸输入是稳的
（同一帧 letterbox 220 像素 / 拉伸 771 像素，都是有效前沿）。

### 9.4 损失

- 距离场：`SmoothL1(beta=0.1)`，前沿带（< 1 m，即 `df > log(10)`）权重 `1 + 6`
- 分类：`CrossEntropy(label_smoothing=0.05)` + 逆频率类权重（`power=0.5`，clip `[0.2, 5]`）
- 两项权重 `df=1.0 / seg=1.0`；类权重由训练集标签直方图现算

### 9.5 结果（240 帧 autosim，204 训练 / 36 验证）

| epoch | val region IoU | df MAE (m) |
| --- | --- | --- |
| 0 | 0.180 | 1.098 |
| 5 | 0.509 | 0.204 |
| 10 | 0.567 | 0.140 |
| 20 | 0.612 | 0.113 |
| 30 | 0.727 | 0.087 |
| 39 | **0.743** | **0.085** |

2040 步 / 134.9 s / RTX 4090 D / batch 4 / bf16。可训练参数 3,643,340
（adapter 398,080 + decoder/heads 3,245,260），ViT 21,635,328 全部冻结。

### 9.6 制品与验收

| 项目 | 值 |
| --- | --- |
| `conf/models/frontiernet_defm.onnx` | 101,269,153 B |
| sha256 | `9f298a9f31f1b4b4829acf8ef933a0b382fe60082b746ff9ebd697c154821a84` |
| ONNX Runtime 对齐 | 3 组输入，`max abs diff` < 1e-3 |
| 帧 `00000` frontier 像素 | PyTorch 2687 / ORT 2687 / C++ 2687，教师 1984（IoU 0.622） |
| 帧 `00000` `df max` | PyTorch 13.9051 / ORT 13.9051 / C++ 13.9051，教师 14.2500 |
| C++ 端到端 | `perception/frontiernet_model_test`：`valid_ratio=1 df_max=13.9051 frontier_pixels=2687` |

C++ 侧新增的回归测试把 manifest → `DepthPreprocessor` → ONNX → 阈值解码整条链路跑一遍，
和 Python 的 2687 完全一致，说明 C++ 预处理与 `defm_preprocess` 没有偏差。
Python 侧用一条命令复现同样的三方对比（checkpoint / ONNX / 教师）：

```bash
/opt/venv/bin/python -m frontiernet.cli verify \
  --frame data/frontiernet/frames/00000.npz \
  --checkpoint data/frontiernet/run/best.pth \
  --defm-weights conf/models/defm_vit_s14.pth \
  --onnx ../conf/models/frontiernet_defm.onnx \
  --teacher-weights ../conf/models/rgbd_11cls.pth
# {"ok": true, "student_pixels": 2687, "onnx_pixels": 2687,
#  "onnx_df_max_abs_delta": 2.24e-4, "teacher_pixels": 1984, "teacher_iou": 0.6219}
```

```bash
# 把一帧 640x480 float32 深度导出成裸文件，再跑测试（缺 fixture 时自动 skip）
FRONTIERNET_TEST_DEPTH=/tmp/one_depth.f32 \
  LD_LIBRARY_PATH=/workspace/autonomy/build/autonomy/lib \
  /workspace/autonomy/build/autonomy/bin/frontiernet_model_test
```

导出必须带 `--load full`：默认的 `--load heads` 只搬 decoder/heads，
会把训练好的 adapter 丢掉，导出的还是空模型。

```bash
/opt/venv/bin/python -m frontiernet.export_onnx --variant defm --load full \
  --weights data/frontiernet/run/best.pth \
  --defm-weights conf/models/defm_vit_s14.pth \
  --out ../conf/models/frontiernet_defm.onnx \
  --manifest ../conf/models/frontiernet_defm.pb.txt \
  --manifest-model-path conf/models/frontiernet_defm.onnx \
  --validate --pytorch-atol 1e-3
```

### 9.7 已知限制

- 教师蒸馏：官方 RGB-D checkpoint 只用于离线伪标签，运行时只有 DeFM。
- 数据只有一间屋子 + 原地旋转 48 s，跨场景泛化未验证。
- 策略在地图有候选时优先走地图 frontier，地图无候选才用 DeFM（见第 8 节第 2 条）。
- 直行控制器只做前向净空与低位净空门限，没有局部绕障；杂物密集区仍会频繁放弃目标。

地图frontier的实现边界、真实深度栅格输入、600 s探索阶段与最终合并回归证据见
[20-exploration-validation.md](20-exploration-validation.md)。
