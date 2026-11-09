# frontiernet

FrontierNet-style frontier detection with a swappable depth encoder.

The official model (`cvg/FrontierNet`) is a ResNet-34 UNet with two heads: an
11-way class head and a distance-field regression head. This package keeps the
official decoder and heads byte-compatible with `rgbd_11cls.pth` and replaces
only the encoder, so the retraining recipe is "frozen DeFM backbone + trainable
adapter + decoder + two heads".

```text
depth ──> DeFM ViT-S/14 ──> 5 × 16×16 token grids ──> adapter ──> official UNet decoder
                                                                   ├── class head  (11)
                                                                   └── distance head (1)
```

## Layout

| Module | Role |
| --- | --- |
| `preprocess.py` | DeFM log-depth contract (matches C++ `DepthPreprocessor`) and the official RGB-D contract |
| `postprocess.py` | distance field + logits -> `FrontierMap` (region, gain, labels), numpy only |
| `unet.py` | official `TwoHeadUnet` rebuilt without `segmentation_models_pytorch` |
| `encoder_defm.py` | DeFM ViT wrapper, five-level adapter, `DefmEncoder` |
| `model.py` | `FrontierNetDeFM` with freeze policy and optimizer groups |
| `checkpoint.py` | transplant `decoder/segmentation_head/df_regression_head` from the official checkpoint |
| `infer.py` | `FrontierPredictor`, picks the preprocessing contract from `model.n_channels` |
| `collect.py` | autolink capture of depth/RGB frames for retraining |
| `label.py` | offline teacher labels from the official RGB-D checkpoint |
| `dataset.py` | `frames/` + `labels/` pairing, flip augmentation, class histogram |
| `loss.py` | weighted SmoothL1 distance field + class-weighted cross entropy |
| `train.py` | frozen-backbone retraining loop, AMP, cosine schedule, eval, checkpoints |
| `verify.py` | checkpoint / ONNX / teacher three-way comparison on one frame |
| `cli.py` | `smoke`, `baseline`, `align`, `collect`, `label`, `train`, `verify` |

## Quick start

```bash
python3 -m frontiernet.cli smoke                  # shapes only, synthetic backbone
python3 -m frontiernet.cli smoke --backbone defm --defm-weights ../conf/models/defm_vit_s14.pth
python3 -m frontiernet.cli baseline --weights rgbd_11cls.pth --depth frame.npy --rgb frame.png
python3 -m frontiernet.cli align    --weights rgbd_11cls.pth --depth frame.npy --rgb frame.png
```

`smoke` needs torch; the numpy half (`preprocess`, `postprocess`) imports without
it, so the preprocessing contract can be checked in a bare environment.

## Training

The official repository ships inference and exploration code but no training
code, so the loop lives here. Three commands, all from `python/`:

```bash
python -m frontiernet.collect --out data/frontiernet/frames --count 240 --hz 5
python -m frontiernet.label   --frames data/frontiernet/frames \
                              --out data/frontiernet/labels \
                              --weights conf/models/rgbd_11cls.pth --device cuda
python -m frontiernet.train   --config configs/train/frontiernet_defm.yaml
```

Labels must be produced on the DeFM pixel grid
(`frontiernet_preprocess_aligned`), not the upstream letterbox crop: the two
differ by a fixed warp and training against the letterbox labels stalls at
region IoU ~0.43 instead of ~0.74.

Measured on 240 autosim frames (204/36 split, 2040 steps, 135 s on an RTX 4090 D):
val region IoU 0.743, distance-field MAE 0.085 m. The exported graph is
`conf/models/frontiernet_defm.onnx`; export it with `--load full` so the trained
adapter is included.

Acceptance on one captured frame (checkpoint vs exported graph vs teacher):

```bash
python -m frontiernet.cli verify --frame data/frontiernet/frames/00000.npz \
  --checkpoint data/frontiernet/run/best.pth --defm-weights conf/models/defm_vit_s14.pth \
  --onnx ../conf/models/frontiernet_defm.onnx --teacher-weights ../conf/models/rgbd_11cls.pth
```

Frame `00000`: PyTorch 2687 / ONNX 2687 / C++ 2687 frontier pixels, teacher 1984
(IoU 0.622), ONNX `max|diff|` 2.24e-4.

`rgbd_11cls.pth` has no declared license file, so it is expected to live outside
the repo. See `../../docs/17-frontiernet.md` section 9 for the full write-up.
