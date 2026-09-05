# Fathom RGB-D Refinement Design

## Scope

Fathom integrates the Apache-2.0 LingBot-Depth model into `autonomy/perception` under a project-neutral name. It provides:

- the released Python model and inference path;
- a maintainable fine-tuning pipeline built from the released architecture;
- fixed-profile ONNX export;
- C++ inference through the existing `autonomy::common::network` abstraction;
- RGB-D preprocessing, validity-mask handling, and depth-to-point-cloud projection;
- Python/ONNX/C++ parity checks.

The upstream project does not publish its training pipeline or complete loss recipe. Fathom's training path is therefore an engineering implementation for fine-tuning, not a claim of reproducing the official LingBot-Depth training procedure.

## Source Layout

```text
autonomy/perception/fathom/
├── README.md
├── fathom_node_runner.cpp
├── fathom_node_runner.hpp
├── config.cpp
├── config.hpp
├── depth/
├── engine/
├── processing/
├── projection/
└── python/
    ├── depth/
    ├── train/
    └── test/
```

C++ sources live directly under the Fathom module, matching the organization of `localization/atlas`. Tests are colocated as `*_test.cpp`. Python tests remain under `python/test` to work with standard Python test discovery.

## Python Model Integration

The released LingBot-Depth model code is imported into `python/depth` while retaining upstream copyright and Apache-2.0 attribution. Imports are adjusted to use the Fathom package boundary without changing model mathematics.

The stable Python API accepts:

- RGB tensor: `float32`, `[B, 3, H, W]`, RGB order, range `[0, 1]`;
- raw depth: `float32`, `[B, H, W]`, meters, with `0` or non-finite values treated as invalid;
- camera intrinsics: optional `float32`, `[B, 3, 3]`, normalized by image width and height.

It returns refined metric depth and a validity mask. Point-cloud construction remains outside the exported network so Python and C++ use the same explicit projection convention.

## Fine-Tuning Pipeline

`python/train` provides a fine-tuning pipeline rather than full pretraining from scratch. A dataset sample contains RGB, noisy or sparse input depth, clean target depth, and optional intrinsics.

The training pipeline:

1. normalizes RGB and converts all depth values to meters;
2. creates additional synthetic missing-depth masks when enabled;
3. runs the released model architecture from a pretrained checkpoint;
4. computes loss only on valid target pixels;
5. optimizes metric depth regression and, when present, validity-mask prediction;
6. saves checkpoints in the released `model_config` plus `model` state-dictionary format.

The initial loss is masked Smooth L1 on metric depth plus optional binary cross-entropy for the predicted validity mask. Additional research losses are out of scope until validated against project data.

## ONNX Boundary

The exported ONNX graph contains only neural-network computation:

```text
image + raw_depth -> refined_depth + validity_probability
```

Export uses a fixed deployment resolution and token count supplied by command-line arguments. This avoids fragile dynamic-shape behavior in DINOv2 positional interpolation and produces a predictable TensorRT profile. Image conversion, depth-unit conversion, invalid-value handling, thresholding, and point projection remain outside the graph.

The exporter enables the released model's ONNX-compatible interpolation path, runs ONNX validation, and compares ONNX Runtime output with PyTorch output before accepting an artifact.

## C++ Components

### `depth`

Defines the depth-refinement facade. Its public C++ API uses existing `automsgs::msgs::sensor_msgs::Image`, `CameraInfo`, and `PointCloud2` messages directly; Fathom does not define parallel image, camera, or point-cloud types. Consumers do not depend on ONNX Runtime or TensorRT types.

### `engine`

Adapts the exported model to `autonomy::common::network::Engine`. ONNX Runtime is the default backend because it is already enabled by the project. TensorRT is selectable through the same project backend when available; Fathom does not introduce another inference abstraction.

### `processing`

Validates aligned automsgs RGB/depth dimensions and buffers, converts the RGB message to normalized RGB NCHW tensors, converts sensor depth to meters, and maps invalid depth values to the exported model convention. It restores refined depth and mask as automsgs image messages.

### `projection`

Projects valid metric-depth and mask image messages into an organized automsgs `PointCloud2` using pixel-center coordinates and the original automsgs `CameraInfo`. Invalid pixels remain explicitly marked and are not converted to finite 3D points.

### Root files

`config.*` owns model path, backend, input resolution, depth scale, and mask
threshold. Token count is an export-time choice baked into the fixed ONNX
artifact; the C++ engine metadata exposes no reliable runtime token binding.
`fathom_node_runner.*` connects the module to the perception process without
embedding model-specific tensor logic.

## Error Handling

Initialization fails with an actionable error for a missing model, unavailable backend, unexpected model inputs or outputs, or an invalid deployment profile. Per-frame processing rejects dimension mismatches, invalid intrinsics, and unsupported image/depth types. Inference errors propagate to the caller; the module never publishes stale output as a successful result.

## Verification

The implementation is accepted when:

- Python unit tests cover input normalization, invalid-depth masks, training loss, checkpoint round trips, and projection;
- C++ unit tests cover preprocessing and projection without requiring a GPU;
- an export smoke test produces an ONNX model and passes ONNX validation;
- PyTorch and ONNX Runtime outputs agree within declared FP32 tolerances on a fixed RGB-D sample;
- the C++ ONNX Runtime path agrees with Python ONNX Runtime on the same sample;
- the autonomy library builds with ONNX Runtime enabled and also builds cleanly with ONNX Runtime unavailable by excluding Fathom engine sources;
- all imported upstream files retain required attribution and modified files identify local changes.

Model weights and generated ONNX or TensorRT artifacts are not committed to Git.

## Deferred Work

- exact reproduction of the unpublished official training recipe;
- distributed pretraining on the full three-million-sample dataset;
- dynamic-resolution TensorRT optimization profiles;
- temporal RGB-D fusion;
- transport-specific publishing details not already defined by the perception server.
