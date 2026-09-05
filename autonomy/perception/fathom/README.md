# Fathom RGB-D depth refinement

`python/depth/model` and `python/depth/utils` are vendored from
[LingBot-Depth](https://github.com/robbyant/lingbot-depth) at commit
[`f3a237e434ae987bc38281476d6cfb5df3e4d739`](https://github.com/robbyant/lingbot-depth/tree/f3a237e434ae987bc38281476d6cfb5df3e4d739).
Those files are distributed under the upstream [Apache License 2.0](https://github.com/robbyant/lingbot-depth/blob/f3a237e434ae987bc38281476d6cfb5df3e4d739/LICENSE).
Their upstream formatting, including trailing whitespace, is retained for
provenance; `.gitattributes` suppresses only blank-at-end-of-line and
blank-at-end-of-file diagnostics inside these two vendored source trees.

The dataset, fine-tuning loop, loss, checkpoint helpers, ONNX wrapper, and C++
integration in this directory are Fathom-authored integration code. They are
not an official reproduction of LingBot-Depth training.

## Python setup and fine-tuning

From the repository root, make the Fathom Python package importable:

```bash
export PYTHONPATH="$PWD/autonomy/perception/fathom/python${PYTHONPATH:+:$PYTHONPATH}"
```

The fine-tuning input is JSONL: one object per line, with paths relative to
the manifest's directory. `rgb`, `raw_depth`, and `target_depth` are required;
`intrinsics` is optional and, when present, is a single 3-by-3 matrix.

```json
{"rgb":"rgb/000001.png","raw_depth":"depth/raw/000001.png","target_depth":"depth/target/000001.png","intrinsics":[[525.0,0.0,319.5],[0.0,525.0,239.5],[0.0,0.0,1.0]]}
```

RGB is converted to normalized float RGB. Raw and target depth samples are
read as numeric images and multiplied by `--depth-scale` (default `0.001`, so
millimetres become metres). Invalid or non-positive raw values become zero;
the target's finite positive values form the training mask.

```bash
python -m train.main \
  --checkpoint /models/released.pt \
  --manifest /datasets/train.jsonl \
  --output /models/fathom-finetuned \
  --epochs 10 --batch-size 4 --learning-rate 1e-4 \
  --depth-scale 0.001 --dropout-probability 0.1 --mask-weight 0.1
```

## Fixed-profile ONNX export

Export uses float32 `image` and `raw_depth` inputs and validates the produced
ONNX file with ONNX Runtime. Pick one width, height, and token count for a
deployment profile; dynamic batch, spatial, and token dimensions are not
exported.

```bash
python -c 'from depth.api import load_model; from depth.export_onnx import export_onnx; model = load_model("/models/fathom-finetuned/latest.pt", "cpu"); export_onnx(model, "/models/fathom.onnx", height=480, width=640, num_tokens=2400)'
```

The exported graph contract is:

| Tensor | Type and fixed shape |
| --- | --- |
| `image` | float32 `[1, 3, height, width]`, normalized RGB in `[0, 1]` |
| `raw_depth` | float32 `[1, height, width]`, metres; zero means invalid |
| `refined_depth` | float32 `[1, height, width]`, metres |
| `validity` | float32 `[1, height, width]`, thresholded by C++ |

## C++ deployment

`proto::FathomOptions` is the single C++ configuration model. It specifies
`model_path`, `backend`, `input_width`, `input_height`, `depth_scale`, and
`mask_threshold`, plus the component output topics. `backend` is `onnx` or
`tensorrt`; it is passed to the project common-network engine. The configured
width and height must exactly match the fixed profile used for ONNX export.
The token count is selected only when exporting and is baked into the graph;
the current C++ engine metadata does not expose it for runtime validation.
The graph itself takes only the two tensors above; camera intrinsics are used
after inference to project the refined depth.

```cpp
proto::FathomOptions options;
options.set_model_path("/models/fathom.onnx");
options.set_backend("onnx");  // Or "tensorrt".
options.set_input_width(640);
options.set_input_height(480);
options.set_depth_scale(0.001F);  // Incoming 16UC1 millimetres to metres.
options.set_mask_threshold(0.5F);
options.set_refined_depth_topic("/perception/fathom/refined_depth");
options.set_point_cloud_topic("/perception/fathom/points");
```

`FathomComponent::Init()` creates the concrete model engine and `DepthRefiner`
once. `Proc(rgb, raw_depth, camera_info)` then synchronously accepts aligned
automsgs `Image` RGB/depth and `CameraInfo`, and publishes an automsgs `Image`
(`32FC1`, metres) plus organized `PointCloud2` (XYZ in metres).
`CameraInfo.width` and `CameraInfo.height` must be positive and match the
aligned input images. Transport topics are owned by the component protobuf
options and remain independent from `PerceptionOptions`.

The concrete model engine and autolink component are compiled only when
`BUILD_ONNXRUNTIME=ON` and ONNX Runtime is found. Option validation, RGB-D
preprocessing, projection, and the injected-runner `DepthRefiner` remain
available without that runtime.

`PrepareRgbd` accepts `bgr8` and `rgb8` color input. Depth input can be
`16UC1` sensor units or `32FC1`; both are multiplied by `depth_scale` before
inference. Use `0.001` for millimeter `16UC1`, and `1.0` for Autosim's metric
`32FC1`. Non-finite and non-positive `32FC1` samples are converted to zero.

## Autolink DAG deployment

When ONNX Runtime is available, `fathom_component` is built as an independent
autolink-loadable shared library (`libfathom_component.so`). It is loaded by
`dag/fathom.dag` into its own mainboard process; the existing
`PerceptionServer` continues to run as the separate binary module in
`autonomy/perception/launch/perception.launch`. For component-only deployment,
use `launch/fathom.launch`.

The component follows the repository's Apollo-style runtime layout:

```text
fathom/
├── conf/fathom.pb.txt
├── dag/fathom.dag
├── launch/fathom.launch
├── proto/fathom.proto
├── fathom_component.hpp
├── fathom_component.cpp
├── fathom_component_test.cpp
├── options.hpp
└── options.cpp
```

The DAG reader order is fixed: RGB `Image`, raw-depth `Image`, then
`CameraInfo`. The sample component configuration is
`conf/fathom.pb.txt`; it uses Autosim's camera topics
`/camera/rgb/image_raw`, `/camera/depth/image_raw`, and
`/camera/camera_info`. Autosim publishes `rgb8` and metric `32FC1`, so the
sample uses `depth_scale: 1.0`. Replace its `/models/fathom.onnx` placeholder
with a deployed model whose fixed width and height match `input_width` and
`input_height`. No model artifacts are stored in this repository.

The component publishes a `32FC1` metric refined-depth `Image` to
`/perception/fathom/refined_depth` and an organized XYZ `PointCloud2` to
`/perception/fathom/points` by default. Change those two explicit output
topics in the component config when integrating another graph.

For a plain CMake build, run the following from the repository root (where the
configured `CMAKE_BINARY_DIR` is `$PWD/build`):

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/perception/launch"
export AUTOLINK_DAG_PATH="$PWD/autonomy/perception/fathom"
export AUTOLINK_CONF_PATH="$PWD/autonomy/perception/fathom"
export AUTOLINK_LIB_PATH="$PWD/build/lib"
autolink launch start perception.launch
```

For a colcon/package build, run the following from `src/autonomy`; this layout
places this package's CMake binary directory at `$PWD/../../build/autonomy`:

```bash
export PATH="$PWD/../../build/autonomy/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/perception/launch"
export AUTOLINK_DAG_PATH="$PWD/autonomy/perception/fathom"
export AUTOLINK_CONF_PATH="$PWD/autonomy/perception/fathom"
export AUTOLINK_LIB_PATH="$PWD/../../build/autonomy/lib"
autolink launch start perception.launch
```

Installation places `libfathom_component.so` in `lib`, the DAG in
`share/autonomy/fathom/dag`, the sample config in
`share/autonomy/fathom/conf`, the standalone launch file in
`share/autonomy/fathom/launch`, and the combined launch file below
`share/autonomy/perception/launch`. For an installed tree, set
`AUTOLINK_DAG_PATH=$prefix/share/autonomy/fathom`,
`AUTOLINK_CONF_PATH=$prefix/share/autonomy/fathom`, and
`AUTOLINK_LIB_PATH=$prefix/lib` before launching the installed perception
launch file.
