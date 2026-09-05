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

`FathomConfig` must specify `model_path`, `backend`, `input_width`,
`input_height`, `depth_scale`, and `mask_threshold`. `backend` is `onnx` or
`tensorrt`; it is passed to the project common-network engine. The configured
width and height must exactly match the fixed profile used for ONNX export.
The token count is selected only when exporting and is baked into the graph;
the current C++ engine metadata does not expose it for runtime validation.
The graph itself takes only the two tensors above; camera intrinsics are used
after inference to project the refined depth.

```cpp
FathomConfig config;
config.model_path = "/models/fathom.onnx";
config.backend = "onnx";  // Or "tensorrt" for a supported engine artifact.
config.input_width = 640;
config.input_height = 480;
config.depth_scale = 0.001F;  // Incoming 16UC1 millimetres to metres.
config.mask_threshold = 0.5F;
```

`FathomNodeRunner::Create(config, &error)` initializes the concrete model once.
`Process(rgb, raw_depth, camera_info, &refined_depth, &point_cloud, &error)`
then synchronously accepts aligned automsgs `Image` RGB/depth and `CameraInfo`,
and returns an automsgs `Image` (`32FC1`, metres) plus organized `PointCloud2`
(XYZ in metres). `CameraInfo.width` and `CameraInfo.height` must be positive and
match the aligned input images. The runner deliberately declares no transport
topics and does not alter `PerceptionOptions`.

The concrete model engine and process runner are compiled only when
`BUILD_ONNXRUNTIME=ON` and ONNX Runtime is found. Configuration, RGB-D
preprocessing, projection, and the injected-runner `DepthRefiner` remain
available without that runtime.

## Autolink DAG deployment

When ONNX Runtime is available, `fathom_component` is built as an independent
autolink-loadable shared library (`libfathom_component.so`). It is loaded by
`fathom.dag` into its own mainboard process; the existing `PerceptionServer`
continues to run as the separate binary module in
`autonomy/perception/launch/perception.launch`.

The DAG reader order is fixed: RGB `Image`, raw-depth `Image`, then
`CameraInfo`. The sample component configuration is
`config/perception/fathom_component.pb.txt`; it uses Autosim's camera topics
`/camera/rgb/image_raw`, `/camera/depth/image_raw`, and
`/camera/camera_info`. Replace its `/models/fathom.onnx` placeholder with a
deployed model whose fixed width and height match `input_width` and
`input_height`. No model artifacts are stored in this repository.

The component publishes a `32FC1` metric refined-depth `Image` to
`/perception/fathom/refined_depth` and an organized XYZ `PointCloud2` to
`/perception/fathom/points` by default. Change those two explicit output
topics in the component config when integrating another graph.
