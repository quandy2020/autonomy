# Fathom RGB-D Refinement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate the released LingBot-Depth model as Fathom with Python fine-tuning and export tooling plus C++ ONNX Runtime/TensorRT deployment in autonomy.

**Architecture:** Python owns the released PyTorch model, a project-authored fine-tuning pipeline, and fixed-profile ONNX export. C++ owns input processing, inference through `autonomy::common::network::Engine`, output validation, and depth-to-point-cloud projection. The ONNX graph accepts normalized RGB and metric raw depth and returns refined metric depth and validity probability.

**Tech Stack:** Python 3.9+, PyTorch 2.6, pytest, ONNX, ONNX Runtime; C++17, OpenCV, Eigen, GoogleTest, autonomy common/network; optional TensorRT through the existing backend.

**Spec:** `docs/superpowers/specs/2026-09-04-fathom-design.md`

## Global Constraints

- Preserve upstream LingBot-Depth Apache-2.0 copyright and attribution in imported files.
- Mark locally modified upstream files as modified from LingBot-Depth.
- Treat RGB as float32 RGB NCHW in `[0, 1]` and depth as float32 meters.
- Treat zero, negative, NaN, and infinity input depth as invalid.
- Keep point-cloud projection outside the neural-network graph.
- Export fixed height, width, and token count; do not implement dynamic TensorRT profiles.
- Use `autonomy::common::network::Engine`; do not add a second generic inference abstraction.
- Do not commit model weights, ONNX files, TensorRT engines, datasets, or generated outputs.
- The training pipeline is a Fathom fine-tuning implementation, not an official training reproduction.
- Do not modify or discard unrelated working-tree changes.

## File Map

### Python

- `autonomy/perception/fathom/python/depth/__init__.py`: stable Python exports.
- `autonomy/perception/fathom/python/depth/api.py`: normalized Fathom inference API.
- `autonomy/perception/fathom/python/depth/export_onnx.py`: fixed-profile ONNX exporter and parity check.
- `autonomy/perception/fathom/python/depth/model/**`: imported upstream model implementation.
- `autonomy/perception/fathom/python/depth/utils/**`: imported upstream geometry and utility implementation.
- `autonomy/perception/fathom/python/train/__init__.py`: training package marker.
- `autonomy/perception/fathom/python/train/dataset.py`: RGB-D manifest dataset and synthetic missing-depth masks.
- `autonomy/perception/fathom/python/train/loss.py`: masked metric-depth and validity losses.
- `autonomy/perception/fathom/python/train/trainer.py`: fine-tuning loop and checkpoint writer.
- `autonomy/perception/fathom/python/train/main.py`: training command-line entry point.
- `autonomy/perception/fathom/python/test/test_api.py`: input/output contract tests.
- `autonomy/perception/fathom/python/test/test_dataset.py`: dataset and masking tests.
- `autonomy/perception/fathom/python/test/test_loss.py`: valid-pixel loss tests.
- `autonomy/perception/fathom/python/test/test_checkpoint.py`: released-format checkpoint tests.
- `autonomy/perception/fathom/python/test/test_export_onnx.py`: exporter smoke and parity tests.

### C++

- `autonomy/perception/fathom/config.hpp`: Fathom deployment options and validation API.
- `autonomy/perception/fathom/config.cpp`: option validation.
- `autonomy/perception/fathom/depth/types.hpp`: public camera, input, and output types.
- `autonomy/perception/fathom/depth/refiner.hpp`: public `DepthRefiner` facade.
- `autonomy/perception/fathom/depth/refiner.cpp`: processing, engine, and output orchestration.
- `autonomy/perception/fathom/engine/model.hpp`: model-specific `FathomEngine` wrapper.
- `autonomy/perception/fathom/engine/model.cpp`: network engine initialization, I/O validation, and execution.
- `autonomy/perception/fathom/processing/rgbd.hpp`: RGB-D tensor conversion API.
- `autonomy/perception/fathom/processing/rgbd.cpp`: RGB/depth resize and normalization.
- `autonomy/perception/fathom/processing/rgbd_test.cpp`: preprocessing tests.
- `autonomy/perception/fathom/projection/point_cloud.hpp`: depth projection API.
- `autonomy/perception/fathom/projection/point_cloud.cpp`: organized XYZ projection.
- `autonomy/perception/fathom/projection/point_cloud_test.cpp`: projection tests.
- `autonomy/perception/fathom/fathom_node_runner.hpp`: process-facing runner API.
- `autonomy/perception/fathom/fathom_node_runner.cpp`: thin runner over `DepthRefiner`.
- `autonomy/perception/fathom/depth/refiner_test.cpp`: facade test with a fake model runner.
- `autonomy/perception/fathom/README.md`: provenance, Python usage, export, and C++ usage.
- `cmake/autonomy_sources.cmake`: exclude Fathom engine-dependent source when no inference backend is available.

---

### Task 1: Import the released Python model behind the Fathom API

**Files:**
- Create: `autonomy/perception/fathom/python/depth/__init__.py`
- Create: `autonomy/perception/fathom/python/depth/api.py`
- Import: `autonomy/perception/fathom/python/depth/model/**`
- Import: `autonomy/perception/fathom/python/depth/utils/**`
- Create: `autonomy/perception/fathom/python/test/test_api.py`
- Create: `autonomy/perception/fathom/README.md`

**Interfaces:**
- Produces: `load_model(source: str | Path, device: str | torch.device) -> MDMModel`
- Produces: `infer(model, image, raw_depth, intrinsics=None, apply_mask=True) -> dict[str, torch.Tensor]`
- Produces: imports `MDMModel` from `depth`.

- [ ] **Step 1: Write the failing API contract test**

```python
class FakeModel:
    def infer(self, image, depth_in, intrinsics, apply_mask, use_fp16):
        assert image.shape == (1, 3, 2, 3)
        assert image.dtype == torch.float32
        assert depth_in.shape == (1, 2, 3)
        return {"depth": depth_in + 1.0, "mask": depth_in > 0}


def test_infer_normalizes_shapes_and_invalid_depth():
    image = torch.full((2, 3, 3), 255, dtype=torch.uint8)
    raw_depth = torch.tensor([[1.0, 0.0, float("nan")], [2.0, -1.0, 3.0]])
    output = infer(FakeModel(), image, raw_depth, use_fp16=False)
    assert output["depth"].shape == (2, 3)
    assert torch.isfinite(output["raw_depth"]).all()
    assert output["raw_depth"][0, 1].item() == 0.0
```

- [ ] **Step 2: Run the test and verify import failure**

Run: `PYTHONPATH=autonomy/perception/fathom/python pytest -q autonomy/perception/fathom/python/test/test_api.py`

Expected: FAIL because `depth.api` does not exist.

- [ ] **Step 3: Import upstream sources with attribution**

Copy upstream `mdm/model` to `python/depth/model` and `mdm/utils` to `python/depth/utils`. Retain upstream license headers, add a local modification notice to changed imports, replace imports rooted at `mdm` with package-relative `depth` imports, and copy the upstream `LICENSE` to `autonomy/perception/fathom/README.md` as a linked provenance statement rather than duplicating the full license text.

- [ ] **Step 4: Implement the stable API**

`infer` must accept HWC uint8 or CHW float RGB, sanitize non-finite/non-positive depth to zero, add a batch dimension, call `model.infer`, remove the synthetic batch dimension, and return `depth`, `mask`, and sanitized `raw_depth`. Reject mismatched spatial dimensions with `ValueError`.

- [ ] **Step 5: Run API tests**

Run: `PYTHONPATH=autonomy/perception/fathom/python pytest -q autonomy/perception/fathom/python/test/test_api.py`

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add autonomy/perception/fathom/python/depth autonomy/perception/fathom/python/test/test_api.py autonomy/perception/fathom/README.md
git commit -m "feat(perception): import Fathom Python model"
```

### Task 2: Add RGB-D fine-tuning data and losses

**Files:**
- Create: `autonomy/perception/fathom/python/train/__init__.py`
- Create: `autonomy/perception/fathom/python/train/dataset.py`
- Create: `autonomy/perception/fathom/python/train/loss.py`
- Create: `autonomy/perception/fathom/python/test/test_dataset.py`
- Create: `autonomy/perception/fathom/python/test/test_loss.py`

**Interfaces:**
- Produces: `RgbdManifestDataset(manifest_path: Path, dropout_probability: float, depth_scale: float)`.
- Produces: samples with `image`, `raw_depth`, `target_depth`, `valid_mask`, and optional `intrinsics`.
- Produces: `fathom_loss(pred_depth, target_depth, valid_mask, pred_mask=None, mask_weight=0.1) -> LossOutput`.

- [ ] **Step 1: Write failing dataset tests**

Create temporary 2x3 RGB and uint16 depth images plus a JSONL record:

```json
{"rgb":"rgb.png","raw_depth":"raw.png","target_depth":"target.png","intrinsics":[[1,0,0.5],[0,1,0.5],[0,0,1]]}
```

Assert that a `depth_scale` of `0.001` converts millimeters to meters, invalid target pixels are excluded, and dropout with probability `1.0` zeros every valid raw-depth pixel without modifying target depth.

- [ ] **Step 2: Run dataset tests and verify failure**

Run: `PYTHONPATH=autonomy/perception/fathom/python pytest -q autonomy/perception/fathom/python/test/test_dataset.py`

Expected: FAIL because `train.dataset` does not exist.

- [ ] **Step 3: Implement the manifest dataset**

Resolve sample paths relative to the manifest, load RGB with Pillow, load depth without lossy conversion, convert to contiguous tensors, normalize RGB to `[0, 1]`, convert depth using `depth_scale`, build `valid_mask = isfinite(target) & (target > 0)`, sanitize invalid raw depth to zero, and apply reproducible Bernoulli dropout through an injectable `torch.Generator`.

- [ ] **Step 4: Write failing loss tests**

```python
def test_loss_ignores_invalid_target_pixels():
    pred = torch.tensor([[1.5, 100.0]])
    target = torch.tensor([[1.0, 0.0]])
    valid = torch.tensor([[True, False]])
    result = fathom_loss(pred, target, valid)
    assert result.valid_pixels == 1
    assert torch.isclose(result.total, F.smooth_l1_loss(pred[:, :1], target[:, :1]))
```

Also assert that an all-invalid batch raises `ValueError` and that mask BCE uses `valid_mask.float()` as its target.

- [ ] **Step 5: Implement and verify the losses**

Implement a `LossOutput` dataclass with `total`, `depth`, `mask`, and `valid_pixels`. Run:

`PYTHONPATH=autonomy/perception/fathom/python pytest -q autonomy/perception/fathom/python/test/test_dataset.py autonomy/perception/fathom/python/test/test_loss.py`

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add autonomy/perception/fathom/python/train autonomy/perception/fathom/python/test/test_dataset.py autonomy/perception/fathom/python/test/test_loss.py
git commit -m "feat(perception): add Fathom fine-tuning data and losses"
```

### Task 3: Add the fine-tuning loop and checkpoint contract

**Files:**
- Create: `autonomy/perception/fathom/python/train/trainer.py`
- Create: `autonomy/perception/fathom/python/train/main.py`
- Create: `autonomy/perception/fathom/python/test/test_checkpoint.py`

**Interfaces:**
- Consumes: Task 1 `load_model`; Task 2 dataset and `fathom_loss`.
- Produces: `save_checkpoint(path, model, model_config, optimizer, step)` with released `model_config` and `model` keys.
- Produces: `train_epoch(model, loader, optimizer, device, mask_weight) -> dict[str, float]`.

- [ ] **Step 1: Write failing checkpoint and train-step tests**

Use a two-parameter fake depth model returning `{"depth_reg": ..., "mask": ...}`. Assert one training batch changes its parameter and that a saved checkpoint contains `model_config`, `model`, `optimizer`, and `step`. Reload the model state into a new fake model and compare parameters exactly.

- [ ] **Step 2: Run tests and verify failure**

Run: `PYTHONPATH=autonomy/perception/fathom/python pytest -q autonomy/perception/fathom/python/test/test_checkpoint.py`

Expected: FAIL because `train.trainer` does not exist.

- [ ] **Step 3: Implement trainer and CLI**

The CLI requires `--checkpoint`, `--manifest`, and `--output`; accepts `--epochs`, `--batch-size`, `--learning-rate`, `--depth-scale`, `--dropout-probability`, `--mask-weight`, and `--device`; loads the released model, uses AdamW, reports mean total/depth/mask loss, and writes one checkpoint per epoch plus `latest.pt`.

- [ ] **Step 4: Verify training tests and CLI help**

Run:

```bash
PYTHONPATH=autonomy/perception/fathom/python pytest -q autonomy/perception/fathom/python/test/test_checkpoint.py
PYTHONPATH=autonomy/perception/fathom/python python -m train.main --help
```

Expected: tests PASS and help exits 0.

- [ ] **Step 5: Commit**

```bash
git add autonomy/perception/fathom/python/train autonomy/perception/fathom/python/test/test_checkpoint.py
git commit -m "feat(perception): add Fathom fine-tuning loop"
```

### Task 4: Export a fixed-profile ONNX model with parity validation

**Files:**
- Create: `autonomy/perception/fathom/python/depth/export_onnx.py`
- Create: `autonomy/perception/fathom/python/test/test_export_onnx.py`

**Interfaces:**
- Consumes: released `MDMModel.forward(image, num_tokens, depth)`.
- Produces: ONNX inputs `image`, `raw_depth`; outputs `refined_depth`, `validity`.
- Produces: `export_onnx(model, output, height, width, num_tokens, opset=17) -> Path`.

- [ ] **Step 1: Write a failing export smoke test**

Use a tiny fake module whose forward returns `depth + image.mean(dim=1)` and an all-one validity tensor. Export at `height=4`, `width=6`, load with `onnx.checker.check_model`, run ONNX Runtime CPU, and assert both outputs match PyTorch within `rtol=1e-5, atol=1e-6`.

- [ ] **Step 2: Run the test and verify failure**

Run: `PYTHONPATH=autonomy/perception/fathom/python pytest -q autonomy/perception/fathom/python/test/test_export_onnx.py`

Expected: FAIL because `depth.export_onnx` does not exist.

- [ ] **Step 3: Implement the export wrapper**

Wrap the released model so `num_tokens` is a fixed Python integer, set `model.encoder.onnx_compatible_mode = True`, convert absent masks to ones, export FP32 with opset 17 and static batch/spatial dimensions, run ONNX checker, then compare one deterministic sample through PyTorch and ONNX Runtime. Delete the output and raise `RuntimeError` if validation fails.

- [ ] **Step 4: Run export tests**

Run: `PYTHONPATH=autonomy/perception/fathom/python pytest -q autonomy/perception/fathom/python/test/test_export_onnx.py`

Expected: PASS or SKIP with an explicit missing optional `onnx`/`onnxruntime` dependency reason.

- [ ] **Step 5: Commit**

```bash
git add autonomy/perception/fathom/python/depth/export_onnx.py autonomy/perception/fathom/python/test/test_export_onnx.py
git commit -m "feat(perception): export Fathom models to ONNX"
```

### Task 5: Implement C++ RGB-D processing and projection

**Files:**
- Create: `autonomy/perception/fathom/depth/types.hpp`
- Create: `autonomy/perception/fathom/processing/rgbd.hpp`
- Create: `autonomy/perception/fathom/processing/rgbd.cpp`
- Create: `autonomy/perception/fathom/processing/rgbd_test.cpp`
- Create: `autonomy/perception/fathom/projection/point_cloud.hpp`
- Create: `autonomy/perception/fathom/projection/point_cloud.cpp`
- Create: `autonomy/perception/fathom/projection/point_cloud_test.cpp`

**Interfaces:**
- Produces: `CameraIntrinsics {fx, fy, cx, cy}` in pixels.
- Produces: `DepthInput {cv::Mat bgr, cv::Mat raw_depth, CameraIntrinsics intrinsics}`.
- Produces: `PrepareRgbd(input, width, height, depth_scale, TensorMap*, error) -> bool`.
- Produces: `ProjectDepth(depth_m, mask, intrinsics, cv::Mat* xyz, error) -> bool`.

- [ ] **Step 1: Write failing preprocessing tests**

Assert that a 1x2 BGR8 image becomes planar RGB float data in `[0,1]`, uint16 depth is multiplied by `0.001`, zero remains invalid/zero, and mismatched RGB/depth dimensions return false with a non-empty error.

- [ ] **Step 2: Write failing projection tests**

For depth `[[2, 2]]` and intrinsics `fx=2, fy=2, cx=0, cy=0`, assert points `(0,0,2)` and `(1,0,2)`. Assert invalid mask positions contain NaN XYZ and non-positive focal lengths are rejected.

- [ ] **Step 3: Run tests and verify failure**

Run: `cmake --build build --target autonomy.perception.fathom.processing.rgbd_test autonomy.perception.fathom.projection.point_cloud_test -j2`

Expected: build fails because the APIs do not exist.

- [ ] **Step 4: Implement minimal processing and projection**

Use OpenCV resize, BGR-to-RGB conversion, and explicit planar NCHW loops. Scale intrinsics by output/input width and height during resize. Bind tensors using exact names `image` and `raw_depth`. Projection uses `x=(u-cx)*z/fx`, `y=(v-cy)*z/fy`, `z=depth`.

- [ ] **Step 5: Configure and run C++ tests**

Run:

```bash
cmake -S . -B build -DBUILD_TEST=ON -DBUILD_ONNXRUNTIME=OFF
cmake --build build --target autonomy.perception.fathom.processing.rgbd_test autonomy.perception.fathom.projection.point_cloud_test -j2
ctest --test-dir build -R 'fathom.(processing.rgbd|projection.point_cloud)_test' --output-on-failure
```

Expected: two tests PASS.

- [ ] **Step 6: Commit**

```bash
git add autonomy/perception/fathom/depth/types.hpp autonomy/perception/fathom/processing autonomy/perception/fathom/projection
git commit -m "feat(perception): process and project Fathom RGB-D data"
```

### Task 6: Implement the C++ model engine and depth-refinement facade

**Files:**
- Create: `autonomy/perception/fathom/config.hpp`
- Create: `autonomy/perception/fathom/config.cpp`
- Create: `autonomy/perception/fathom/engine/model.hpp`
- Create: `autonomy/perception/fathom/engine/model.cpp`
- Create: `autonomy/perception/fathom/depth/refiner.hpp`
- Create: `autonomy/perception/fathom/depth/refiner.cpp`
- Create: `autonomy/perception/fathom/depth/refiner_test.cpp`

**Interfaces:**
- Produces: `FathomConfig {model_path, backend, input_width, input_height, num_tokens, depth_scale, mask_threshold}`.
- Produces: `FathomEngine::Create(config, error) -> unique_ptr<FathomEngine>`.
- Produces: `FathomEngine::Run(TensorMap, TensorMap*, error) -> bool`.
- Produces: `DepthRefiner::Create(config, error)` and `Refine(input, DepthOutput*, error) -> bool`.

- [ ] **Step 1: Write failing configuration tests in `refiner_test.cpp`**

Assert empty model path, non-positive dimensions, non-positive depth scale, and mask thresholds outside `[0,1]` are rejected. Assert backend values `onnx` and `tensorrt` are accepted.

- [ ] **Step 2: Write a facade test with an injectable fake runner**

Inject a runner that returns static `refined_depth` and `validity` tensors. Assert `DepthRefiner::Refine` restores output to the original input size, thresholds validity, and produces an organized XYZ image using the original intrinsics.

- [ ] **Step 3: Run tests and verify failure**

Run: `cmake --build build --target autonomy.perception.fathom.depth.refiner_test -j2`

Expected: build fails because Fathom config/refiner APIs do not exist.

- [ ] **Step 4: Implement configuration and engine validation**

Create `common::network::InferenceOptions` from `backend` and `model_path`. After engine creation, require inputs named `image` and `raw_depth`, outputs named `refined_depth` and `validity`, rank four/three as exported, float32 element type, batch one, and configured static spatial size. Return the underlying engine error unchanged with a `Fathom:` prefix.

- [ ] **Step 5: Implement the facade**

Call `PrepareRgbd`, the model runner, output-shape validation, OpenCV resize to original resolution, threshold validity, and `ProjectDepth`. Never reuse the previous successful output after a failed frame.

- [ ] **Step 6: Run facade and regression tests**

Run:

```bash
cmake --build build --target autonomy.perception.fathom.depth.refiner_test -j2
ctest --test-dir build -R 'fathom' --output-on-failure
```

Expected: all Fathom C++ tests PASS.

- [ ] **Step 7: Commit**

```bash
git add autonomy/perception/fathom/config.* autonomy/perception/fathom/engine autonomy/perception/fathom/depth
git commit -m "feat(perception): deploy Fathom through common network"
```

### Task 7: Add the process-facing runner and build guards

**Files:**
- Create: `autonomy/perception/fathom/fathom_node_runner.hpp`
- Create: `autonomy/perception/fathom/fathom_node_runner.cpp`
- Modify: `cmake/autonomy_sources.cmake`
- Modify: `autonomy/perception/fathom/README.md`

**Interfaces:**
- Consumes: Task 6 `DepthRefiner`.
- Produces: `FathomNodeRunner::Create(FathomConfig, error)` and synchronous `Process(DepthInput, DepthOutput*, error)`.

- [ ] **Step 1: Add a build-only runner contract**

Define a non-copyable runner owning `std::unique_ptr<DepthRefiner>`. `Create` performs all model initialization, while `Process` is synchronous and delegates one frame. Do not add topic names or alter `PerceptionOptions` because the current perception server has no established RGB image/output message contract for this model.

- [ ] **Step 2: Add build-source guards**

Collect `${_AUTONOMY_ROOT}/perception/fathom/engine/*.cpp`, `depth/refiner.cpp`, and `fathom_node_runner.cpp` into `FATHOM_NETWORK_SRCS`. Remove them from `ALL_LIBRARY_SRCS` when `BUILD_ONNXRUNTIME` is off or `OnnxRuntime_FOUND` is false. Keep processing and projection available without a neural-network runtime.

- [ ] **Step 3: Document exact workflows**

README must include upstream provenance and license, `PYTHONPATH` setup, manifest schema, fine-tuning command, ONNX export command, C++ config fields, backend selection, tensor names, units, fixed-profile restriction, and a statement that official training is not reproduced.

- [ ] **Step 4: Verify both build modes**

Run:

```bash
cmake -S . -B build-no-ort -DBUILD_TEST=ON -DBUILD_ONNXRUNTIME=OFF
cmake --build build-no-ort -j2
cmake -S . -B build-ort -DBUILD_TEST=ON -DBUILD_ONNXRUNTIME=ON
cmake --build build-ort -j2
```

Expected: no-ORT build succeeds without engine/refiner/runner; ORT build succeeds when ONNX Runtime is installed. If ONNX Runtime is unavailable, configuration must report it and exercise the guarded build rather than falsely claiming ORT coverage.

- [ ] **Step 5: Commit**

```bash
git add autonomy/perception/fathom/fathom_node_runner.* autonomy/perception/fathom/README.md cmake/autonomy_sources.cmake
git commit -m "feat(perception): integrate Fathom runner into autonomy build"
```

### Task 8: Run final Python, C++, license, and parity verification

**Files:**
- Modify only files implicated by verification failures.

**Interfaces:**
- Consumes all earlier tasks.
- Produces verified source tree and documented test evidence.

- [ ] **Step 1: Run all dependency-light Python tests**

Run: `PYTHONPATH=autonomy/perception/fathom/python pytest -q autonomy/perception/fathom/python/test`

Expected: all available tests PASS; heavyweight export tests may only SKIP for named missing optional packages.

- [ ] **Step 2: Run all Fathom C++ tests**

Run: `ctest --test-dir build-no-ort -R 'fathom' --output-on-failure`

Expected: processing and projection tests PASS with no inference runtime.

- [ ] **Step 3: Run formatting and static checks**

Run:

```bash
git diff --check
python -m compileall -q autonomy/perception/fathom/python
```

Expected: both commands exit 0.

- [ ] **Step 4: Run real-model parity when a checkpoint is available**

Export the selected released checkpoint at the deployment profile, run Python PyTorch versus Python ONNX Runtime, then run the C++ Fathom engine on the same RGB-D fixture. Require finite-pixel depth `rtol <= 1e-3`, `atol <= 1e-3` for FP32 ONNX Runtime. Record TensorRT differences separately; do not weaken the ONNX Runtime threshold.

- [ ] **Step 5: Audit attribution and artifacts**

Run:

```bash
rg -n "LingBot-Depth|Apache License" autonomy/perception/fathom
find autonomy/perception/fathom -type f \( -name '*.onnx' -o -name '*.engine' -o -name '*.plan' -o -name '*.pt' \)
git status --short
```

Expected: attribution is present; artifact search returns no committed candidates; unrelated pre-existing changes remain untouched.

- [ ] **Step 6: Commit verification-only fixes if needed**

```bash
git add autonomy/perception/fathom cmake/autonomy_sources.cmake
git commit -m "test(perception): verify Fathom deployment parity"
```

Skip this commit when verification required no source changes.
