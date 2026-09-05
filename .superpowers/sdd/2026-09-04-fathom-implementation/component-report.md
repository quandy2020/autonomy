# Fathom autolink component implementation report

## Scope delivered

- Added `FathomComponent`, an `autolink::Component<Image, Image, CameraInfo>`
  DAG module that uses the existing `FathomNodeRunner` exactly once per frame.
- `Init()` reads `FathomComponentConfig` through `ComponentBase::GetProtoConfig`,
  translates and validates the deployment profile, creates the runner, then
  creates refined-depth and organized-point-cloud writers.
- `Proc()` rejects null inputs and uninitialized state, refines the frame before
  attempting either publication, reports runner/writer failures, and returns
  false if refinement or either publication fails.
- `Clear()` resets the runner and both writers, making repeated cleanup safe.

## Runtime/configuration artifacts

- Added `fathom_component_config.proto` with model path, backend, fixed input
  dimensions, depth scale, mask threshold, and both output channels.
- Added `fathom.dag` with readers ordered RGB Image, raw-depth Image, then
  CameraInfo. It loads `libfathom_component.so` and references
  `config/perception/fathom_component.pb.txt`.
- Added the sample config using Autosim camera inputs and explicit
  `/perception/fathom/` outputs. Its `/models/fathom.onnx` path is a replaceable
  placeholder; no model artifact was added.
- Added the DAG as its own mainboard process in `perception.launch`; the
  existing `autonomy.perception` binary remains unchanged.

## Build integration

- Excluded `component.cpp` from the globbed `libautonomy` source set so the
  registration entrypoint is compiled only into `fathom_component`.
- Added the loadable shared library under the same `BUILD_ONNXRUNTIME` and
  `OnnxRuntime_FOUND` condition as the concrete Fathom runner, linking it to
  `autonomy`, `autolink`, and `automsgs`.
- Kept config translation in `libautonomy`, allowing its focused test to remain
  available without the concrete ONNX runtime.

## Source tests and review

- Added focused tests for valid config translation, empty output-topic
  rejection, and inherited Fathom deployment-profile validation.
- No build, CMake configure, compilation, test execution, model download, or
  inference was run, per the explicit no-run constraint.
- Performed read-only diff inspection and `git diff --check` review only.

## Main-checkout recovery

The patch helper initially resolved relative paths against the main checkout.
The following exact accidental untracked files were moved, without deletion,
to `/private/tmp/codex-fathom-component-accidental/` preserving their relative
layout:

- `autonomy/perception/fathom/component_test.cpp`
- `autonomy/perception/fathom/proto/fathom_component_config.proto`
- `autonomy/perception/fathom/component_config.hpp`
- `autonomy/perception/fathom/component_config.cpp`
- `autonomy/perception/fathom/component.hpp`
- `autonomy/perception/fathom/component.cpp`
- `autonomy/perception/fathom/CMakeLists.txt`
- `autonomy/perception/fathom/fathom.dag`
- `config/perception/fathom_component.pb.txt`

No tracked or pre-existing main-checkout files were modified or removed.
