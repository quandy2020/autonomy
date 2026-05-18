# common/network

Model inference and shared preprocess/postprocess utilities. Namespace: `autonomy::common::network`.

## Layout

```
network/
├── network.hpp              # Application umbrella (recommended entry)
├── preprocess.hpp           # Preprocess public API
├── postprocess.hpp          # Postprocess public API
├── common/                  # Tensor, InferenceOptions
├── backend/                 # Backend, BackendFactory, Engine
│   ├── onnx/                # ONNX Runtime implementation
│   └── tensorrt/            # TensorRT implementation (optional)
├── pipeline/                # RunPipeline / RunResult
└── detail/                  # Implementation (not installed)
```

## Naming

| Category | API | Notes |
|----------|-----|-------|
| Inference | `Engine::Run` | Application-facing entry |
| Factory | `BackendFactory::Create` | Internal registration |
| Pipeline | `RunPipeline` | Preprocess + inference |
| Spatial size | `GetSpatialSize` | H×W from input metadata |
| Output pick | `FindFloatOutput` | float32 output view by name/keyword |
| Conversion | `FromFloatTensorMap` / `ToFloatTensorMap` | float map ↔ `TensorMap` |
| Preprocess I/O | `FromPreprocessFloat` | float pipeline → model element type |
| Warmup | `Engine::Warmup` | Static-shape models only |

## Headers

| Need | Include |
|------|---------|
| Default | `network.hpp` |
| Inference only | `backend/engine.hpp` |
| Extend backend | `backend/backend.hpp` + `backend/onnx/onnx.hpp`, etc. |

## Runtime types

- I/O uses `Tensor` / `TensorMap` (float32, float16, bfloat16, int8, int32, int64, uint8).
- `Engine::Run(FloatTensorMap, …)` is a float32 convenience overload.
- Preprocess converts float pipelines to the model input `element_type` via `FromPreprocessFloat`.
- For calibrated INT8/quantized inputs, supply `Sample::named_tensors` instead of relying on naive cast.

## ONNX Runtime

`OnnxRuntimeOptions`:

| Field | Description |
|-------|-------------|
| `execution_provider` | `""` / `"cpu"` (default) or `"cuda"` |
| `device_id` | CUDA device index when using `"cuda"` |
| `use_io_binding` | Pre-bind static CPU outputs (skip output memcpy when shapes are fixed) |
| `intra_op_num_threads` / `inter_op_num_threads` | 0 = ORT default |
| `graph_optimization_level` | -1 = default, 0–3 = ORT levels |

CUDA EP requires an ONNX Runtime build that includes the CUDA provider.

## Thread safety

`Engine` and `Backend` are **not** thread-safe. Use one engine per thread or external locking.

## Dynamic shapes

- `ResolveShapeForElementCount` supports a single dynamic dimension per tensor.
- `Engine::Warmup` is a no-op (returns success) when any input shape is not fully static.
- IoBinding is used only when `use_io_binding` is true and every output shape is fully static.

## Postprocess scope

See the table in `postprocess.hpp`. `Decode` targets grid-style detection heads only.

## Build

`NETWORK_CXX17_SRCS` in `CMakeLists.txt` lists module sources. `detail/**` and `backend/onnx/io.hpp` are not installed.
