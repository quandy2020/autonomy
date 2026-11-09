# Architecture

`vntpa` follows the useful parts of the Ultralytics layout without copying
its task-specific modules.

```text
vntpa/
├── cfg/        # reserved for packaged defaults
├── cli/        # command entry points
├── data/       # datasets, transforms, context, collection
├── engine/     # model, predictor, trainer, validator, exporter
├── models/     # VNT-PA + DeFM network adapters
├── simulator/  # autolink/autosim bridge
└── utils/      # config, errors, logging, maths
```

## Ultralytics mapping

| Ultralytics | vntpa | Purpose |
| --- | --- | --- |
| `ultralytics/engine/model.py` | `vntpa/engine/model.py` | `VNT` facade |
| `engine/predictor.py` | `engine/predictor.py` | inference loop |
| `engine/trainer.py` | `engine/trainer.py` | training |
| `engine/validator.py` | `engine/validator.py` | evaluation result |
| `engine/metrics.py` | `engine/metric.py` | success/SPL/benchmark |
| `engine/exporter.py` | `engine/deployment/exporter.py` | ONNX export |
| `engine/result.py` | `engine/result.py` | result object |
| `cfg/default.yaml` | `configs/train/default.yaml` | defaults |
| `data/` | `data/` | datasets/transforms |
| `models/` | `models/` | network builders |
| `utils/` | `utils/` | shared helpers |

## Design rules

1. **One task facade**: `VNT` in `engine/model.py`.
2. **Engine owns lifecycle**: predictor, trainer, validator, exporter.
3. **Models own tensors**: encoder/planner/builders only.
4. **Data owns storage**: dataset, transforms, memory, collection.
5. **Utils has no engine dependency**.
6. **Stateful classes only**: sessions, memory, autolink node, exporter wrapper.
7. **Pure logic is functions**: metrics, transforms, hooks, config lookup.

## Runtime flow

```text
depth -> data.transforms -> models.encoder -> engine.predictor
                                         -> models.vntpa
                                         -> engine.result
                                         -> simulator action adapter -> /cmd_vel
```

## Training flow

```text
configs -> engine.trainer -> torchrun -> thirdparty.pan.planner.train
                                  -> checkpoint hook
                                  -> metrics / TensorBoard
```
