# 部署

## 1. 导出 ONNX

```bash
python -m vntpa.cli.export \
  --checkpoint checkpoints/vntpa_vit_s14/best.pt \
  --out exports/vntpa.onnx \
  --context-size 64
```

同时生成：

```text
exports/vntpa.onnx.json
```

## 2. 模型契约

```text
kf_tokens  [64,256,384]
kf_pose    [64,3]
kf_valid   [64]
query_pose [1,3]
goal       [1,2]
logits     [1,8,4]
```

DeFM：

```text
depth [1,3,224,224]
tokens [1,256,384]
pooled [1,384]
```

## 3. 校验

```python
from vntpa.runtime.deployment.onnx_validator import inspect_onnx

info = inspect_onnx("exports/vntpa.onnx")
print(info.inputs)
print(info.outputs)
```

`OnnxPlanner` 启动时会自动校验输入/输出名称，不匹配直接报错。

## 4. Python 推理

```python
from vntpa import VNT

model = VNT.from_pretrained(
    "exports/vntpa.onnx",
    "exports/defm_vit_s14.onnx",
)
result = model.predict(depth, pose, goal)
print(result.actions)
```

## 5. autosim 部署

```bash
export PYTHONPATH=\
/path/to/automsgs/proto/gen/python:\
/path/to/autolink/autolink/python:\
/path/to/autonomy/build/autonomy/navigation/generated/python:\
$PWD

python -m vntpa.runtime.sim.autosim_client \
  --planner exports/vntpa.onnx \
  --defm exports/defm_vit_s14.onnx
```

## 6. C++ 部署

C++ `navigation` 使用：

```text
conf/models/vntpa.pb.txt
conf/models/vntpa.onnx
conf/models/defm.pb.txt
conf/models/defm_vit_s14.onnx
```

manifest 中的 SHA 必须替换为实际模型哈希。

## 7. 当前限制

| 格式 | 状态 |
| --- | --- |
| ONNX | ✅ |
| TensorRT | ❌ 显式报错 |
| OpenVINO | ❌ |

TensorRT 需要外部 `trtexec`，当前不伪造支持。
