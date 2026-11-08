# 06 · 模型制品

## DeFM ViT-S/14 (used by VNTPA)

| File | Size | SHA-256 |
| --- | --- | --- |
| [defm_vit_s14.pth](../conf/models/defm_vit_s14.pth) | 86,599,124 B | `37a6e95befea3a16732a743b2ebec854fd5eaed912ebaf9fbffc63a2306f1e90` |
| [defm_vit_s14.onnx](../conf/models/defm_vit_s14.onnx) | 86,624,446 B | `bea909a58f645bb3009348d6431e1ba9c9a824a1a76a51dcfaffd5cbc14dc7f4` |

- Upstream checkpoint: [leggedrobotics/defm](https://huggingface.co/leggedrobotics/defm)
  [defm_vit_s14.pth](../conf/models/defm_vit_s14.pth)（Apache-2.0）。
- ONNX 导出：opset 17，输入 `depth [1,3,224,224]`，输出
  `tokens [1,256,384]`、`pooled [1,384]`；PyTorch vs ONNX 最大误差约
  `1.2e-5`（tokens）/ `4.3e-6`（pooled）。
- 推理 manifest：[defm.pb.txt](../conf/models/defm.pb.txt)。
- `.pth` / `.onnx` 约 83 MB，已加入根 `.gitignore`；仓库只提交 manifest 和
  `LICENSE.defm`，权重由开发者本地导出/下载。

```bash
# 重新导出（需要上游 DeFM 代码 + PyTorch）
python3 ../tools/export_defm_onnx.py --defm-source /path/to/VNT-PA/third_party/defm
```

## VNTPA planner

`conf/models/vntpa.onnx` 仍是外部制品：上游 VNT-PA 仓库未发布预训练 planner checkpoint，
需要用 `../tools/vntpa/` 训练并导出，或提供内部制品地址。[vntpa.pb.txt](../conf/models/vntpa.pb.txt)
目前是占位 sha256。
