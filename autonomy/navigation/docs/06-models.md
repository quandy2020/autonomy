# 06 · 模型制品

## DeFM ViT-S/14

当前制品位于 navigation 的 `conf/models/`。2026-11-09 在 SpaceHero 中重新导出并验证：

| 文件 | 字节数 | SHA-256 |
| --- | ---: | --- |
| `defm_vit_s14.pth` | 86,599,124 | `37a6e95befea3a16732a743b2ebec854fd5eaed912ebaf9fbffc63a2306f1e90` |
| `defm_vit_s14.onnx` | 86,625,968 | `dafdb6557600a26e01e77c795262c0c880546765599ae2b62af5a52e34034dd2` |

来源为 `leggedrobotics/defm`，checkpoint revision
`b6c9eb68ac833ccd93bf2571279235cb9b065251`，许可证见 `LICENSE.defm`（Apache-2.0）。
权重已被 Git 忽略；manifest 为 [defm.pb.txt](../conf/models/defm.pb.txt)。

导出使用 opset 17、float32 静态接口：

```text
depth  [1,3,224,224]
tokens [1,256,384]
pooled [1,384]
```

`EngineDefmModel::Load` 在创建 Engine 前校验文件 SHA-256，加载后核对张量名称、
dtype 和完整形状。缺文件、错误哈希、错误元数据均拒绝加载；加载失败会清除旧 Engine。
Run 拒绝错误输入长度与 NaN/Inf。

## 导出与数值验收

在 colcon workspace 执行，使用已准备的导出 venv；默认源码来自 navigation 的
`python/thirdparty/defm`，不需要另找上游代码目录：

```sh
cd /workspace/autonomy
.venvs/defm-export/bin/python src/autonomy/autonomy/navigation/tools/export_defm_onnx.py \
  --validate --update-manifest \
  --reference-dir /tmp/navigation-defm-reference \
  --report /tmp/navigation-defm-export.json
NAVIGATION_CONF_DIR=/workspace/autonomy/src/autonomy/autonomy/navigation \
  build/autonomy/bin/navigation_defm_validator \
  src/autonomy/autonomy/navigation/conf/models/defm.pb.txt \
  /tmp/navigation-defm-reference
```

导出脚本先生成临时文件，通过 ONNX checker 与数值检查后替换最终文件。
参考输入和 Python ORT 输出保存为 little-endian float32 文件，供 C++ 工具读取。

以下是 PyTorch 2.10.0 与 Python ORT 1.23.2 在 CPU 上的最大绝对误差：

| 输入 | tokens | pooled |
| --- | ---: | ---: |
| zeros | 2.4557114e-5 | 2.8312206e-6 |
| random | 1.3589859e-5 | 2.8610229e-6 |
| ramp | 3.5154819e-4 | 1.8417835e-5 |

跨框架 float32 检查阈值为 `--pytorch-atol 5e-4`。ramp 的 tokens 误差超过 1e-4，
因此不能宣称 PyTorch 与 ORT 所有测试均满足 1e-4。
独立的 **C++ Engine 与 Python ORT** 检查仍要求误差 `<1e-4`；本轮三组输入的
tokens/pooled 最大误差均为 **0**。该验证覆盖 CPU，不包含 CUDA/TensorRT。

## 深度预处理

`DepthPreprocessor` 使用上游三通道 log-depth 变换，直接 resize 到 224×224：

```text
C1 = log1p(depth) / log1p(100)
C2 = clip(log1p(depth) / log1p(9), 0, 1)
C3 = (log1p(depth) - min_log) / (max_log - min_log)
mean = [0.248880, 0.495620, 0.492858]
std  = [0.139357, 0.271314, 0.297177]
```

缩小使用三角抗锯齿滤波，放大使用双线性，与 torchvision Resize 并非逐位一致。
上面的 Engine 比较使用相同的已预处理张量，不能替代端到端预处理对齐验收。

## VNTPA planner

`conf/models/vntpa.onnx` 尚未提供；[vntpa.pb.txt](../conf/models/vntpa.pb.txt) 的哈希仍是占位。
单 planner 的当前接口与待办见 [15-vntpa-alignment.md](15-vntpa-alignment.md)。
DeFM 验证通过不代表 VNTPA 权重、真实导航策略或论文 SR/SPL 已通过。

## FrontierNet（探索头）

官方 `rgbd_11cls.pth`（ResNet-34 UNet，11 类 + 距离场）**不进仓库**：上游仓库没有
LICENSE 文件，再分发许可未确认。本地缓存后用 `python/frontiernet/` 的
`baseline` / `align` 命令加载，详见 [17-frontiernet.md](17-frontiernet.md)。
DeFM 变体复用同一份 decoder/两个头权重，encoder 换成 `conf/models/defm_vit_s14.pth`。
