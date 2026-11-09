# 命名与布局

## 布局

- flat layout：`vntpa/` 直接位于工程根；
- 无 `src/`；
- 根目录只保留 `configs/docs/scripts/tests/thirdparty/vntpa`。

## 子包

```text
data       数据与采集
models     模型与构建器
engine     训练、评估、推理、部署
simulator  autolink / autosim
cli        命令入口
utils      配置与工具
```

## 命名

| 类型 | 规则 | 示例 |
| --- | --- | --- |
| 包/模块 | lower_snake_case | `query.py` |
| 类 | PascalCase | `OnnxPlanner` |
| 函数 | 动词或 `verb_noun`；能单词语义明确时用单词 | `context`、`validate` |
| 私有成员 | snake_case + 后缀下划线 | `twist_type_`、`frames_` |
| 私有函数 | `verb_noun`，lower_snake_case，无前导下划线 | `tensor`、`weights` |
| 常量 | UPPER_SNAKE_CASE | `DEFM_MEAN` |
| 配置 | `<domain>_<variant>.yaml` | `vntpa_vit_s14.yaml` |
| 模型 | `<family>_<backbone>` | `vntpa_vit_s14` |
| CLI | `vntpa.cli.<task>` | `vntpa.cli.export` |

## 禁止

- 变量/属性/函数使用前导下划线，例如 `self._twist_type`、`def _publish`；
- 变量/字段 `_m` 后缀；
- C++ 相对 include；
- 隐式网络下载；
- 空壳模块和重复配置；
- 把大模型权重提交到 Git。
