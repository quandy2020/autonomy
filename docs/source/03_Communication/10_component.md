# 10. 组件 Component

生产环境推荐的模块形态：编 `.so`，在 DAG 里声明输入 channel，由 Scheduler 调 `Proc()`。支持 1–4 路输入（`Component<M0,…,M3>`）。 Component 把输入 channel 写进 DAG，由 Scheduler 调 `Proc()`，是车上推荐的模块形态。多路输入时注意时间对齐与丢帧策略，否则 fusion 会在控制环里表现为抖动。

## 流程

```text
mainboard LoadLibrary → Initialize(注入 node_) → Init()
→ DataVisitor 对齐 readers → Scheduler 调 Proc() → Writer 输出
```

## 实现要点

```cpp
class CommonComponentSample : public Component<Chatter, Chatter> {
    bool Init() override { return true; }
    bool Proc(const std::shared_ptr<Chatter>& m0,
        const std::shared_ptr<Chatter>& m1) override {
        // 多路齐备后触发
        return true;
    }
};
AUTOLINK_REGISTER_COMPONENT(CommonComponentSample)
```

| 规则 | 说明 |
|------|------|
| 勿在 `Init` 里 `CreateNode` | 使用框架注入的 `node_` |
| Writer 在 `Init` 创建 | channel 名可硬编码或读 conf |
| `Proc` 勿阻塞 | 重逻辑移交工作线程 |

## 运行

```bash
mainboard -d …/common.dag
# 或 autolink_launch …/common.launch
```

| 维度 | Binary | Component |
|------|--------|-----------|
| 配置 | channel 硬编码 | DAG 改配置 |
| 调度 | 自建 `Rate` | Scheduler |
| 用途 | 示例、工具 | 生产模块 |
