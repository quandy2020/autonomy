# 10. 组件 Component

推荐算法模块形态：`.so` + DAG，支持 1–4 路输入（`Component<M0,…,M3>`）。

## 10.1 流程

```text
mainboard LoadLibrary → Initialize(注入 node_) → Init()
→ DataVisitor 对齐 readers → Scheduler 调 Proc() → Writer 输出
```

## 10.2 实现要点

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

## 10.3 运行

```bash
mainboard -d …/common.dag
# 或 autolink_launch …/common.launch
```

| — | Binary | Component |
|---|--------|-----------|
| 配置 | channel 硬编码 | DAG 改配置 |
| 调度 | 自建 `Rate` | Scheduler |
| 用途 | 示例、工具 | 生产模块 |
