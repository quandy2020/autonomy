# 2. 贡献指南

第一次提 PR 或不确定格式/分支约定。

### 最短流程

```bash
git checkout -b feature/short-topic
python3 tools/python/clang_format_sources.py    # 改 C++ 后
cmake -G Ninja -B build && ninja -C build -j$(nproc)  # 至少编你动到的域
git commit -m "feat: 一句话动机"
```

推送后在 GitHub 开 PR：写清动机、如何测、是否动到 `BUILD_GRPC` / 默认 conf。

### 提交前缀（习惯）

| 前缀 | 用途 |
|------|------|
| `feat:` | 新能力 |
| `fix:` | 修 bug |
| `docs:` | 仅文档 |
| `refactor:` / `test:` / `chore:` | 重构 / 测试 / 工具 |

### 要求摘要

- C++17；根目录 `.clang-format`（Google 基、4 空格、80 列）  
- 新 RPC：改 `automsgs/proto/rpcs` + Handler + 文档 rpc 页  
- 许可：Apache-2.0（根 `LICENSE`）  
- 不要提交密钥、本地 `build/` 产物

### 常见坑

| 坑 | 说明 |
|----|------|
| 只跑 `--check` format 本地未 format | CI 会红 |
| 关 BUILD_GRPC 测 Bridge | 目标不存在，应开 ON 或注明不测 Bridge |
| 文档写 `autonomy/navigator` | 应用 `autonomy/task/` 路径 |
