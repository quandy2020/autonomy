# 4. 版本记录

需要了解近期架构方向，又不替代仓库根目录的正式 CHANGELOG。

发版以 **tag + `CHANGELOG.rst`** 为准；线上对外命令面仅为 **gRPC Bridge**。文档章名 Navigator 对应代码 `autonomy/task/`。

### 阅读正式记录

```bash
cat CHANGELOG.rst | less
```

### 与文档相关的方向（摘要）

- 多进程 launch 取代进程内 `CreateAutonomy`  
- Task 栈 + BT 在 `autonomy/task/`  
- 对外命令面：**gRPC Bridge**  
- `tasks` 命名收敛到 navigator/task 文档章

本节仅为摘要，细节以 `CHANGELOG.rst` 为准。
