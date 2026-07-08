# 8. 工具综述

### 8.1 工具矩阵

| 工具 | 类型 | 状态 | 文档 |
|------|------|------|------|
| `format.py` | 开发 | ✅ | [§3](03_dev_scripts.md) |
| `install_deps` | 开发 | ✅ | [§3](03_dev_scripts.md) |
| `autonomy_nav_test` | 测试 | ⚠️ CMake 接入待确认 | [§4](04_nav_test.md) |
| `MonitorRegistry` | 运维 | ✅ | [§5](05_monitor.md) |
| Autolink CLI | 调试 | ⚠️ 嵌入构建默认关闭 | [§6](06_autolink_tools.md) |
| `run_autonomy.py` | Docker | ✅ | [§7](07_docker_scripts.md) |
| `autonomy_planning_test` | 测试 | ❌ 未实现 | — |
| `autonomy_controller_test` | 测试 | ❌ 未实现 | — |

### 8.2 按场景选型

| 场景 | 推荐工具 |
|------|----------|
| 首次搭建环境 | `install_deps` + `run_autonomy.py` |
| 提交前检查格式 | `format.py --check` |
| 端到端导航验证 | `autonomy_nav_test` |
| 通道/话题调试 | Autolink CLI（独立构建） |
| 生产监控 | `MonitorRegistry` + Prometheus |

### 8.3 相关文档

- [04 Running](../04_Running/index.rst)
- [19 FAQs](../19_FAQs/index.rst)
- [20 Other · 贡献指南](../20_Other/03_contributing.md)
