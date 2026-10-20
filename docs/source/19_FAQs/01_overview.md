(faq-overview)=
# 1. FAQ 概览

安装、编译、launch、Bridge、导航 BT 报错，按**关键词**比按章节号更快定位；本节只做索引。

先判断是 **依赖/CMake**、**运行时路径/launch**、还是 **Task/BT/Bridge** 逻辑问题；`BUILD_GRPC` 与 `BUILD_AUTOVIZ` 分开排查。更长表见 Installation/Running 排错章。

## 问答

### 索引表

| 你看到… | 打开 |
|---------|------|
| apt 失败、依赖脚本报错 | [02 安装](02_installation.md) |
| `Could NOT find …`、链接错误 | [03 编译](03_build.md) |
| 找不到 `.so`、launch 起不来 | [04 运行](04_running.md) |
| 容器、GPU、挂载 | [05 Docker](05_docker.md) |
| `No module named`、tcmalloc、channel | [06 Autolink](06_autolink.md) |
| GetPlan 失败、BT 插件、车不动 | [07 导航](07_navigation.md) |

### 延伸阅读

- [Installation §8 排错](../02_Installation/08_troubleshooting.md)  
- [Running §8 排错](../04_Running/08_troubleshooting.md)  
- Issues：[github.com/quandy2020/autonomy/issues](https://github.com/quandy2020/autonomy/issues)
