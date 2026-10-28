# 07 开发规范

## 文件与命名

- Atlas 自有 C++ 文件使用 `.hpp` 和 `.cpp`。
- 文件名使用 lower_snake_case。
- 类型使用 PascalCase，函数和变量遵循工程现有 Google C++ 风格。
- ORB-SLAM3 派生代码位于
  `autonomy::localization::atlas`，不恢复上游 ORB_SLAM3 命名空间。

## 模块边界

- 通用基础设施放入 `atlas/common`。
- 图优化实现统一放入 `atlas/common/optimization`。
- 特征提取放入 `feature`，匹配策略放入 `match`。
- AutoMsgs 只在 sensor 数据边界和消息适配层出现。
- 不修改 Voxel-SLAM、Argus 或无关 localization 模块。

## 格式化

使用 autonomy 工程根目录的 `.clang-format`：

```bash
find autonomy/localization/atlas/atlas \
     autonomy/localization/atlas/examples \
  -type f \( -name '*.hpp' -o -name '*.cpp' \) -print0 |
  xargs -0 clang-format -i \
    --style=file:/workspace/autonomy/src/autonomy/.clang-format
```

提交前使用 `--dry-run --Werror` 检查全部 Atlas 自有 C++ 文件。

## 并发与性能

- OpenMP 并行区必须是可关闭、固定线程数且不存在共享写入的数据并行。
- 不在 OpenCV、Eigen、g2o 或已有后台线程内部创建嵌套并行。
- 优先减少深拷贝、过度 reserve 和临时对象，再增加并行度。
- 描述子共享只能用于构造后只读的数据。

## 修改后的最小验证

1. 确认旧路径和旧 include 不再出现。
2. 运行全量 clang-format 检查。
3. 在 `SpaceHero` 中执行
   `colcon build --packages-up-to autonomy`。
4. 至少运行一个普通 EuRoC 序列和一个困难序列。
5. 算法或并发变更需要同时报告 CPU、RSS 与 ATE。
