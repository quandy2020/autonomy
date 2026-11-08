# 09 · 代码规范

参考：`autonomy/bridge/policy/metadata/metadata_validator.hpp`

## Include

- 工程内头文件一律使用**从 include root 开始的完整路径**，禁止相对 include：
  ```cpp
  #include "navigation/common/pose2d.hpp"
  #include "navigation/planning/interface/policy_interface.hpp"
  #include "automsgs/msgs/geometry_msgs/twist.pb.h"
  #include "autonomy/common/math/math.hpp"
  #include "autolink/autolink.hpp"
  ```
- 标准库用 `<...>`；第三方库用其安装路径（如 `google/protobuf/text_format.h`）。
- navigation 的 include root 是 `autonomy/`，因此包内头文件写作
  `navigation/...`；跨模块写作 `autonomy/...`、`automsgs/...`、`autolink/...`。
- 不允许 `#include "pose2d.hpp"`、`#include "../foo.hpp"`。

## 文件

```cpp
/*
 * Copyright 2026 The Navigation Authors (duyongquan)
 */

/**
 * @file xxx.hpp
 * @brief 一句话说明职责。
 */

#pragma once

#include <...>

#include "navigation/..."
```

- 版权块、`@file`、`@brief` 固定在文件顶部。
- 头文件保护统一 `#pragma once`。
- 命名空间逐层展开：`namespace navigation {` / `namespace planning {`，结尾逐层注释。
- 4 空格缩进；函数体左括号独占一行；行宽 80。

## 类 / 结构体

```cpp
/**
 * @brief 一句话职责。
 *
 * @details
 * 关键算法、边界条件、失败语义。
 *
 * @par Threading
 * 调用线程 / 并发约束。
 *
 * @par Ownership
 * 持有/非持有关系。
 */
class Foo
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(Foo)

    /** @brief 成员说明。 */
    int value{0};
};
```

- 每个类/结构体必须有 `@brief`；复杂类补 `@details`、`@par Threading`、`@par Ownership`。
- 每个**成员变量**上方必须有 `/** @brief ... */`，带单位/取值范围。
- 每个**公有函数**必须有 Doxygen：
  - `@brief` 一句话；
  - `@param[in]` / `@param[out]`；
  - `@return`（非 void）；
  - `@note` / `@warning` 仅在必要时；
  - `@par Threading` 如跨线程。
- 私有函数若逻辑不直观也补 `@brief`。
- 枚举值逐个注释（`///<` 或上方 `/** @brief */`）。

## 定义（.cpp）

- 只放实现；不重复头文件 Doxygen。
- 匿名命名空间辅助函数加 `/** @brief ... */`。
- 禁止 `using namespace`（测试文件可用 `// NOLINT` 标注）。

## 命名

- 类型 `PascalCase`；函数 `PascalCase`；变量/成员 `snake_case`，成员后缀 `_`，带单位。
- 常量 `kPascalCase`；宏 `AUTONOMY_*`。

## 检查清单

- [ ] 文件头四件套
- [ ] `#pragma once`
- [ ] 类/结构体 `@brief`
- [ ] 所有成员变量注释
- [ ] 所有公有函数参数/返回值注释
- [ ] 单位与取值范围
- [ ] 线程/所有权说明
