# Transform Query 完整文档

> Transform Query 是一个强大的命令行工具，用于查询和显示ROS/Autonomy系统中任意两个坐标系之间的变换关系。

## 目录

- [1. 简介](#1-简介)
- [2. 功能特性](#2-功能特性)
- [3. 编译和安装](#3-编译和安装)
- [4. 使用指南](#4-使用指南)
- [5. 代码架构](#5-代码架构)
- [6. 测试说明](#6-测试说明)
- [7. 开发文档](#7-开发文档)
- [8. 常见问题](#8-常见问题)

---

## 1. 简介

`transform_query` (原名 `tf_query`) 是Autonomy框架中的TF变换查询工具，提供了丰富的功能来查询、显示和监控坐标系之间的变换关系。

### 主要用途

- 🔍 **调试TF树**：快速检查坐标系关系
- 📊 **数据分析**：记录和分析TF数据
- 🎯 **实时监控**：连续监控TF变化
- 🧪 **验证配置**：确认静态TF配置正确性

---

## 2. 功能特性

### 核心功能

- ✅ **多种显示格式**
  - 四元数格式（默认）
  - 欧拉角格式（Roll, Pitch, Yaw）
  - 4x4变换矩阵
  - 详细信息（包含范数、距离等）

- ✅ **查询模式**
  - 单次查询
  - 连续监控模式
  - 可配置更新频率

- ✅ **美化输出**
  - Unicode表格格式
  - 清晰的数据展示
  - 彩色日志支持

- ✅ **高级功能**
  - 查询超时配置
  - 启动延迟等待
  - 坐标系列表

### 与ROS tf_echo对比

| 功能 | ROS tf_echo | Transform Query |
|------|-------------|-----------------|
| 基础查询 | ✅ | ✅ |
| 连续监控 | ✅ | ✅ |
| 欧拉角显示 | ✅ | ✅ |
| 矩阵格式 | ❌ | ✅ |
| 详细输出 | 部分 | ✅ |
| 美化输出 | ❌ | ✅ |
| 自定义频率 | ❌ | ✅ |
| 范数检查 | ❌ | ✅ |
| 距离计算 | ❌ | ✅ |

---

## 3. 编译和安装

### 编译工具

```bash
cd build
ninja autonomy.transform.transform_query
```

### 编译测试

```bash
ninja autonomy.transform.transform_query_test
```

### 运行测试

```bash
./bin/autonomy.transform.transform_query_test
```

**预期输出**：
```
[==========] Running 16 tests from 5 test suites.
[  PASSED  ] 16 tests.
```

---

## 4. 使用指南

### 4.1 基础用法

#### 单次查询

查询两个坐标系之间的变换：

```bash
./bin/autonomy.transform.transform_query \
  --source_frame=base_link \
  --target_frame=map
```

**输出示例**：
```
╔════════════════════════════════════════════════════════════════╗
║              Transform: map → base_link                        ║
╠════════════════════════════════════════════════════════════════╣
║ Translation:                                                   ║
║   x:   1.234567 m                                              ║
║   y:   2.345678 m                                              ║
║   z:   0.123456 m                                              ║
║                                                                ║
║ Rotation (Quaternion):                                         ║
║   x:   0.000000                                                ║
║   y:   0.000000                                                ║
║   z:   0.707107                                                ║
║   w:   0.707107                                                ║
╚════════════════════════════════════════════════════════════════╝
```

#### 显示欧拉角

```bash
./bin/autonomy.transform.transform_query \
  --source_frame=base_link \
  --target_frame=map \
  --euler_angles
```

添加欧拉角输出：
```
║ Rotation (Euler Angles - ZYX):                                 ║
║   Roll  (X):   0.000000 rad (   0.000°)                        ║
║   Pitch (Y):   0.000000 rad (   0.000°)                        ║
║   Yaw   (Z):   1.570796 rad (  90.000°)                        ║
```

#### 矩阵格式显示

```bash
./bin/autonomy.transform.transform_query \
  --source_frame=base_link \
  --target_frame=map \
  --matrix_format
```

**输出**：
```
Transformation Matrix (4x4):
╔                                                           ╗
║     0.000000     -1.000000      0.000000      1.234567   ║
║     1.000000      0.000000      0.000000      2.345678   ║
║     0.000000      0.000000      1.000000      0.123456   ║
║     0.000000      0.000000      0.000000      1.000000   ║
╚                                                           ╝
```

### 4.2 高级用法

#### 详细输出模式

```bash
./bin/autonomy.transform.transform_query \
  --source_frame=base_link \
  --target_frame=map \
  --verbose_output
```

包含：
- 完整的时间戳信息
- 四元数范数验证
- 平移向量的欧几里得距离
- 同时显示四元数和欧拉角

#### 连续监控模式

```bash
./bin/autonomy.transform.transform_query \
  --source_frame=base_link \
  --target_frame=odom \
  --continuous \
  --rate=10
```

- 实时监控TF变化
- 10Hz更新频率
- 按 `Ctrl+C` 退出

#### 组合使用多个选项

```bash
./bin/autonomy.transform.transform_query \
  --source_frame=base_link \
  --target_frame=map \
  --verbose_output \
  --euler_angles \
  --continuous \
  --rate=5 \
  --timeout=0.5
```

### 4.3 命令行参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--source_frame` | string | **必需** | 源坐标系ID |
| `--target_frame` | string | **必需** | 目标坐标系ID |
| `--timeout` | double | 1.0 | 查询超时时间（秒） |
| `--continuous` | bool | false | 启用连续监控模式 |
| `--rate` | double | 1.0 | 连续模式更新频率（Hz） |
| `--verbose_output` | bool | false | 显示详细信息 |
| `--euler_angles` | bool | false | 显示欧拉角 |
| `--matrix_format` | bool | false | 以矩阵格式显示 |
| `--list_frames` | bool | false | 列出所有可用坐标系 |
| `--wait_time` | double | 0.0 | 查询前等待时间（秒） |

### 4.4 使用场景

#### 场景1：调试TF树

```bash
./bin/autonomy.transform.transform_query \
  --source_frame=camera_link \
  --target_frame=base_link \
  --verbose_output
```

#### 场景2：验证静态TF配置

```bash
./bin/autonomy.transform.transform_query \
  --source_frame=laser \
  --target_frame=base_link \
  --matrix_format
```

#### 场景3：监控动态TF变化

```bash
./bin/autonomy.transform.transform_query \
  --source_frame=base_link \
  --target_frame=map \
  --continuous \
  --rate=10 \
  --euler_angles
```

#### 场景4：记录TF数据

```bash
./bin/autonomy.transform.transform_query \
  --source_frame=base_link \
  --target_frame=odom \
  --continuous \
  --rate=1 \
  > robot_trajectory_$(date +%Y%m%d_%H%M%S).log 2>&1
```

### 4.5 Docker环境使用

```bash
docker exec SpaceHero /bin/bash -c "cd /workspace/autonomy/build && \
  ./bin/autonomy.transform.transform_query \
  --source_frame=base_link \
  --target_frame=map"
```

---

## 5. 代码架构

### 5.1 文件结构

#### 当前结构（重构后）

```
autonomy/transform/
├── transform_query.hpp          (157行) - 接口定义
├── transform_query.cpp          (408行) - 功能实现
├── transform_query_main.cpp     (108行) - 程序入口
├── transform_query_test.cpp     (346行) - 单元测试
└── TRANSFORM_QUERY.md                    - 本文档
```

#### 历史演进

### 5.2 类设计

#### TransformQueryOptions

配置选项结构体，封装所有命令行参数：

```cpp
struct TransformQueryOptions {
    // 必需参数
    std::string source_frame;
    std::string target_frame;
    
    // 查询选项
    double timeout = 1.0;
    double wait_time = 0.0;
    
    // 显示选项
    bool verbose_output = false;
    bool euler_angles = false;
    bool matrix_format = false;
    
    // 连续模式
    bool continuous = false;
    double rate = 1.0;
    
    // 其他
    bool list_frames = false;
};
```

#### TransformQuery

主查询类，提供所有查询功能：

```cpp
class TransformQuery {
public:
    explicit TransformQuery(const TransformQueryOptions& options);
    ~TransformQuery() = default;
    
    // 主运行函数
    int Run();
    
    // 查询和显示
    bool QueryAndDisplayTransform(...);
    
    // 输出格式
    void PrintTransformBasic(...);
    void PrintTransformMatrix(...);
    void PrintTransformVerbose(...);
    
    // 其他功能
    void ListAvailableFrames(...);
    
private:
    TransformQueryOptions options_;
};
```

#### 工具函数

```cpp
// 四元数转欧拉角
void QuaternionToEuler(double qx, double qy, double qz, double qw,
                       double& roll, double& pitch, double& yaw);

// 四元数转旋转矩阵
void QuaternionToRotationMatrix(double qx, double qy, double qz, double qw,
                                 double matrix[3][3]);
```

### 5.3 类图

```
┌─────────────────────────┐
│ TransformQueryOptions   │
├─────────────────────────┤
│ + source_frame: string  │
│ + target_frame: string  │
│ + timeout: double       │
│ + verbose_output: bool  │
│ + euler_angles: bool    │
│ + matrix_format: bool   │
│ + continuous: bool      │
│ + rate: double          │
│ + wait_time: double     │
│ + list_frames: bool     │
└─────────────────────────┘
           ▲
           │ uses
           │
┌─────────────────────────────────────────┐
│        TransformQuery                   │
├─────────────────────────────────────────┤
│ - options_: TransformQueryOptions       │
├─────────────────────────────────────────┤
│ + TransformQuery(options)               │
│ + Run(): int                            │
│ + QueryAndDisplayTransform(...): bool   │
│ + ListAvailableFrames(buffer)           │
│ + PrintTransformBasic(transform)        │
│ + PrintTransformMatrix(transform)       │
│ + PrintTransformVerbose(transform)      │
└─────────────────────────────────────────┘
```

### 5.4 设计优势

#### 1. 代码组织清晰
- **关注点分离**：接口、实现、入口分别在不同文件
- **模块化**：每个文件职责明确
- **可维护性**：修改实现不影响接口

#### 2. 易于扩展

添加新的显示格式示例：

```cpp
// 1. 在 TransformQueryOptions 中添加选项
struct TransformQueryOptions {
    bool json_format = false;  // 新增
};

// 2. 在 TransformQuery 中添加方法
class TransformQuery {
    void PrintTransformJson(...);  // 新增
};

// 3. 在 main 中添加参数
DEFINE_bool(json_format, false, "Display as JSON format");
```

#### 3. 易于测试

```cpp
// 可以单独测试工具类
TEST(TransformQueryTest, BasicQuery) {
    TransformQueryOptions options;
    options.source_frame = "base_link";
    options.target_frame = "map";
    
    TransformQuery query(options);
    // 测试逻辑...
}
```

#### 4. 易于复用

```cpp
// 其他程序可以直接使用
#include "autonomy/transform/transform_query.hpp"

void MyFunction() {
    TransformQueryOptions options;
    options.source_frame = "base_link";
    options.target_frame = "map";
    
    TransformQuery query(options);
    query.Run();
}
```

---

## 6. 测试说明

### 6.1 测试概览

- **测试文件**：`transform_query_test.cpp`
- **测试套件**：5个
- **测试用例**：16个
- **通过率**：100% ✅

### 6.2 测试套件详情

#### Suite 1: QuaternionToEulerTest (5个测试)

| 测试用例 | 说明 |
|---------|------|
| `IdentityQuaternion` | 测试单位四元数转换 |
| `QuarterRotationZ` | 测试绕Z轴90度旋转 |
| `QuarterRotationX` | 测试绕X轴90度旋转 |
| `QuarterRotationY` | 测试绕Y轴90度旋转 |
| `NegativeRotation` | 测试负角度旋转 |

#### Suite 2: QuaternionToRotationMatrixTest (4个测试)

| 测试用例 | 说明 |
|---------|------|
| `IdentityQuaternion` | 测试单位四元数生成单位矩阵 |
| `QuarterRotationZ` | 测试绕Z轴旋转的矩阵形式 |
| `MatrixOrthogonal` | 测试旋转矩阵的正交性 (R * R^T = I) |
| `Determinant` | 测试行列式为1 |

#### Suite 3: TransformQueryOptionsTest (2个测试)

| 测试用例 | 说明 |
|---------|------|
| `DefaultValues` | 测试默认配置值 |
| `CustomValues` | 测试自定义配置值 |

#### Suite 4: TransformQueryTest (3个测试)

| 测试用例 | 说明 |
|---------|------|
| `Construction` | 测试对象构造 |
| `EmptyFramesReturnError` | 测试空坐标系处理 |
| `OptionsPreserved` | 测试选项保存 |

#### Suite 5: TransformQueryIntegrationTest (2个测试)

| 测试用例 | 说明 |
|---------|------|
| `QuaternionEulerRoundTrip` | 测试四元数与欧拉角转换 |
| `MatrixQuaternionConsistency` | 测试矩阵与四元数一致性 |

### 6.3 测试覆盖率

| 类别 | 覆盖率 |
|------|--------|
| 数学工具函数 | 100% |
| 配置选项结构 | 100% |
| 类基础功能 | 80%+ |
| 错误处理 | ✅ |
| 边界条件 | ✅ |

### 6.4 测试质量验证

#### 数学验证
- ✅ 单位四元数正确性
- ✅ 旋转角度精度（1e-6）
- ✅ 矩阵正交性
- ✅ 行列式为1
- ✅ 欧拉角转换正确性

#### 边界测试
- ✅ 零旋转（单位四元数）
- ✅ 正向旋转
- ✅ 负向旋转
- ✅ 各轴独立旋转
- ✅ 空输入处理

#### 集成测试
- ✅ 多重转换一致性
- ✅ 矩阵-四元数-欧拉角三者一致

### 6.5 运行测试

```bash
# 编译测试
ninja autonomy.transform.transform_query_test

# 运行所有测试
./bin/autonomy.transform.transform_query_test

# 运行特定测试
./bin/autonomy.transform.transform_query_test \
  --gtest_filter=QuaternionToEulerTest.*
```

---

## 7. 开发文档

### 7.1 添加新功能

#### 示例：添加JSON输出格式

**步骤1**：在选项中添加标志

```cpp
// transform_query.hpp
struct TransformQueryOptions {
    // ... 现有选项
    bool json_format = false;  // 新增
};
```

**步骤2**：在类中添加方法

```cpp
// transform_query.hpp
class TransformQuery {
    // ... 现有方法
    void PrintTransformJson(const TransformStamped& transform);  // 新增
};
```

**步骤3**：实现方法

```cpp
// transform_query.cpp
void TransformQuery::PrintTransformJson(
    const commsgs::geometry_msgs::TransformStamped& transform)
{
    std::cout << "{\n";
    std::cout << "  \"frame_id\": \"" << transform.header.frame_id << "\",\n";
    // ... JSON格式输出
    std::cout << "}\n";
}
```

**步骤4**：在main中添加参数

```cpp
// transform_query_main.cpp
DEFINE_bool(json_format, false, "Display as JSON format");
```

**步骤5**：更新Run()逻辑

```cpp
// transform_query.cpp
if (options_.json_format) {
    PrintTransformJson(transform);
} else if (options_.matrix_format) {
    PrintTransformMatrix(transform);
}
// ...
```

### 7.2 编写测试

```cpp
// transform_query_test.cpp
TEST(TransformQueryTest, JsonFormat) {
    TransformQueryOptions options;
    options.source_frame = "base_link";
    options.target_frame = "map";
    options.json_format = true;
    
    TransformQuery query(options);
    // 测试JSON输出...
}
```

### 7.3 代码规范

- 使用一致的命名风格
- 添加完整的注释和文档
- 遵循C++17标准
- 使用智能指针管理资源
- 添加单元测试

---

## 8. 常见问题

### Q1: 提示找不到TF变换怎么办？

**原因**：
- TransformServer未运行
- TF发布器未启动
- 坐标系名称拼写错误
- 系统未初始化完成

**解决方案**：

```bash
./bin/autonomy.transform.transform_query \
  --source_frame=base_link \
  --target_frame=map \
  --timeout=5.0 \
  --wait_time=2.0
```

### Q2: 四元数的范数不是1.0？

使用详细输出查看范数：

```bash
./bin/autonomy.transform.transform_query \
  --source_frame=base_link \
  --target_frame=map \
  --verbose_output
```

如果范数明显偏离1.0，说明TF数据可能有问题。

### Q3: 如何理解欧拉角？

- **Roll (X轴)**：侧倾角，绕X轴旋转
- **Pitch (Y轴)**：俯仰角，绕Y轴旋转
- **Yaw (Z轴)**：偏航角，绕Z轴旋转

使用ZYX顺序（先绕Z轴，再Y轴，最后X轴）。

### Q4: 连续模式如何退出？

按 `Ctrl+C` 即可安全退出，程序会正确清理资源。

### Q5: 输出格式乱码？

确保终端支持UTF-8编码：

```bash
export LANG=en_US.UTF-8
./bin/autonomy.transform.transform_query --source_frame=A --target_frame=B
```

### Q6: 如何在程序中使用？

```cpp
#include "autonomy/transform/transform_query.hpp"

using namespace autonomy::transform;

int main() {
    TransformQueryOptions options;
    options.source_frame = "base_link";
    options.target_frame = "map";
    options.euler_angles = true;
    
    TransformQuery query(options);
    return query.Run();
}
```

---

## 9. 最佳实践

### 9.1 使用建议

1. **首次使用**：先用 `--verbose_output` 了解完整信息
2. **调试TF**：使用 `--matrix_format` 验证变换矩阵
3. **实时监控**：使用 `--continuous --rate=10` 适中的频率
4. **数据记录**：重定向到文件并包含时间戳
5. **自动化脚本**：结合 `--wait_time` 确保系统就绪

### 9.2 性能优化

- 连续模式下使用合理的频率（1-20Hz）
- 避免过长的超时时间
- 使用基础格式提高性能

### 9.3 调试技巧

```bash
# 1. 检查TF是否存在
./bin/autonomy.transform.transform_query --list_frames

# 2. 使用长超时和等待时间
./bin/autonomy.transform.transform_query \
  --source_frame=A --target_frame=B \
  --timeout=10.0 --wait_time=5.0

# 3. 使用详细输出诊断
./bin/autonomy.transform.transform_query \
  --source_frame=A --target_frame=B \
  --verbose_output
```



---

**文档维护者**: Autonomy开发团队  
**最后更新**: 2025年10月

