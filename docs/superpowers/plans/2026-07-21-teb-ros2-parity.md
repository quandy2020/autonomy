# TEB ROS2 全功能对齐 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 ROS2 `teb_local_planner` 的 Homotopy 多拓扑规划、FailureDetector 振荡恢复、以及点/线/多边形障碍物转换接到本仓库 `teb_controller`，使 `Optimizer` 可按配置在单/多拓扑间切换。

**Architecture:** 直迁上游算法到 `core/`（去 rclcpp）；`Optimizer` 持有 `PlannerInterface` + `FailureDetector` + 增强版 `CostmapObstacleConverter`；图搜索保留 Boost.Graph；智能指针与 optional 用标准库。

**Tech Stack:** C++17、g2o、Eigen、Boost.Graph、protobuf、gtest、Docker `SpaceHero` 构建。

**Spec:** `docs/superpowers/specs/2026-07-21-teb-ros2-parity-design.md`

**对照源码根目录:** `/Users/quandy/Workspace/github/ros2/ros-planning/teb_local_planner/teb_local_planner`

---

## File map

| 文件 | 动作 | 职责 |
|------|------|------|
| `core/recovery_behaviors.hpp/.cpp` | Create | `FailureDetector` |
| `core/recovery_behaviors_test.cpp` | Create | 振荡检测单测 |
| `core/equivalence_relations.hpp` | Create | 拓扑等价接口 |
| `core/h_signature.hpp` | Create | H-signature |
| `core/graph_search.hpp/.cpp` | Create | Boost.Graph 搜索 |
| `core/homotopy_class_planner.hpp` | Create | 多拓扑 planner 声明 |
| `core/homotopy_class_planner.hpp` 旁 `homotopy_class_planner_impl.hpp` | Create | 模板实现（对应上游 `.hpp`） |
| `core/homotopy_class_planner.cpp` | Create | 非模板实现 |
| `optimizer.hpp/.cpp` | Modify | `PlannerInterface` 选型 + FailureDetector |
| `tools/costmap_obstacle_converter.hpp/.cpp` | Modify | 点/线/多边形 |
| `proto/teb_controller.proto` | Modify | Homotopy/Recovery/转换模式字段 |
| `tools/teb_options.cpp` | Modify | 解析新字段 |
| `core/teb_config.hpp` | 已有默认 | 由 Optimizer 从 proto 覆盖关键开关 |

**构建说明:** `libautonomy` 对 `autonomy/**/*.cpp` 为 glob 收录；新增 `.cpp` 后重新 cmake/build 即可，一般无需改 CMakeLists。

**不做:** `teb_local_planner_ros`、pluginlib、`costmap_converter` 插件、完整 Marker 可视化、重写 Boost.Graph。

---

### Task 1: FailureDetector + 单测

**Files:**
- Create: `autonomy/control/controller/teb_controller/core/recovery_behaviors.hpp`
- Create: `autonomy/control/controller/teb_controller/core/recovery_behaviors.cpp`
- Create: `autonomy/control/controller/teb_controller/core/recovery_behaviors_test.cpp`
- Source: `.../teb_local_planner/src/recovery_behaviors.cpp` + `include/.../recovery_behaviors.h`

- [ ] **Step 1: 写头文件（定长 deque 替代 circular_buffer）**

```cpp
// autonomy/control/controller/teb_controller/core/recovery_behaviors.hpp
#pragma once

#include <cstddef>
#include <deque>

#include "autonomy/control/controller/teb_controller/core/teb_core.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {

class FailureDetector {
 public:
  FailureDetector() = default;

  void setBufferLength(std::size_t length) {
    buffer_length_ = length;
    while (buffer_.size() > buffer_length_) {
      buffer_.pop_front();
    }
  }

  void update(const Twist& twist, double v_max, double v_backwards_max,
              double omega_max, double v_eps, double omega_eps);
  bool isOscillating() const { return oscillating_; }
  void clear();

 protected:
  struct VelMeasurement {
    double v = 0;
    double omega = 0;
  };

  bool detect(double v_eps, double omega_eps);

 private:
  std::deque<VelMeasurement> buffer_;
  std::size_t buffer_length_{0};
  bool oscillating_{false};
};

}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
```

- [ ] **Step 2: 写实现（逻辑对齐上游，字段改用 `Twist`）**

```cpp
// autonomy/control/controller/teb_controller/core/recovery_behaviors.cpp
#include "autonomy/control/controller/teb_controller/core/recovery_behaviors.hpp"

#include <cmath>
#include <g2o/stuff/misc.h>

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {

void FailureDetector::update(const Twist& twist, double v_max,
                             double v_backwards_max, double omega_max,
                             double v_eps, double omega_eps) {
  if (buffer_length_ == 0) {
    return;
  }
  VelMeasurement measurement;
  measurement.v = twist.linear.x;
  measurement.omega = twist.angular.z;
  if (measurement.v > 0 && v_max > 0) {
    measurement.v /= v_max;
  } else if (measurement.v < 0 && v_backwards_max > 0) {
    measurement.v /= v_backwards_max;
  }
  if (omega_max > 0) {
    measurement.omega /= omega_max;
  }
  buffer_.push_back(measurement);
  while (buffer_.size() > buffer_length_) {
    buffer_.pop_front();
  }
  detect(v_eps, omega_eps);
}

void FailureDetector::clear() {
  buffer_.clear();
  oscillating_ = false;
}

bool FailureDetector::detect(double v_eps, double omega_eps) {
  oscillating_ = false;
  if (buffer_length_ == 0 || buffer_.size() < buffer_length_ / 2) {
    return false;
  }
  const double n = static_cast<double>(buffer_.size());
  double v_mean = 0;
  double omega_mean = 0;
  int omega_zero_crossings = 0;
  for (std::size_t i = 0; i < buffer_.size(); ++i) {
    v_mean += buffer_[i].v;
    omega_mean += buffer_[i].omega;
    if (i > 0 &&
        g2o::sign(buffer_[i].omega) != g2o::sign(buffer_[i - 1].omega)) {
      ++omega_zero_crossings;
    }
  }
  v_mean /= n;
  omega_mean /= n;
  if (std::abs(v_mean) < v_eps && std::abs(omega_mean) < omega_eps &&
      omega_zero_crossings > 1) {
    oscillating_ = true;
  }
  return oscillating_;
}

}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
```

- [ ] **Step 3: 写单测**

```cpp
// autonomy/control/controller/teb_controller/core/recovery_behaviors_test.cpp
#include "autonomy/control/controller/teb_controller/core/recovery_behaviors.hpp"

#include "gtest/gtest.h"

namespace autonomy::control::controller::teb_controller {
namespace {

Twist MakeTwist(double vx, double omega) {
  Twist t;
  t.linear.x = vx;
  t.angular.z = omega;
  return t;
}

TEST(FailureDetectorTest, DetectsOscillationOnAlternatingOmega) {
  FailureDetector detector;
  detector.setBufferLength(10);
  // Normalized |v|,|omega| means stay small; alternate omega sign.
  for (int i = 0; i < 10; ++i) {
    const double omega = (i % 2 == 0) ? 0.05 : -0.05;
    detector.update(MakeTwist(0.02, omega), /*v_max=*/1.0,
                    /*v_backwards_max=*/1.0, /*omega_max=*/1.0,
                    /*v_eps=*/0.1, /*omega_eps=*/0.1);
  }
  EXPECT_TRUE(detector.isOscillating());
}

TEST(FailureDetectorTest, ClearResetsState) {
  FailureDetector detector;
  detector.setBufferLength(8);
  for (int i = 0; i < 8; ++i) {
    detector.update(MakeTwist(0.01, (i % 2) ? 0.05 : -0.05), 1, 1, 1, 0.1,
                    0.1);
  }
  ASSERT_TRUE(detector.isOscillating());
  detector.clear();
  EXPECT_FALSE(detector.isOscillating());
}

}  // namespace
}  // namespace autonomy::control::controller::teb_controller
```

- [ ] **Step 4: 编译并跑测**

```bash
docker exec SpaceHero bash -c 'cd /workspace/autonomy/build && cmake --build . --target recovery_behaviors_test -j$(nproc) 2>&1 | tail -30'
# 若无独立 target，则构建含 gtest 的对应测试二进制，或先并入已有 control 测试 target。
# 最低验收：autonomy 库能编译进 recovery_behaviors.cpp
docker exec SpaceHero bash -c 'cd /workspace/autonomy/build && cmake --build . --target autonomy -j$(nproc) 2>&1 | grep -E "error:|Built target autonomy" | tail -20'
```

Expected: `Built target autonomy`；单测若已接入则 PASS。

- [ ] **Step 5: Commit**

```bash
git add autonomy/control/controller/teb_controller/core/recovery_behaviors.hpp \
        autonomy/control/controller/teb_controller/core/recovery_behaviors.cpp \
        autonomy/control/controller/teb_controller/core/recovery_behaviors_test.cpp
git commit -m "$(cat <<'EOF'
feat(teb): add FailureDetector for oscillation recovery

Port ROS2 recovery_behaviors with std::deque instead of boost circular_buffer.
EOF
)"
```

---

### Task 2: Proto + teb_options 暴露开关

**Files:**
- Modify: `autonomy/control/proto/teb_controller.proto`
- Modify: `autonomy/control/controller/teb_controller/tools/teb_options.cpp`
- Modify: `autonomy/control/controller/teb_controller/optimizer.cpp`（`applyOptionsToConfig`）

- [ ] **Step 1: 扩展 proto（字段号接在 38 之后）**

```protobuf
  // Homotopy / recovery / obstacle conversion
  bool enable_homotopy_class_planning = 39;
  bool enable_multithreading = 40;
  int32 max_number_classes = 41;
  bool oscillation_recovery = 42;
  double oscillation_filter_duration = 43;
  // "points_only" | "points_lines_polygons"
  string obstacle_conversion_mode = 44;
```

- [ ] **Step 2: `teb_options.cpp` 解析**

在现有 `LoadOptions` 末尾增加（沿用文件内 `SetBoolIfPresent` / `SetDoubleIfPresent` / `SetIntIfPresent` / `SetStringIfPresent` 风格）：

```cpp
  SetBoolIfPresent(parameter_dictionary, "enable_homotopy_class_planning",
                   [&](bool v) { options.set_enable_homotopy_class_planning(v); });
  SetBoolIfPresent(parameter_dictionary, "enable_multithreading",
                   [&](bool v) { options.set_enable_multithreading(v); });
  SetIntIfPresent(parameter_dictionary, "max_number_classes",
                  [&](int v) { options.set_max_number_classes(v); });
  SetBoolIfPresent(parameter_dictionary, "oscillation_recovery",
                   [&](bool v) { options.set_oscillation_recovery(v); });
  SetDoubleIfPresent(parameter_dictionary, "oscillation_filter_duration",
                     [&](double v) { options.set_oscillation_filter_duration(v); });
  SetStringIfPresent(parameter_dictionary, "obstacle_conversion_mode",
                     [&](const std::string& v) {
                       options.set_obstacle_conversion_mode(v);
                     });
```

- [ ] **Step 3: `applyOptionsToConfig` 覆盖 TebConfig**

```cpp
  // Homotopy
  if (options_->has_enable_homotopy_class_planning() ||
      options_->enable_homotopy_class_planning() || true) {
    // proto3 bool 默认 false：用「字段是否在 lua 出现」困难时，约定：
    // 显式从 options 写入；Lua 未写时保持 TebConfig 构造默认（true）。
  }
  // 推荐实现：仅当 Lua 提供了键才覆盖。若当前 LoadOptions 无法区分「未设置」，
  // 则在 LoadOptions 里对缺省键写入 ROS2 默认：
  //   enable_homotopy_class_planning=true, enable_multithreading=true, ...
```

具体约定（实现时必须遵守）：

1. `LoadOptions` 在解析结束后，若 `obstacle_conversion_mode` 为空，设为 `"points_lines_polygons"`。
2. 对 bool：在 `teb_options.cpp` 增加「缺省填充」——构造临时 `TebConfig defaults;`，对未出现在 dictionary 的键用 `defaults.hcp.*` / `defaults.recovery.*` 写入 options（或在 Optimizer 中：仅当 dictionary 有键才 `set`；否则保留 `teb_config_` 构造默认）。
3. 最简可落地方案：`applyOptionsToConfig` 末尾：

```cpp
  auto& hcp = teb_config_.hcp;
  // Proto 显式值：若工程已 regenerate 且 lua 总会写全，可直接赋值。
  // 为对齐 ROS2 默认，当 options 未定制时保持 TebConfig() 默认。
  if (options_->max_number_classes() > 0) {
    hcp.max_number_classes = options_->max_number_classes();
  }
  // 下列 bool：由 teb_options 在 Load 时写入 ROS2 默认，保证非零初始化
  hcp.enable_homotopy_class_planning =
      options_->enable_homotopy_class_planning();
  hcp.enable_multithreading = options_->enable_multithreading();
  teb_config_.recovery.oscillation_recovery =
      options_->oscillation_recovery();
  if (options_->oscillation_filter_duration() > 0.0) {
    teb_config_.recovery.oscillation_filter_duration =
        options_->oscillation_filter_duration();
  }
```

并在 `LoadOptions` 开头或结尾设置 ROS2 对齐默认：

```cpp
  options.set_enable_homotopy_class_planning(true);
  options.set_enable_multithreading(true);
  options.set_oscillation_recovery(true);
  options.set_obstacle_conversion_mode("points_lines_polygons");
  // 然后再用 dictionary 覆盖
```

- [ ] **Step 4: 重新生成 proto 并编译**

```bash
docker exec SpaceHero bash -c 'cd /workspace/autonomy/build && cmake --build . --target autonomy -j$(nproc) 2>&1 | grep -E "teb_controller.pb|error:|Built target autonomy" | tail -30'
```

Expected: proto 重生成功，`Built target autonomy`。

- [ ] **Step 5: Commit**

```bash
git add autonomy/control/proto/teb_controller.proto \
        autonomy/control/controller/teb_controller/tools/teb_options.cpp \
        autonomy/control/controller/teb_controller/optimizer.cpp
git commit -m "$(cat <<'EOF'
feat(teb): expose homotopy and recovery options in proto

Align defaults with ROS2 TebConfig for homotopy multithreading and oscillation recovery.
EOF
)"
```

---

### Task 3: 移植 equivalence_relations + h_signature

**Files:**
- Create: `autonomy/control/controller/teb_controller/core/equivalence_relations.hpp`
- Create: `autonomy/control/controller/teb_controller/core/h_signature.hpp`
- Source: `include/teb_local_planner/equivalence_relations.h`, `h_signature.h`

- [ ] **Step 1: 复制并做机械替换**

从对照源复制内容后统一替换：

| 查找 | 替换 |
|------|------|
| `namespace teb_local_planner` | `namespace autonomy { namespace control { namespace controller { namespace teb_controller`（闭合同理） |
| `#include "teb_local_planner/` | `#include "autonomy/control/controller/teb_controller/core/` |
| `.h"` | `.hpp"`（本仓库头文件后缀） |
| `geometry_msgs::msg::PoseStamped` | `PoseStamped`（已由 `teb_core.hpp` 提供） |
| `boost::optional` | `std::optional` |
| `boost::none` | `std::nullopt` |
| `boost::shared_ptr` | `std::shared_ptr` |

删除/改写仅服务 ROS msg 且本仓库不用的重载；保留 `VertexPose*`、图顶点、`PoseSE2` 相关重载。

- [ ] **Step 2: 确认 `h_signature.hpp` 能被单独 include 编译**

临时在任意已编译 `.cpp` 加 `#include ".../h_signature.hpp"` 或直接进入 Task 4 一起编。

- [ ] **Step 3: Commit**

```bash
git add autonomy/control/controller/teb_controller/core/equivalence_relations.hpp \
        autonomy/control/controller/teb_controller/core/h_signature.hpp
git commit -m "$(cat <<'EOF'
feat(teb): port H-signature and equivalence relations from ROS2 TEB

EOF
)"
```

---

### Task 4: 移植 graph_search

**Files:**
- Create: `autonomy/control/controller/teb_controller/core/graph_search.hpp`
- Create: `autonomy/control/controller/teb_controller/core/graph_search.cpp`
- Source: `include/teb_local_planner/graph_search.h`, `src/graph_search.cpp`

- [ ] **Step 1: 复制并替换（同 Task 3 表）**

额外注意：

1. **保留** Boost.Graph 头与 `BOOST_NO_CXX11_DEFAULTED_FUNCTIONS` 变通（与上游一致）。
2. `#include <geometry_msgs/msg/twist.hpp>` → `"teb_core.hpp"`，`geometry_msgs::msg::Twist` → `Twist`。
3. 前向声明 `class HomotopyClassPlanner;` 放在本命名空间。
4. `#include <boost/random.hpp>` 保留（PRM 采样）。

- [ ] **Step 2: 编译**

```bash
docker exec SpaceHero bash -c 'cd /workspace/autonomy/build && cmake --build . --target autonomy -j$(nproc) 2>&1 | grep -E "graph_search|error:|Built target autonomy" | tail -40'
```

Expected: 无 `error:`（若 HomotopyClassPlanner 尚未移植，可先让 `graph_search.cpp` 只编通到缺失符号；若链接依赖 HCP，则本 Task 与 Task 5 同一提交亦可）。

- [ ] **Step 3: Commit**（若与 Task 5 拆分）

```bash
git add autonomy/control/controller/teb_controller/core/graph_search.hpp \
        autonomy/control/controller/teb_controller/core/graph_search.cpp
git commit -m "$(cat <<'EOF'
feat(teb): port homotopy graph search (Boost.Graph)

EOF
)"
```

---

### Task 5: 移植 HomotopyClassPlanner

**Files:**
- Create: `autonomy/control/controller/teb_controller/core/homotopy_class_planner.hpp`
- Create: `autonomy/control/controller/teb_controller/core/homotopy_class_planner_impl.hpp`
- Create: `autonomy/control/controller/teb_controller/core/homotopy_class_planner.cpp`
- Source: `homotopy_class_planner.h` / `.hpp` / `src/homotopy_class_planner.cpp`

- [ ] **Step 1: 复制三文件并机械替换（Task 3 表）**

额外适配清单（必须全部处理）：

1. 去掉所有 `rclcpp::`、`RCLCPP_*`、`node` 构造参数；构造函数改为与 `TebOptimalPlanner` 一致：

```cpp
HomotopyClassPlanner();
HomotopyClassPlanner(const TebConfig& cfg, ObstContainer* obstacles = nullptr,
                     TebVisualizationPtr visual = TebVisualizationPtr(),
                     const ViaPointContainer* via_points = nullptr);
void initialize(const TebConfig& cfg, ObstContainer* obstacles = nullptr,
                TebVisualizationPtr visual = TebVisualizationPtr(),
                const ViaPointContainer* via_points = nullptr);
```

2. `plan` / `getVelocityCommand` / `clearPlanner` / `setPreferredTurningDir` / `hasDiverged` / `isTrajectoryFeasible` 签名对齐本仓库 `PlannerInterface`（`PoseStamped`、`Twist`、`CostmapFeasibilityModel*`）。
3. `boost::optional` → `std::optional`；线程用 `std::thread` / `std::mutex`（上游若已是 std 则保持）。
4. 可视化调用改为现有 `TebVisualization` stub，禁止依赖 `visualization_msgs` publisher。
5. Include 路径全部改为 `autonomy/control/controller/teb_controller/core/...`。

- [ ] **Step 2: 编译修复直至通过**

```bash
docker exec SpaceHero bash -c 'cd /workspace/autonomy/build && cmake --build . --target autonomy -j$(nproc) 2>&1 | grep -E "homotopy|error:|Built target autonomy" | tail -60'
```

Expected: `Built target autonomy`。

- [ ] **Step 3: Commit**

```bash
git add autonomy/control/controller/teb_controller/core/homotopy_class_planner.hpp \
        autonomy/control/controller/teb_controller/core/homotopy_class_planner_impl.hpp \
        autonomy/control/controller/teb_controller/core/homotopy_class_planner.cpp \
        autonomy/control/controller/teb_controller/core/graph_search.hpp \
        autonomy/control/controller/teb_controller/core/graph_search.cpp
git commit -m "$(cat <<'EOF'
feat(teb): port HomotopyClassPlanner from ROS2 TEB

Enable multi-topology planning without rclcpp, keeping Boost.Graph search.
EOF
)"
```

---

### Task 6: Optimizer 选型 + FailureDetector 接线

**Files:**
- Modify: `autonomy/control/controller/teb_controller/optimizer.hpp`
- Modify: `autonomy/control/controller/teb_controller/optimizer.cpp`
- Reference: ROS2 `teb_local_planner_ros.cpp` 中 planner 创建与 oscillation 分支（约 L101–110、L1000–1032）

- [ ] **Step 1: 改成员类型**

```cpp
// optimizer.hpp
#include "autonomy/control/controller/teb_controller/core/homotopy_class_planner.hpp"
#include "autonomy/control/controller/teb_controller/core/planner_interface.hpp"
#include "autonomy/control/controller/teb_controller/core/recovery_behaviors.hpp"

// ...
  std::unique_ptr<PlannerInterface> planner_;
  FailureDetector failure_detector_;
  RotType last_preferred_rotdir_{RotType::none};
  double controller_frequency_{5.0};
  // 可选：记录振荡时间的 steady_clock（若需 min_duration）
  std::chrono::steady_clock::time_point time_last_oscillation_{};
```

- [ ] **Step 2: `initialize` 按配置创建 planner**

```cpp
  obstacle_converter_ =
      std::make_unique<tools::CostmapObstacleConverter>(*options_);

  if (teb_config_.hcp.enable_homotopy_class_planning) {
    planner_ = std::make_unique<HomotopyClassPlanner>(
        teb_config_, &obstacles_, TebVisualizationPtr(), &via_points_);
    AINFO << "TEB parallel planning in distinctive topologies enabled.";
  } else {
    planner_ = std::make_unique<TebOptimalPlanner>(
        teb_config_, &obstacles_, TebVisualizationPtr(), &via_points_);
    AINFO << "TEB parallel planning in distinctive topologies disabled.";
  }

  failure_detector_.setBufferLength(static_cast<std::size_t>(std::round(
      teb_config_.recovery.oscillation_filter_duration * controller_frequency_)));
  has_plan_ = false;
```

`initialize` 签名已有 `controller_frequency`：存入 `controller_frequency_`。

- [ ] **Step 3: `evalControl` 末尾加振荡恢复**

在成功得到 `vx,vy,omega` 并填充 `TwistStamped cmd` 之后：

```cpp
  Twist last_cmd = ToTebTwist(cmd.twist);  // 或直接从 vx/omega 构造
  if (teb_config_.recovery.oscillation_recovery) {
    failure_detector_.update(
        last_cmd, teb_config_.robot.max_vel_x,
        teb_config_.robot.max_vel_x_backwards, teb_config_.robot.max_vel_theta,
        teb_config_.recovery.oscillation_v_eps,
        teb_config_.recovery.oscillation_omega_eps);
    if (failure_detector_.isOscillating()) {
      time_last_oscillation_ = std::chrono::steady_clock::now();
      if (last_preferred_rotdir_ == RotType::none) {
        last_preferred_rotdir_ =
            (last_cmd.angular.z >= 0.0) ? RotType::left : RotType::right;
      }
      planner_->setPreferredTurningDir(last_preferred_rotdir_);
    } else {
      const auto elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        time_last_oscillation_)
              .count();
      if (elapsed > teb_config_.recovery.oscillation_recovery_min_duration) {
        last_preferred_rotdir_ = RotType::none;
        planner_->setPreferredTurningDir(RotType::none);
      } else if (last_preferred_rotdir_ != RotType::none) {
        planner_->setPreferredTurningDir(last_preferred_rotdir_);
      }
    }
  }
```

（细节以 ROS2 L1000–1032 为准微调符号与 recently_oscillated 逻辑。）

- [ ] **Step 4: 编译**

```bash
docker exec SpaceHero bash -c 'cd /workspace/autonomy/build && cmake --build . --target autonomy -j$(nproc) 2>&1 | grep -E "optimizer|error:|Built target autonomy" | tail -40'
```

Expected: `Built target autonomy`。

- [ ] **Step 5: Commit**

```bash
git add autonomy/control/controller/teb_controller/optimizer.hpp \
        autonomy/control/controller/teb_controller/optimizer.cpp
git commit -m "$(cat <<'EOF'
feat(teb): wire HomotopyClassPlanner and FailureDetector into Optimizer

Select planner from config and apply oscillation prefer-rotdir recovery.
EOF
)"
```

---

### Task 7: CostmapObstacleConverter 点/线/多边形

**Files:**
- Modify: `autonomy/control/controller/teb_controller/tools/costmap_obstacle_converter.hpp`
- Modify: `autonomy/control/controller/teb_controller/tools/costmap_obstacle_converter.cpp`

- [ ] **Step 1: 模式分支**

在 `update()` 开头：

```cpp
  const std::string mode = options_.obstacle_conversion_mode().empty()
                               ? "points_lines_polygons"
                               : options_.obstacle_conversion_mode();
  if (mode == "points_only") {
    // 保留现有 lethal 栅格点采样实现（抽成 UpdateAsPoints(...)）
    UpdateAsPoints(costmap, robot_pose);
    return;
  }
  UpdateAsClusters(costmap, robot_pose);
```

- [ ] **Step 2: 聚类转换算法（最小可用版）**

`UpdateAsClusters` 步骤：

1. 按 `sample_resolution` 下采样扫描 lethal 细胞，收集世界坐标点，过滤 `behind_robot_dist`。
2. 网格连通域（4-邻接）聚类。
3. 对每个簇：
   - `cells <= 2` → `PointObstacle`（质心）
   - 否则计算 PCA：若最大特征值远大于次大（细长，如 `λ1/λ2 > 4`）→ `LineObstacle`（沿主轴两端点）
   - 否则提取轴对齐或凸包顶点（可用 `common::math::Polygon2d` 若方便）→ `PolygonObstacle`（≥3 点）；失败则退回多 `PointObstacle`
4. `storage_` / `obstacles_` 与现逻辑相同（shared_ptr 入容器）。

参考现有类型：`PointObstacle`、`LineObstacle`、`PolygonObstacle`（`obstacles.hpp`）。

- [ ] **Step 3: 编译**

```bash
docker exec SpaceHero bash -c 'cd /workspace/autonomy/build && cmake --build . --target autonomy -j$(nproc) 2>&1 | grep -E "costmap_obstacle|error:|Built target autonomy" | tail -30'
```

Expected: `Built target autonomy`。

- [ ] **Step 4: Commit**

```bash
git add autonomy/control/controller/teb_controller/tools/costmap_obstacle_converter.hpp \
        autonomy/control/controller/teb_controller/tools/costmap_obstacle_converter.cpp
git commit -m "$(cat <<'EOF'
feat(teb): convert costmap clusters to point/line/polygon obstacles

Keep points_only mode for behavioral rollback.
EOF
)"
```

---

### Task 8: 联调验收

**Files:** 无强制新文件；可更新 `config/` 下 TEB lua（若存在）示例键。

- [ ] **Step 1: Homotopy 关闭冒烟**

Lua / options：`enable_homotopy_class_planning=false`  
运行控制栈或单元路径：应走 `TebOptimalPlanner`，日志含 `topologies disabled`。

- [ ] **Step 2: Homotopy 开启冒烟**

`enable_homotopy_class_planning=true`，`enable_multithreading=true/false` 各编一次/跑一次：日志 `topologies enabled`，`evalControl` 不抛 `NoValidControl`（有合法 plan 时）。

- [ ] **Step 3: FailureDetector 单测**

```bash
docker exec SpaceHero bash -c 'cd /workspace/autonomy/build && ctest -R recovery_behaviors -V 2>&1 | tail -40'
# 或直接跑测试二进制
```

Expected: PASS。

- [ ] **Step 4: 全量库编译**

```bash
docker exec SpaceHero bash -c 'cd /workspace/autonomy/build && cmake --build . --target autonomy -j$(nproc) 2>&1 | grep -E "error:|Built target autonomy" | tail -20'
```

Expected: `Built target autonomy`。

- [ ] **Step 5: 对照 spec §8 勾选并 Commit 配置示例（若有）**

```bash
git add config/  # 仅当确有 TEB lua 变更
git commit -m "$(cat <<'EOF'
chore(teb): document homotopy options for ROS2 parity

EOF
)"
```

---

## Spec coverage

| Spec 项 | Task |
|---------|------|
| FailureDetector / Recovery | 1, 6, 8 |
| Proto 开关与默认 | 2 |
| H-signature / equivalence | 3 |
| Graph search + Boost.Graph | 4 |
| HomotopyClassPlanner + 多线程可配 | 5, 6 |
| Optimizer 选型接线 | 6 |
| 障碍物点/线/多边形 + points_only | 7 |
| 验收清单 | 8 |

## 风险备忘（实现时）

- Homotopy 移植体量大：优先保证编译与 `PlannerInterface` 契约，再修运行时边界情况。
- `proto3` bool 默认 false：必须靠 `LoadOptions` 预置 ROS2 默认，避免误关 Homotopy。
- Boost.Graph 变通宏原样保留，勿「清理」导致编译失败。
