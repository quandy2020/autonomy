# Teleop MPPI RGBD Assist Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `task/apps/teleop` 内实现「手柄意图 → 路径簇打分 → MPPI → `/cmd_vel`」，障碍仅来自 RGBD→局部 costmap。

**Architecture:** `TeleopMppiAssist` 编排 `RgbdObstacleFeeder` + `IntentPathSelector` + 进程内 `MPPIController`；`TeleopClient` 仍负责限速/watchdog/发布。路径簇 MVP 用程序生成扇形弧线集（CMU 打分语义），不强制移植 343 条 PLY。

**Tech Stack:** C++17、`Costmap2DWrapper`、`MPPIController`、Autolink Reader/Writer、BT teleop、gtest、Lua 配置。

**Spec:** `docs/superpowers/specs/2026-07-17-teleop-mppi-assist-design.md`

---

## File map

| 文件 | 职责 |
|------|------|
| `autonomy/task/apps/teleop/intent_path_selector.hpp/.cpp` | 生成/持有路径簇；按 joyDir + costmap 选路 |
| `autonomy/task/apps/teleop/intent_path_selector_test.cpp` | 选路单测（空旷 / 正前方障碍） |
| `autonomy/task/apps/teleop/rgbd_obstacle_feeder.hpp/.cpp` | Depth Image → PointCloud2 → `feedPointCloud2` |
| `autonomy/task/apps/teleop/teleop_mppi_assist.hpp/.cpp` | 周期：选路 → MPPI → twist；持有 costmap + MPPI |
| `autonomy/control/.../critic_manager.cpp` | 静态注册 CostCritic + PathFollowCritic（解堵 MPPI） |
| `autonomy/task/apps/teleop/teleop_client.hpp/.cpp` | 接入 assist：`SetAssist` / 发布前可选过滤 |
| `autonomy/task/apps/teleop/teleop.cpp` / `teleop.hpp` | 启动时构造 assist（若 `assist_enabled`） |
| `autonomy/task/apps/teleop/plugins/action/apply_teleop_velocity_action.cpp` | 发布前调用 `assist->Tick()`（若存在） |
| `config/task/teleop_assist.lua` | 配置 |
| `config/task/task_options.lua` | teleop 块引用 assist 配置（可选 include） |

**不做：** grid_map、ControllerServer 改造、激光、CMU terrain / pathFollower 整包移植。

---

### Task 1: IntentPathSelector + 单测

**Files:**
- Create: `autonomy/task/apps/teleop/intent_path_selector.hpp`
- Create: `autonomy/task/apps/teleop/intent_path_selector.cpp`
- Create: `autonomy/task/apps/teleop/intent_path_selector_test.cpp`

- [ ] **Step 1: 写失败单测**

```cpp
// autonomy/task/apps/teleop/intent_path_selector_test.cpp
#include "autonomy/task/apps/teleop/intent_path_selector.hpp"

#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "gtest/gtest.h"

namespace autonomy::task::teleop {
namespace {

using map::costmap_2d::Costmap2D;
using map::costmap_2d::LETHAL_OBSTACLE;
using map::costmap_2d::FREE_SPACE;

std::shared_ptr<Costmap2D> MakeEmptyMap() {
  // 10m x 10m, 0.05m, origin at (-5,-5) so robot at (0,0) is map center
  auto map = std::make_shared<Costmap2D>(200, 200, 0.05, -5.0, -5.0, FREE_SPACE);
  return map;
}

TEST(IntentPathSelectorTest, PrefersForwardWhenOpen) {
  IntentPathSelector selector;
  selector.GenerateDefaultLibrary(/*num_dirs=*/9, /*num_lengths=*/3,
                                  /*max_range=*/3.0, /*ds=*/0.1);
  auto map = MakeEmptyMap();
  const double joy_dir_deg = 0.0;  // +x in base frame
  const double joy_speed = 0.5;
  auto path = selector.Select(*map, joy_dir_deg, joy_speed);
  ASSERT_TRUE(path.has_value());
  ASSERT_GE(path->poses.size(), 2u);
  // First segment should go roughly forward (+x)
  EXPECT_GT(path->poses[1].pose.position.x, path->poses[0].pose.position.x);
}

TEST(IntentPathSelectorTest, AvoidsLethalAhead) {
  IntentPathSelector selector;
  selector.GenerateDefaultLibrary(9, 3, 3.0, 0.1);
  auto map = MakeEmptyMap();
  // Wall ahead at x≈1.0, |y|<0.4
  for (double y = -0.4; y <= 0.4; y += 0.05) {
    unsigned int mx, my;
    ASSERT_TRUE(map->worldToMap(1.0, y, mx, my));
    map->setCost(mx, my, LETHAL_OBSTACLE);
  }
  auto path = selector.Select(*map, /*joy_dir_deg=*/0.0, /*joy_speed=*/0.5);
  ASSERT_TRUE(path.has_value());
  // Selected path should end with |y| larger than a straight path (steer aside)
  // or be shorter; at minimum must not contain lethal cells.
  for (const auto& ps : path->poses) {
    unsigned int mx, my;
    if (map->worldToMap(ps.pose.position.x, ps.pose.position.y, mx, my)) {
      EXPECT_LT(map->getCost(mx, my), LETHAL_OBSTACLE);
    }
  }
}

TEST(IntentPathSelectorTest, ZeroSpeedReturnsNullopt) {
  IntentPathSelector selector;
  selector.GenerateDefaultLibrary(5, 2, 2.0, 0.1);
  auto map = MakeEmptyMap();
  EXPECT_FALSE(selector.Select(*map, 0.0, 0.0).has_value());
}

}  // namespace
}  // namespace autonomy::task::teleop
```

- [ ] **Step 2: 跑测确认失败**

Run（在 Docker/build 目录）:

```bash
cmake --build . --target autonomy.task.apps.teleop.intent_path_selector_test -j$(nproc)
# 或全量后:
./bin/autonomy.task.apps.teleop.intent_path_selector_test
```

Expected: 链接/编译失败（类不存在）或测试未注册。

- [ ] **Step 3: 实现 IntentPathSelector**

```cpp
// intent_path_selector.hpp（要点）
#pragma once
#include <optional>
#include <vector>
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"

namespace autonomy::task::teleop {

struct PathCandidate {
  double end_dir_deg{0.0};  // 终点相对车头方位角
  commsgs::planning_msgs::Path path;
};

class IntentPathSelector {
 public:
  // 程序生成：方向扇区 × 长度档，每条为恒定曲率弧（含直行 κ=0）
  void GenerateDefaultLibrary(int num_dirs, int num_lengths,
                              double max_range, double ds);

  // joy_dir_deg: 期望行驶方向相对车头（度，CMU 风格）
  // 路径与 costmap 均在 robot/base 系（原点为机器人）
  std::optional<commsgs::planning_msgs::Path> Select(
      const map::costmap_2d::Costmap2D& costmap,
      double joy_dir_deg, double joy_speed) const;

  const std::vector<PathCandidate>& candidates() const { return candidates_; }

 private:
  static int CountLethalHits(const map::costmap_2d::Costmap2D& map,
                             const commsgs::planning_msgs::Path& path);
  static double WrapDeg(double deg);

  std::vector<PathCandidate> candidates_;
  int point_per_path_thre_{2};   // 对标 CMU pointPerPathThre
  double dir_weight_{0.02};
};

}  // namespace autonomy::task::teleop
```

打分（对标 CMU `clearPathList` / `dirDiff`）：

1. 路径上 lethal 点数 `< point_per_path_thre_` 视为可通行  
2. `score = 1 - sqrt(dirDiff/120)` 再加 `dir_weight` 微调（实现时用清晰公式并注释 CMU 出处）  
3. 在可通行候选中取最高分；若无候选返回 `nullopt`

`GenerateDefaultLibrary`：`dir ∈ [-90,90]` 均分，`length ∈ [min,max]` 均分，弧长积分生成 poses（`frame_id` 可先空，assist 再填）。

- [ ] **Step 4: 跑测通过**

```bash
./bin/autonomy.task.apps.teleop.intent_path_selector_test
```

Expected: 全部 PASS。

- [ ] **Step 5: Commit**

```bash
git add autonomy/task/apps/teleop/intent_path_selector.hpp \
        autonomy/task/apps/teleop/intent_path_selector.cpp \
        autonomy/task/apps/teleop/intent_path_selector_test.cpp
git commit -m "feat(teleop): add IntentPathSelector with costmap scoring"
```

---

### Task 2: CriticManager 静态加载（MPPI 解堵）

**Files:**
- Modify: `autonomy/control/controller/mppi_controller/critic_manager.cpp`
- Modify: `autonomy/control/controller/mppi_controller/critic_manager.hpp`（若需 `registerBuiltinCritics`）

- [ ] **Step 1: 在 `loadCritics()` 用名字工厂实例化**

对 `CostCritic`、`PathFollowCritic`（及配置里出现的已知 critic）`make_shared` → `configure(...)` → `push_back`。未知名字打 `AERROR` 并跳过。

```cpp
// critic_manager.cpp loadCritics() 伪代码
#include "autonomy/control/controller/mppi_controller/critics/cost_critic.hpp"
#include "autonomy/control/controller/mppi_controller/critics/path_follow_critic.hpp"
// ... PreferForwardCritic if listed

void CriticManager::loadCritics() {
  critics_.clear();
  for (const auto& name : critic_names_) {
    std::shared_ptr<critics::CriticFunction> critic;
    if (name == "CostCritic" || name == "cost_critic") {
      critic = std::make_shared<critics::CostCritic>();
    } else if (name == "PathFollowCritic" || name == "path_follow_critic") {
      critic = std::make_shared<critics::PathFollowCritic>();
    } else {
      AERROR << "Unknown MPPI critic: " << name;
      continue;
    }
    critic->configure(parent_, name_ + "." + name, name, costmap_ros_, options_);
    critics_.push_back(critic);
  }
  if (critics_.empty()) {
    AWARN << "No MPPI critics loaded; using CostCritic+PathFollowCritic defaults";
    // 默认装两个，保证 teleop assist 可用
  }
}
```

核对 `CriticFunction::configure` 实际签名后按现有头文件调用。

- [ ] **Step 2: 编译 libautonomy**

```bash
ninja autonomy
```

Expected: 成功。

- [ ] **Step 3: Commit**

```bash
git add autonomy/control/controller/mppi_controller/critic_manager.cpp \
        autonomy/control/controller/mppi_controller/critic_manager.hpp
git commit -m "fix(mppi): statically load Cost and PathFollow critics"
```

---

### Task 3: RgbdObstacleFeeder

**Files:**
- Create: `autonomy/task/apps/teleop/rgbd_obstacle_feeder.hpp`
- Create: `autonomy/task/apps/teleop/rgbd_obstacle_feeder.cpp`

- [ ] **Step 1: 实现接口**

```cpp
// rgbd_obstacle_feeder.hpp 要点
class RgbdObstacleFeeder {
 public:
  struct Options {
    std::string depth_topic{"/camera/depth/image_raw"};
    std::string camera_frame{"camera_depth_optical_frame"};
    double fx{525}, fy{525}, cx{320}, cy{240};  // 可后接 CameraInfo
    double min_depth{0.2}, max_depth{4.0};
    double min_height{-0.3}, max_height{0.5};   // 相对 base
    int stride{4};  // 降采样
    double stale_timeout_sec{0.5};
  };

  void Configure(std::shared_ptr<autolink::Node> node,
                 std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
                 const Options& opt);
  void Start();  // subscribe Image
  bool IsCloudFresh() const;
  void OnDepthImage(const commsgs::sensor_msgs::Image& img);
 private:
  commsgs::sensor_msgs::PointCloud2 Project(const Image&) const;
  // ...
};
```

`OnDepthImage`：按 `encoding`（优先 `16UC1` mm 或 `32FC1` m）反投影 → 高度/深度过滤 → `costmap_->feedPointCloud2(cloud)`；更新 `last_cloud_time_`。

- [ ] **Step 2: 编译**

```bash
ninja autonomy
```

- [ ] **Step 3: Commit**

```bash
git add autonomy/task/apps/teleop/rgbd_obstacle_feeder.hpp \
        autonomy/task/apps/teleop/rgbd_obstacle_feeder.cpp
git commit -m "feat(teleop): RGBD depth feeder into local costmap"
```

---

### Task 4: TeleopMppiAssist 编排

**Files:**
- Create: `autonomy/task/apps/teleop/teleop_mppi_assist.hpp`
- Create: `autonomy/task/apps/teleop/teleop_mppi_assist.cpp`

- [ ] **Step 1: 实现**

```cpp
class TeleopMppiAssist {
 public:
  struct Options {
    bool enabled{true};
    IntentPathSelector 相关参数…
    RgbdObstacleFeeder::Options rgbd;
    // costmap proto/lua 片段或内联默认：rolling 局部图 ~6x6m @ 0.05
    // mppi: 填 proto::ControllerOptions.mppi_controller_options
  };

  bool Configure(std::shared_ptr<autolink::Node> node,
                 std::shared_ptr<transform::Buffer> tf,
                 const Options& opt);

  // 输入手柄意图（已限幅），输出可发布 twist；失败时 zero / 仅角速度
  bool Tick(double joy_linear_x, double joy_angular_z,
            const commsgs::geometry_msgs::PoseStamped& robot_pose,
            const commsgs::geometry_msgs::Twist& robot_speed,
            commsgs::geometry_msgs::TwistStamped* cmd_out);

  bool enabled() const { return options_.enabled; }
  bool IsPerceptionOk() const;  // feeder fresh

 private:
  double AngularToJoyDirDeg(double angular_z, double linear_x) const;
  Options options_;
  std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_;
  RgbdObstacleFeeder feeder_;
  IntentPathSelector selector_;
  std::unique_ptr<control::controller::mppi_controller::MPPIController> mppi_;
};
```

`Tick` 逻辑：

1. `!enabled` → `cmd_out` = 输入透传，return true  
2. `!IsPerceptionOk()` → 零速，return false（符合 spec 默认停车）  
3. `|linear| < eps` → 零线速；可选保留小 `angular_z`  
4. `joy_dir = AngularToJoyDirDeg(...)`（例：`atan2(angular* L, linear)` 或简单 `k * angular` 映射到 ±90°，实现时固定一种并单测）  
5. `Select` → 无路径则零线速 + 小角速度  
6. `mppi_->SetPlan(*path)`；`ComputeVelocityCommands` → `cmd_out`

- [ ] **Step 2: 编译**

```bash
ninja autonomy
```

- [ ] **Step 3: Commit**

```bash
git add autonomy/task/apps/teleop/teleop_mppi_assist.hpp \
        autonomy/task/apps/teleop/teleop_mppi_assist.cpp
git commit -m "feat(teleop): TeleopMppiAssist orchestration loop"
```

---

### Task 5: 接入 TeleopClient / BT / TeleopTask

**Files:**
- Modify: `teleop_client.hpp/.cpp`
- Modify: `teleop.hpp/.cpp`
- Modify: `plugins/action/apply_teleop_velocity_action.cpp`

- [ ] **Step 1: TeleopClient 挂 assist**

```cpp
void SetAssist(std::shared_ptr<TeleopMppiAssist> assist);
// PublishVelocity():
//   if (assist_ && assist_->enabled()) {
//     TwistStamped out;
//     if (!assist_->Tick(linear_x_, angular_z_, pose, speed, &out)) {
//       PublishZeroVelocity(); return false; // 或仍发零速并 SUCCESS 视安全策略
//     }
//     applied_ = out; writer_->Write(out);
//   } else { 现有逻辑 }
```

位姿/里程计：MVP 从 `costmap_->getRobotPose` 或 `tf` lookup `base_link`；若不可用则 Tick 失败停车。

- [ ] **Step 2: TeleopTask::OnTreeInitialize**

读 `teleop_assist.lua`（路径由 task options / 常量），`assist_enabled` 为真则 `Configure` assist 并 `SetAssist`。

- [ ] **Step 3: ApplyTeleopVelocity 保持调用 `PublishVelocity()`**（assist 在 client 内）

无需改 BT XML，除非要暴露 blackboard 标志。

- [ ] **Step 4: 编译 + 冒烟**

```bash
ninja autonomy task
# 手动：assist_enabled=false 时行为与旧 teleop 一致
```

- [ ] **Step 5: Commit**

```bash
git add autonomy/task/apps/teleop/
git commit -m "feat(teleop): wire MPPI assist into TeleopClient and task"
```

---

### Task 6: 配置 Lua

**Files:**
- Create: `config/task/teleop_assist.lua`
- Modify: `config/task/task_options.lua`（teleop 段增加 `assist_config = "task/teleop_assist.lua"` 或等价字段；若 proto 暂无字段，则代码内默认相对路径 `config/task/teleop_assist.lua`）

- [ ] **Step 1: 写入默认配置**

```lua
-- config/task/teleop_assist.lua
return {
  assist_enabled = true,
  control_rate_hz = 20,
  stale_cloud_timeout_sec = 0.5,
  rgbd = {
    depth_topic = "/camera/depth/image_raw",
    camera_frame = "camera_depth_optical_frame",
    fx = 525.0, fy = 525.0, cx = 320.0, cy = 240.0,
    min_depth = 0.2, max_depth = 4.0,
    min_height = -0.3, max_height = 0.5,
    stride = 4,
  },
  path_library = {
    num_dirs = 9,
    num_lengths = 3,
    max_range = 3.0,
    ds = 0.1,
    point_per_path_thre = 2,
  },
  -- costmap / mppi：实现时用现有 LoadOptions 模式填充；可先硬编码安全默认再 Lua 化
}
```

- [ ] **Step 2: Commit**

```bash
git add config/task/teleop_assist.lua config/task/task_options.lua
git commit -m "config(teleop): add teleop_assist.lua defaults"
```

---

### Task 7: Spec 小修订 + README 一句（可选）

**Files:**
- Modify: `docs/superpowers/specs/2026-07-17-teleop-mppi-assist-design.md`  
  增加一句：MVP 路径簇为程序生成扇形弧；明确 **不用 grid_map**。

- [ ] **Step 1: 更新 spec 后 commit**

```bash
git add docs/superpowers/specs/2026-07-17-teleop-mppi-assist-design.md
git commit -m "docs: note procedural path library and no grid_map for teleop assist"
```

---

## Spec coverage check

| Spec 项 | Task |
|---------|------|
| 手柄意图 + MPPI 避障 | 4–5 |
| RGBD → costmap → MPPI | 3–4 |
| teleop 内闭环 | 4–5 |
| 路径簇打分 | 1 |
| assist_enabled 透传 | 5–6 |
| watchdog 不变 | 5（client 原逻辑） |
| 过期停车 / 无路径 | 4 |
| 非目标（无 ControllerServer/激光/grid） | 全文遵守 |
| Critic 可用 | 2 |

## Placeholder scan

无 TBD；路径文件 PLY 导入刻意不做，改程序生成（与 B 的打分语义一致，降低 MVP 体积）。

---
