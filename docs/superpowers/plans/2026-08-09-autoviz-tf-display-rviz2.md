# Autoviz TF Display RViz2 Parity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Autoviz Displays 面板中的 TF Display 在属性树、交互与 3D 可视化上对齐 RViz2 `TFDisplay`（完整方案，见设计规格）。

**Architecture:** `TfDisplay` 以 `transform::Buffer` 为权威帧源，内部维护 `FrameInfo`（enabled / parent / pose / age）；`/tf` Channel 仅灌入 Buffer。Displays 面板扩展 category / 动态 / 只读节点，对 Frames/Tree 做增量同步。渲染用 SceneOverlay 轴线 + 箭头近似 + `drawLabelsOgreOrGl` 帧名。

**Tech Stack:** C++17、Qt6、Autoviz SceneOverlay / transform::Buffer、`std::regex`、现有 `DisplayPropertyMap` 会话 YAML。

**Spec:** `docs/superpowers/specs/2026-08-09-autoviz-tf-display-rviz2-design.md`

---

## File map

| Path | Responsibility |
|------|----------------|
| Create: `autoviz/autoviz/display/tf_display_utils.hpp` | 纯函数：regex 过滤、timeout 颜色、默认轴长常量 |
| Create: `autoviz/autoviz/display/tf_display_utils.cpp` | 实现上述纯函数 |
| Create: `autoviz/tests/tf_display_utils_test.cpp` | gtest（`BUILD_AUTOVIZ_TESTS=ON`） |
| Modify: `autoviz/autoviz/display/tf_display.hpp` | `FrameInfo`、快照 API、override load/save |
| Modify: `autoviz/autoviz/display/tf_display.cpp` | buffer 驱动 update/draw |
| Modify: `autoviz/autoviz/common/display_property.hpp` | `DisplayPropertyKind` 扩展 + optional `parent_key` |
| Modify: `autoviz/autoviz/ui/display_tree_delegate.hpp` | 新 `DisplayTreeItemKind` |
| Modify: `autoviz/autoviz/ui/displays_panel.hpp/.cpp` | category / TF Frames·Tree 增量同步 |
| Modify: `autoviz/cmake/Tests.cmake` | 注册 `tf_display_utils_test`（若尚无通用 helper） |
| Modify: `autoviz/CMakeLists.txt` | 若需显式加入新 `.cpp`（通常 GLOB） |

---

### Task 1: Pure helpers + unit tests

**Files:**
- Create: `autoviz/autoviz/display/tf_display_utils.hpp`
- Create: `autoviz/autoviz/display/tf_display_utils.cpp`
- Create: `autoviz/tests/tf_display_utils_test.cpp`
- Modify: `autoviz/cmake/Tests.cmake`（注册测试）

- [ ] **Step 1: Add utils header/impl**

```cpp
// tf_display_utils.hpp
#pragma once
#include <regex>
#include <string>
#include <vector>
#include <QColor>

namespace autoviz::display {

constexpr float kTfDefaultAxisLength = 0.2f;

/** Empty whitelist → pass-all; empty blacklist → ban-none. Invalid regex → treat as empty + *error_out. */
std::vector<std::string> FilterTfFrameNames(
    const std::vector<std::string>& frames,
    const std::string& whitelist_regex,
    const std::string& blacklist_regex,
    std::string* error_out = nullptr);

/** age/timeout RViz thirds: return color + alpha multiplier in [0,1]. visible=false if age>timeout. */
struct TfAgeVisual {
  bool visible = true;
  QColor color = QColor(255, 255, 255);
  float alpha = 1.f;
};
TfAgeVisual TfAgeVisualForTimeout(double age_sec, double timeout_sec,
                                  const QColor& base_rgb);

}  // namespace autoviz::display
```

实现要点：
- `FilterTfFrameNames`：`std::regex_search`；构造失败时清空对应侧并写 `error_out`。
- `TfAgeVisualForTimeout`：`timeout<=0` 视为不过期；`age > timeout` → `visible=false`；`(t/3, 2t/3]` 灰色；`(2t/3, t]` 灰色且 alpha 线性降到 0。

- [ ] **Step 2: Add gtest**

```cpp
TEST(TfDisplayUtils, FilterWhitelistBlacklist) {
  const std::vector<std::string> in{"map", "base_link", "laser", "camera"};
  auto out = FilterTfFrameNames(in, "laser|camera", "", nullptr);
  ASSERT_EQ(out.size(), 2u);
  out = FilterTfFrameNames(in, "", "map", nullptr);
  EXPECT_EQ(out.size(), 3u);
}

TEST(TfDisplayUtils, AgeTimeoutSegments) {
  auto v = TfAgeVisualForTimeout(1.0, 15.0, QColor(220, 60, 60));
  EXPECT_TRUE(v.visible);
  EXPECT_FLOAT_EQ(v.alpha, 1.f);
  v = TfAgeVisualForTimeout(16.0, 15.0, QColor(220, 60, 60));
  EXPECT_FALSE(v.visible);
}
```

- [ ] **Step 3: Register in Tests.cmake + build/run**

```bash
cd autoviz/build && cmake -DBUILD_AUTOVIZ_TESTS=ON .. && cmake --build . --target tf_display_utils_test -j8
ctest -R tf_display_utils_test --output-on-failure
```

Expected: PASS

- [ ] **Step 4: Commit**（仅当用户要求 commit 时执行）

```bash
git add autoviz/autoviz/display/tf_display_utils.hpp autoviz/autoviz/display/tf_display_utils.cpp \
  autoviz/tests/tf_display_utils_test.cpp autoviz/cmake/Tests.cmake
git commit -m "$(cat <<'EOF'
feat(autoviz): add TF display filter/timeout helpers

EOF
)"
```

---

### Task 2: Rewrite `TfDisplay` core (buffer-driven draw)

**Files:**
- Modify: `autoviz/autoviz/display/tf_display.hpp`
- Modify: `autoviz/autoviz/display/tf_display.cpp`

- [ ] **Step 1: Replace header model**

在 `tf_display.hpp` 定义：

```cpp
struct TfFrameInfo {
  std::string name;
  std::string parent;
  bool enabled = true;
  QVector3D position;
  QQuaternion orientation;  // or store TransformStamped
  QVector3D rel_position;
  QQuaternion rel_orientation;
  int64_t last_update_ns = 0;
  bool have_fixed_pose = false;
};

struct TfFrameSnapshot {
  std::string name;
  std::string parent;
  bool enabled = true;
  QVector3D position;
  QVector3D orientation_xyzw;  // x,y,z,w for UI string
  QVector3D rel_position;
  QVector3D rel_orientation_xyzw;
};

class TfDisplay : public ChannelDisplay<...> {
 public:
  // ...
  std::vector<TfFrameSnapshot> frameSnapshots() const;
  std::vector<std::pair<std::string, std::string>> treeEdges() const; // child→parent
  void setAllFramesEnabled(bool enabled);
  void setFrameEnabled(const std::string& frame, bool enabled);
  bool allFramesEnabled() const { return all_enabled_; }

  void load(const common::Config& config) override;
  void save(common::Config config) const override;

 protected:
  void onUpdate() override;
  void processMessage(...) override;
  void onDraw(...) override;
  void onPropertyChanged(const std::string& key) override;

 private:
  void updateFrames();
  void drawFrame(rendering::SceneOverlay& scene, const TfFrameInfo& info,
                 float scale, bool show_axes, bool show_arrows, bool show_names);

  std::map<std::string, TfFrameInfo> frames_;
  std::map<std::string, bool> frame_enabled_from_config_;
  bool all_enabled_ = true;
  bool changing_single_frame_ = false;
  double update_timer_sec_ = 0.0;
  std::string filter_error_;
};
```

- [ ] **Step 2: Implement `propertySpecs`（对齐 RViz2，去掉 axis_length）**

```cpp
return {
  {"show_names", "Show Names", "false"},
  {"show_axes", "Show Axes", "true"},
  {"show_arrows", "Show Arrows", "true"},
  {"marker_scale", "Marker Scale", "1.0"},
  {"update_interval", "Update Interval", "0"},
  {"frame_timeout", "Frame Timeout", "15.0"},
  {"filter_whitelist", "Filter (whitelist)", ""},
  {"filter_blacklist", "Filter (blacklist)", ""},
};
```

- [ ] **Step 3: `processMessage` 只灌 Buffer + 请求 redraw**

保持 `ApplyTfMessageToBuffer`；**不要**再仅用 message 里的 child 作为唯一帧集。

- [ ] **Step 4: `updateFrames`（在 `onUpdate` 中按 interval 调用）**

伪逻辑：

```
names = buffer->frameStats() 的 frame_id
names = FilterTfFrameNames(names, whitelist, blacklist, &filter_error_)
for name in names:
  get-or-create FrameInfo
  apply frame_enabled_from_config_ / all_enabled_
  lookup fixed←name → position/orientation
  _getParent → parent; lookup parent←name → relative
  last_update_ns from TfFrameStats
delete frames_ not in filtered set
set Status Ok/Warn
```

`Update Interval`：用 wall clock delta 累加 `update_timer_sec_`；`< 1e-4` 或超时则 `updateFrames()`。

- [ ] **Step 5: `onDraw`**

对每个 `enabled && have_fixed_pose` 的帧：
- `TfAgeVisualForTimeout`；不可见则跳过
- Axes：三线 RGB，长度 `kTfDefaultAxisLength * marker_scale`，颜色乘 age
- Arrows：子原点 → 父原点；用 `addLine` + 末端两侧短线作箭头头（或复用其他 display 的箭头 helper 若有）
- Names：`drawLabelsOgreOrGl`，`char_height = 0.1f * marker_scale`

- [ ] **Step 6: Manual smoke**

```bash
cd autoviz && cmake --build build --target autoviz autoviz_cpp_05_transform_tree -j8
# Terminal A: ./build/bin/examples/autoviz_cpp_05_transform_tree
# Terminal B: ./build/bin/autoviz --config config/default.autoviz
```

Expected: TF 勾选后可见轴；开 Show Names / Arrows 有名称与父子箭头；停 05 后约 15s 帧消失。

- [ ] **Step 7: Commit**（用户要求时）

```bash
git commit -m "feat(autoviz): rewrite TfDisplay with buffer-driven RViz2 visuals"
```

---

### Task 3: Display tree kinds + property kind extensions

**Files:**
- Modify: `autoviz/autoviz/common/display_property.hpp`
- Modify: `autoviz/autoviz/ui/display_tree_delegate.hpp`
- Modify: `autoviz/autoviz/ui/display_tree_delegate.cpp`（只读/category 不进编辑器）

- [ ] **Step 1: Extend enums**

```cpp
// display_property.hpp
enum class DisplayPropertyKind {
  kAuto,
  kColor,
  kChannel,
  kPath,
  kCategory,   // 无值列编辑
  kReadOnly,
  kRegex,
};
```

```cpp
// display_tree_delegate.hpp — append to DisplayTreeItemKind
  kDisplayPropertyCategory,
  kDisplayFrameEnabled,   // Frames/<id> checkbox
  kDisplayFrameField,     // readonly child under frame
  kDisplayTreeNode,       // Tree hierarchy node
  kDisplayAllFramesEnabled,
```

- [ ] **Step 2: Delegate：category/readonly/`kDisplayFrameField`/`kDisplayTreeNode` 不创建 editor**

在 `createEditor` / `editorEvent` 开头按 kind 返回 `nullptr` / false。

- [ ] **Step 3: Build autoviz**

```bash
cmake --build autoviz/build --target autoviz -j8
```

Expected: success

---

### Task 4: DisplaysPanel — static TF categories + incremental Frames sync

**Files:**
- Modify: `autoviz/autoviz/ui/displays_panel.hpp`
- Modify: `autoviz/autoviz/ui/displays_panel.cpp`

- [ ] **Step 1: In `populateDisplayProperties`, after flat `propertySpecs()`, if `typeId()=="TF"` append structure**

```
Filter props already from specs
Frames (category)
  All Enabled (checkable value)
Tree (category, initially empty children)
```

用新 `DisplayTreeItemKind` + roles：`kDisplayTreeRolePropertyKey` 存 `frames.all_enabled` / `frames.<id>` / `frames.<id>.parent` 等。

- [ ] **Step 2: Add `syncTfDisplayProperties(QTreeWidgetItem* display_item, TfDisplay*)`**

- 从 `frameSnapshots()` diff Frames 子节点（除 All Enabled）：增删改 checkbox / 只读字段文本。
- 重建 Tree：清空 Tree 子节点，按 parent 挂接（无 parent → Tree 根下）。
- **禁止**调用全量 `refresh()`。
- 用 `updating_` 包裹，避免 `itemChanged` 回环。

- [ ] **Step 3: Call sync from `refreshStatus`**（每秒）且仅当该 display 为 TF 且未在 EditingState

```cpp
if (auto* tf = dynamic_cast<display::TfDisplay*>(display)) {
  syncTfDisplayProperties(display_item, tf);
}
```

需要在 walk 时保留 `display` 行 `QTreeWidgetItem*`（当前 walk 已有 display 节点）。

- [ ] **Step 4: Handle itemChanged for All Enabled / per-frame**

```cpp
if (kind == kDisplayAllFramesEnabled) {
  tf->setAllFramesEnabled(checked);
}
if (kind == kDisplayFrameEnabled) {
  tf->setFrameEnabled(frame_id, checked);
}
```

- [ ] **Step 5: Manual UI check**

跑 05 + autoviz：展开 TF → Frames 列出 map/base_link/laser/camera；取消 All Enabled 全隐；单开 laser 仅 laser；Tree 层级正确。

---

### Task 5: Persistence (All Enabled + per-frame map)

**Files:**
- Modify: `autoviz/autoviz/display/tf_display.cpp` (`load`/`save` / `saveToConfig`)

- [ ] **Step 1: Save**

在 `save` / `saveToConfig` 中写入 properties：

- `all_enabled`: `"true"`/`"false"`
- `frame_enabled.<frame_id>`: 对各帧（或仅非默认）

或嵌套 Config map `Frames`（若 `Display::save(Config)` 支持子 map——优先与 RViz 类似的 Config 子树；否则用扁平前缀键）。

推荐扁平前缀（改动最小）：

```
properties["all_enabled"] = "true"
properties["frame_enabled.laser"] = "false"
```

- [ ] **Step 2: Load**

`loadFromConfig` / `setProperties` 后解析前缀填入 `frame_enabled_from_config_`；`setAllFramesEnabled` 初值。

- [ ] **Step 3: Verify**

Save config → 重启 → laser 仍为禁用。

---

### Task 6: Filter wiring + Status copy

**Files:**
- Modify: `autoviz/autoviz/display/tf_display.cpp`
- Modify: `autoviz/autoviz/ui/displays_panel.cpp`（若 Filter 需特殊 editor；默认 LineEdit 即可）

- [ ] **Step 1: `onPropertyChanged` for filter_* → 立即 `updateFrames()`**

- [ ] **Step 2: Status messages**

```
Ok: "Showing N frames"
Warn: "No frames in TF buffer"
Warn: "No frames match filters"
Warn: "Invalid filter regex: ..."
Warn: "No transforms to Fixed Frame 'map'"  // 有帧但全部 lookup 失败
```

Channel 无消息但 Buffer 有帧 → **仍 Ok**（不报 No messages）。可能需要 override `ChannelDisplay::onUpdate` 尾部 status：在 `TfDisplay::onUpdate` 末尾根据 buffer 覆盖 `setStatus*`（在基类设置 Warn 之后调用）。

检查 `ChannelDisplay::onUpdate`：若 `!has_received_message_` 会 Warn。TF 应：

```cpp
// TfDisplay::onUpdate after ChannelDisplay path — easiest: override onUpdate fully
// or set has_received_message_ when buffer non-empty via protected hook.
```

**选定方案：** `TfDisplay` 不依赖消息计数；在 `updateFrames` 末尾无条件 `setStatusOk/Warn`，并在 `onUpdate` 中先调用 `ChannelDisplay` 的订阅/解析逻辑，再 `updateFrames()` **覆盖** status。

实现：复制 ChannelDisplay 的 queue drain 到 override，或给基类加 `virtual void updateStatusAfterParse()`；**最小改动**：在 `TfDisplay` 中 override `onUpdate`：

```cpp
void TfDisplay::onUpdate() override {
  ChannelDisplay::onUpdate();  // may set No messages
  updateFramesMaybe();         // overwrites status
}
```

- [ ] **Step 3: Manual filter test**

Whitelist `laser|camera` → Frames 仅二者；Status Ok。

---

### Task 7: Default config + docs polish

**Files:**
- Modify: `autoviz/config/default.autoviz`（TF 属性默认与 RViz2 对齐：Show Names false）
- Modify: `autoviz/docs/RVIZ_PARITY.md`（若存在 TF 条目，更新一句）

- [ ] **Step 1: Align default.autoviz TF properties**

```yaml
  - Type: TF
    Name: TF
    Channel: /tf
    Enabled: true
    Properties:
      show_names: "false"
      show_axes: "true"
      show_arrows: "true"
      marker_scale: "1.0"
      update_interval: "0"
      frame_timeout: "15.0"
```

- [ ] **Step 2: Full acceptance checklist（对照 spec §5）**

跑 05：Axes/Names/Arrows/Scale/Timeout/Interval/Filter/All Enabled/Tree/无卡顿。

- [ ] **Step 3: Final commit**（用户要求时）

```bash
git commit -m "feat(autoviz): TF display RViz2 property tree and behavior parity"
```

---

## Spec coverage checklist

| Spec 项 | Task |
|---------|------|
| Show Names/Axes/Arrows + Marker Scale | Task 2 |
| Frame Timeout 三段视觉 | Task 1 + 2 |
| Update Interval | Task 2 |
| Filter whitelist/blacklist | Task 1 + 6 |
| Frames + All Enabled + per-frame | Task 4 + 5 |
| Parent/Position/Orientation 只读 | Task 4 |
| Tree | Task 4 |
| Buffer 权威 + Channel 灌入 | Task 2 |
| Status 不误报 No messages | Task 6 |
| 增量 UI 不卡顿 | Task 4 |
| 持久化 | Task 5 |
| 去掉 Axis Length | Task 2 + 7 |

---

## Notes for executors

- 不要在 `refreshStatus` 里对 Displays 调 `refresh()`。
- `MessageQueue` / 属性同步必须在 UI 线程改 `QTreeWidgetItem`。
- `_getParent` 在 `tf2::BufferCore` 上为 public（带下划线 API）；经 `transform::Buffer` 调用。
- Commit 步骤默认跳过，除非用户明确要求 git commit。
