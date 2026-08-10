# Autoviz TF Display 对齐 RViz2 设计

- **日期**：2026-08-09
- **状态**：已批准（待实现计划）
- **范围**：Displays 面板中 TF Display 的属性树、交互与 3D 可视化，对齐 ROS 2 RViz2 `rviz_default_plugins::displays::TFDisplay`
- **不做**：帧 SelectionHandler / 点击选中高亮；Ogre 专用 Axes/Arrow 实体（用 SceneOverlay 近似）

## 1. 背景与问题

当前 Autoviz `TfDisplay`：

- 属性表有 Show Names / Axes / Arrows / Marker Scale / Frame Timeout / Update Interval / Axis Length，但 `onDraw` 仅绘制轴，且多数属性未生效。
- 缺少 RViz2 的 **Frames**（All Enabled + 逐帧开关 + 只读位姿）、**Tree**、**Filter whitelist/blacklist**。
- Status 依赖 Channel 消息，易误报 `No messages received`（RViz2 TF 以 TF buffer 全量帧为准）。

用户要求：**属性与操作与 RViz2 一致**（完整对齐）。

## 2. 目标属性树

```
TF [✓]
├── Status
├── Channel: /tf              # Autoviz 保留：将 /tf 灌入 tf_buffer
├── Show Names
├── Show Axes
├── Show Arrows
├── Marker Scale
├── Update Interval
├── Frame Timeout
├── Filter (whitelist)        # regex；空 = 不过滤
├── Filter (blacklist)        # regex；空 = 不过滤
├── Frames
│   ├── All Enabled [✓]
│   └── <frame_id> [✓]
│       ├── Parent            # 只读
│       ├── Position          # Fixed Frame 下，只读
│       ├── Orientation       # Fixed Frame 下，只读
│       ├── Relative Position # 相对 Parent，只读
│       └── Relative Orientation
└── Tree                      # 父子层级，只读
    └── …
```

**移除**独立属性 `Axis Length`（并入 RViz2 语义：默认轴长 0.2 m × Marker Scale）。旧配置中的 `axis_length` 可忽略或在加载时映射为 scale 提示（非必须）。

默认值对齐 RViz2 Rolling/`tf_display.cpp`：

| 属性 | 默认 |
|------|------|
| Show Names | false |
| Show Axes | true |
| Show Arrows | true |
| Marker Scale | 1.0 |
| Update Interval | 0（每帧） |
| Frame Timeout | 15.0 s |
| Filter whitelist/blacklist | 空 |
| All Enabled | true |

## 3. 行为规范

### 3.1 数据源

1. **权威源**：`context_->tf_buffer`（`frameStats()` / `_getParent` / `lookupTransform`）。
2. **Channel `/tf`**：继续 `ApplyTfMessageToBuffer`；其他路径（如 RobotModel、外部 `/tf_static`）写入的帧同样出现在 Frames/Tree。
3. Status：
   - 至少一帧可变换到 Fixed Frame → Ok（可附带帧数量）。
   - Buffer 空或全部超时/不可达 → Warn，文案说明原因（不用单纯 “No messages received”）。

### 3.2 可视化

| 开关 | 行为 |
|------|------|
| Show Axes | 每启用帧画 RGB 轴；长度 `0.2 * Marker Scale` |
| Show Arrows | 子→父箭头（SceneOverlay：轴杆 + 箭头头近似）；颜色对齐 RViz FrameInfo 默认 |
| Show Names | 帧原点附近绘制帧名（现有 text raster / textured billboard） |
| Frame Timeout | 年龄 `age`：`age > timeout` 隐藏；`age ∈ (timeout/3, 2*timeout/3]` 变灰；`age ∈ (2*timeout/3, timeout]` 淡出透明度 |
| Update Interval | `interval==0` 或经过间隔后重算帧列表与姿态；否则复用上次结果绘制 |

### 3.3 过滤与启用

1. 从 buffer 取全部 frame id 后应用：
   - whitelist 非空：保留 `regex_search` 命中者；
   - blacklist 非空：排除命中者。
2. **All Enabled**：写入时批量设置各帧 Enabled；单帧改动时若与 All 不一致，不反向强制 All（与 RViz `changing_single_frame_enabled_state_` 同思路）。
3. 未启用帧：不画 name/axes/arrow，Frames 下复选框为未勾选。

### 3.4 Frames / Tree UI

- **Frames**：动态子节点；每帧勾选 = Enabled；展开后只读 Parent / Position / Orientation / Relative*。
- **Tree**：按 parent 关系挂树；无 parent 的帧挂在 Tree 根下；只读、不可拖拽改父子。
- **增量刷新**：`DisplaysPanel` 对 TF 的 Frames/Tree 做 diff 更新（增删改文本/勾选），禁止每秒 `refresh()` 整树重建。

### 3.5 会话持久化

- 保存扁平属性（含两个 Filter 字符串）。
- 保存 `frames.all_enabled` 与各帧 `enabled` 映射（键为 frame id）。
- 加载时：先应用 All Enabled 默认，再用映射覆盖已知帧；未知帧稍后创建时应用映射。

## 4. 架构

```
┌─────────────────┐     property change      ┌──────────────────┐
│ DisplaysPanel   │ ───────────────────────► │ TfDisplay        │
│ (category/dyn)  │ ◄── frame list / poses ──│ FrameInfo map    │
└─────────────────┘     (light UI sync)      └────────┬─────────┘
                                                      │ lookup / parent
                                                      ▼
                                             ┌──────────────────┐
                                             │ transform::Buffer│
                                             └──────────────────┘
                                                      ▲
                         ChannelDisplay</tf> ─────────┘
```

### 4.1 组件

1. **`TfDisplay` / `FrameInfo`**（`autoviz/display/tf_display.*`）  
   - 维护 `std::map<std::string, FrameInfo>`。  
   - `onUpdate`：按 Update Interval 调用 `updateFrames()`。  
   - `onDraw`：按开关与超时绘制。  
   - 暴露只读快照 API 供面板同步（帧列表、parent、pose、enabled）。

2. **Displays 面板属性模型扩展**  
   - 扩展 `DisplayPropertySpec` 或并行 `PropertyNode`：支持 `kCategory`、`kBool`、`kReadOnly`、`kRegex`、动态子节点。  
   - TF 专用：`syncTfPropertyTree(QTreeWidgetItem*)`，由 `refreshStatus`/定时轻量调用，或 Display 回调 `request_property_sync`。  
   - 交互：All Enabled / 单帧 Enabled / Filter 文本 → `setPropertyValue` / TF 专用 setter。

3. **渲染辅助**  
   - 轴：现有 `addLine`。  
   - 箭头：小型 helper（方向向量 + 锥形三线或短线段头）。  
   - 名称：复用 `text_raster_utils` → textured billboard。

### 4.2 错误处理

- `lookupTransform` / `_getParent` 失败：该帧 Warn 状态文案（可选写入 Status 子项或仅跳过绘制）；不崩溃。
- 非法 regex：Filter 视为「不匹配任何」（或保持上次有效 regex），Status Warn 一次。

## 5. 测试与验收

手动（配合 `autoviz_cpp_05_transform_tree` 或 `04_sensor` 的 `/tf`）：

1. Show Axes/Names/Arrows 开关即时反映。  
2. Marker Scale 同时缩放轴与箭头。  
3. Frame Timeout：停发 TF 后帧变灰→消失。  
4. Update Interval：设大值时姿态更新变慢。  
5. Filter whitelist=`laser|camera` 仅显示匹配帧。  
6. All Enabled 关闭 → 全隐；单开 `base_link` → 仅该帧。  
7. Tree 显示 map→base_link→laser/camera。  
8. 属性操作不卡顿（Frames 增量更新，无整面板 rebuild）。

可选单测：regex 过滤纯函数；timeout 颜色分段；enabled 映射 load/save。

## 6. 实现分期（单计划内顺序）

1. `TfDisplay` 行为重写（buffer 驱动、超时、轴/箭头/名称、interval）。  
2. 属性模型 + DisplaysPanel category/动态/只读。  
3. Frames + All Enabled + 持久化。  
4. Tree + Filter。  
5. Status 文案与验收。

## 7. 参考

- RViz2：`rviz_default_plugins/.../displays/tf/tf_display.cpp`（Rolling）  
- 现有：`autoviz/display/tf_display.*`、`autoviz/transform/buffer.hpp`（`frameStats`）、`autoviz/ui/displays_panel.*`
