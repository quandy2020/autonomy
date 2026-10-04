# 框架对照（rviz_common）

| rviz_common | Autoviz | 状态 |
|-------------|---------|------|
| VisualizationFrame | `ui/frame*` | ✅ |
| VisualizationManager | `common/visualization_manager` | ✅ |
| Display / DisplayGroup | `display/` + Group | ✅ |
| DisplayContext | `common/display_context` | ✅ |
| FrameManager | `common/frame_manager` | ✅ |
| ViewManager | `common/view_manager` | ✅ |
| SelectionManager | `common/selection_manager` | ✅ |
| Property 树 | Display 属性 + Displays 面板 | ✅ |
| ToolManager | ToolManager + ToolRegistry | ✅ |
| ViewControllerRegistry | `view_controller_registry` | ✅ |
| Config + YAML | SessionConfig + YAML IO | ✅ |
| DisplayFactory | DisplayRegistry | ✅ |
| pluginlib | Registries + `AUTOVIZ_PLUGIN_PATH` | ✅ |
| GPU Pick | PickRegistry + FBO / Ogre pick | ✅ |

## 生命周期（Display）

```text
initialize → onInitialize
onEnable  → 订阅 channel
update    → 出队 processMessage
onDisable → 取消订阅
reset     → 清空场景对象
```

## 相关文档

- [模块](../architecture/modules.md) · [Display 清单](displays-panels.md)
