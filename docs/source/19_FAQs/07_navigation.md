# 7. 导航与 BT

### Q: GetPlan 失败 / 路径空？

检查起终点、地图是否加载。

### Q: planned path too short？

起终点重合或不可达。

### Q: 导航超时？

增大超时；查 controller / 里程计。

### Q: BT 插件 load 失败？

```bash
export AUTONOMY_BT_PLUGIN_PATH=build/lib
ls build/lib/*bt*
```

### Q: BT vs 直驱？

见 [17 · §6](../17_Tasks/06_execution_modes.md)。

### Q: TransformAvailable / no robot pose？

核对帧名；需有 odom/TF 发布。

### Q: 看 Tasks 还是 Navigator？

语义 → [17](../17_Tasks/00_guide.md)；BT → [16](../16_Navigator/00_guide.md)。
