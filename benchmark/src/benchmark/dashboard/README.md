# Flask 网页监控面板 + BicMap（本地源码）

工作流：**场景 → 地图 → 模型 → 任务 → 录制 → 监控**。

地图引擎使用仓库内 **BicMap 源码** 构建，运行时 **不上网**。

## 源码与产物

```
third_party/bic-map-src/          # BicMap 库源码（src/bicMap）
scripts/build_bicmap.sh           # 本地 npm build:cdn
src/benchmark/dashboard/static/vendor/bic-map/bicMap.ext.min.js  # 产物
```

```bash
bash scripts/build_bicmap.sh      # 从源码重新构建 vendor
pip install -e ".[dashboard]"
benchmark dashboard --port 8090
```

页面加载 `/static/vendor/bic-map/bicMap.ext.min.js`，地图使用离线 blank style（无瓦片 / 无 glyph CDN / 无 Google Fonts）。

## 任务场景

导航 / 回充 / 探索建图 / 巡检 / 遥操 → Marker + 路径折线（仿真米制坐标换算）。

## 能力摘要

- 场景区可选仿真器 / 行人模型（写入 job overrides）
- 运行预设（仅导航 / 五任务 / 导航+录制）与本机勾选保存
- 运行中禁用「开始」、toast、SSE 断线自动重连
- 监控：结果卡片、曲线悬停读数 / 图例切换、下载 summary/curves、历史回看
- 任务级进度条；快捷键 `R` / `Esc`；移动端底部运行栏
