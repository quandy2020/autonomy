# Fields2Cover 构建与使用

[返回目录](README.md) · [参数](fields2cover_parameters.md) · [测试](fields2cover_testing.md)

## 1. 启用覆盖规划库

工作区位置：宿主机 `/home/quandy/workspace/github/autonomy`，在 `SpaceHero` 容器挂载为 `/workspace/autonomy`。源码根是 `src/autonomy`，本目录是 `src/autonomy/autonomy/planning/docs`。

```bash
ssh quandy@192.168.66.124
docker exec -it SpaceHero /bin/bash
cd /workspace/autonomy
source /opt/ros/humble/setup.bash

colcon build --packages-up-to autonomy --cmake-args \
  -DAUTONOMY_BUILD_PLANNING=ON \
  -DAUTONOMY_BUILD_FIELDS2COVER=ON \
  -DAUTONOMY_BUILD_FIELDS2COVER_TESTS=ON

ctest --test-dir build/autonomy \
  -R '^planning\.fields2cover_test$' --output-on-failure
```

`AUTONOMY_BUILD_FIELDS2COVER` 默认关闭。Fields2Cover 专项测试由 `AUTONOMY_BUILD_FIELDS2COVER_TESTS` 显式开启，当前可以独立于全仓 `BUILD_TEST` 运行。修改全仓测试开关会影响其他模块，不是运行该专项的必要步骤。

已有同仓 CMake 目标可以使用：

```cmake
target_link_libraries(my_coverage_application PRIVATE autonomy::planning)
```

本目录的独立示例直接连接当前工作区的规划共享库，不依赖全仓安装导出包。它仅构建示例，不重新配置生产规划库。

## 2. 运行完整示例

[coverage_demo.cc](examples/coverage_demo.cc) 包含可运行的顶层与分阶段调用。默认使用 80 × 60 m 区域及一个孔洞，机器人宽 1.5 m、作业宽 3 m、地头 3 m、转弯半径 1 m。

```bash
cd /workspace/autonomy
cmake -S src/autonomy/autonomy/planning/docs/examples \
  -B artifacts/fields2cover-docs-example \
  -DAUTONOMY_SOURCE_ROOT=/workspace/autonomy/src/autonomy \
  -DAUTONOMY_BUILD_ROOT=/workspace/autonomy/build/autonomy
cmake --build artifacts/fields2cover-docs-example -j2

# 默认内置田地；先输出逐步流程，再复用顶层 Route 生成 Dubins/RS 路径。
artifacts/fields2cover-docs-example/coverage_demo \
  artifacts/fields2cover-docs-example/output

# 按 Parser 支持的 FeatureCollection / Polygon 格式导入另一个田地。
artifacts/fields2cover-docs-example/coverage_demo \
  artifacts/fields2cover-docs-example/from-json \
  src/autonomy/autonomy/planning/docs/examples/field.geojson
```

输出包括 `field.wkt`、`core.wkt`、`swaths.csv`、`dubins.path.txt`、`reeds_shepp.path.txt`、`top-level-dubins.path.txt` 和 `summary.txt`。示例把空核心、空条带、条带数不一致、覆盖不足和空路径转成非零退出码；它不替代完整独立几何验收。

2026-11-18 容器实测：内置 WKT 和附带 GeoJSON 均生成 22 条作业带，核心覆盖率 99.688474%；Dubins 长 1383.039280 m，Reeds–Shepp 长 1383.004531 m，后者含后退状态。附带的 [paths PNG](images/documentation-example.png) 由实际输出绘制，文本结果见 [expected-output-20261118.txt](examples/expected-output-20261118.txt)。示例还检查平移田地的 `Field` 参考点恢复。

绘制实际路径 PNG：

```bash
# 复用本工作区随机验证安装的隔离 Python 依赖。
export PYTHONPATH=/workspace/autonomy/artifacts/fields2cover-random-fix/python-deps
python3 src/autonomy/autonomy/planning/docs/examples/render_paths.py \
  artifacts/fields2cover-docs-example/output
```

绘图依赖 Matplotlib 和 Shapely。其他环境可以创建独立 Python 虚拟环境安装这两个依赖。脚本生成 `paths.png`，显示田地、孔洞、3 m 核心边界、作业直行与连接路径；制图不改变规划输出，也不判定全车体可执行性。

## 3. 顶层 API 最小调用

```cpp
#include "autonomy/planning/planner/fields2cover/fields2cover.hpp"

namespace f2c = autonomy::planning::planner::fields2cover;

f2c::F2CCell cell;
cell.ImportFromWkt("POLYGON ((0 0,80 0,80 60,0 60,0 0))");
f2c::F2CRobot robot(1.5, 3.0);
robot.SetMinTurningRadius(1.0);
robot.SetCruiseVel(1.0);
robot.SetTurnVel(0.5);

f2c::Options options;
options.hg_swaths = 2;  // 1.5 × 2 = 3 m 地头
options.pp_alg = f2c::PPAlg::DUBINS;
auto path = f2c::PlanCovPath(robot, cell, options);
```

在业务边界捕获 `std::exception` 并检查 `path.size()`，把失败转换为可追踪的任务状态。此片段展示 API；完整的检查、孔洞输入和文件输出在示例文件中。

如果只需要给定角条带：

```cpp
options.sg_alg = f2c::SGAlg::GIVEN_ANGLE;
options.sg_angle = 0.0;  // 沿 +X；参数单位是弧度
```

## 4. 分阶段规划与比较两种算法

```cpp
f2c::F2CCells field(cell);
f2c::hg::ConstHL headland;
auto core = headland.GenerateHeadlands(field, 3.0);
f2c::sg::BruteForce generator;
f2c::obj::NSwathModified objective;
auto swaths = generator.GenerateBestSwaths(
    objective, robot.GetCovWidth(), core);

auto rings = headland.GenerateHeadlandSwaths(field, robot.GetWidth(), 2);
f2c::rp::RoutePlannerBase route_planner;
auto route = route_planner.GenRoute(rings.at(1), swaths);
route.SetField(field);  // 保存含孔洞的原始田地，供连接校验

f2c::pp::DubinsCurves dubins;
f2c::pp::ReedsSheppCurves reeds_shepp;
auto forward_path = f2c::pp::PathPlanning::PlanPath(robot, route, dubins);
auto reversible_path = f2c::pp::PathPlanning::PlanPath(robot, route, reeds_shepp);
```

分阶段方式可以控制米制地头距离、角度步长、路线起终点和转弯离散化。输入地头距离变化时，还要明确连接参考环深度；示例的 `rings.at(1)` 对应 2.25 m 参考环，核心区是 3 m 内缩。

不要省略 `route.SetField(field)`：否则 `PathPlanning` 没有真实孔洞边界可用，只能用连接附近的人工走廊检查部分候选。也不要把 `PlanPath(robot, swaths, turn)` 的直接条带连接重载当作带田地的避障路径接口。

## 5. 覆盖率和状态读取

```cpp
f2c::F2CSwaths all_swaths;
for (const auto& group : route.GetVectorSwaths()) {
  all_swaths.Append(group);
}
double coverage_pct = 100.0 * f2c::obj::FieldCoverage().ComputeCost(core, all_swaths);

for (const auto& state : forward_path.GetStates()) {
  const auto end = state.AtEnd();
  // state.point / end: 段起止点
  // state.angle: 车体航向，state.dir: 前进或倒车
  // state.type: 作业/转弯/地头连接，state.velocity: 速度参数
}
```

对相同核心区、相同作业幅宽的条带评分，避免把地头外的空白计入核心漏扫。倒车段的航向是车体方向，运动速度符号需要由 `dir` 明确转换；用于通用导航路径的朝向与车辆执行轨迹应分别定义。

## 6. 导入、导出与坐标注意事项

- WKT：`ImportFromWkt()` / `ExportToWkt()`。可以明确表达多边形外环、孔洞和多个组件，示例采用此格式。
- JSON：`Parser::ImportCellJson()` 读取 `features[0].geometry.coordinates`；示例 [field.geojson](examples/field.geojson) 给出其实际支持的结构。`ImportJson()` 逐 feature 建立 `Field`，读取 `properties.Name`，没有自动解析任意 CRS 或完整 GeoJSON 类型。
- GML：`ImportFieldGml()` 读取当前实现指定的 `Field/geometry/Polygon/OuterBoundaryIs/LinearRing/coordinates` 结构，不是通用 GML 数据源。其 `fail_silently` 参数仅影响缺失 CRS 的判定；缺失文件或解析错误仍会抛异常。
- 路径文本：`SaveToFile()` 输出 `x y z angle velocity len dir type`，保存 CRS、机器人参数和任务元数据需另外处理。

当前 `NativeGeom::exportToJson()`、`exportToGML()`、`exportToKML()` 使用 WKT 文本兼容返回，不能把这几种函数的输出当作对应标准格式。需要真正 GeoJSON 时由应用显式转换几何；使用前核对 [native_geom.cpp](../planner/fields2cover/types/native_geom.cpp)。

`F2CField` 构造会将田地移到参考点附近。以局部坐标调用时，输出要恢复到原始米制平面可使用：

```cpp
f2c::F2CField local_field(field, "work-area");
auto local_path = f2c::PlanCovPath(robot, local_field, options, true);
local_path.MoveTo(local_field.GetRefPoint());
```

推荐在明确的米制投影坐标中规划和控制。经纬度输入必须先投影；`Field` 的 `local_crs=false` 会转回原 CRS，输出段长也随坐标单位改变。当前 CRS 后端支持范围见[数据流说明](fields2cover_dataflow.md)。

## 7. 接入实际任务

为覆盖任务定义区域和孔洞输入、机器人配置、作业模式与任务取消语义，再选择上面的 API。生成路径之后校验覆盖、车辆包络、障碍距离和底盘控制能力，将状态中的倒车和作业段语义保留给控制及工具执行层。当前仓库的导航服务没有覆盖区域协议，不能仅配置一个 Fields2Cover `planner_id` 完成这些适配。
