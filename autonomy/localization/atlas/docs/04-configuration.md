# 04 配置

参考配置位于 `config/`：

- `euroc_mono.yaml`：EuRoC 单目。
- `euroc_mono_inertial.yaml`：EuRoC 单目惯性。
- `euroc_stereo_inertial.yaml`：EuRoC 双目惯性。
- `tum_vi_stereo_inertial.yaml`：TUM-VI 双鱼眼 + IMU。
- `tum_rgbd.yaml`：TUM RGB-D。
- `tartanair_rgbd_inertial.yaml`：TartanAir RGB-D 惯性。
- `orb_vocab.fbow`：二进制 FBow 词袋。

验证状态：EuRoC 单目惯性和双目惯性配置均已通过真实数据集验证；
TUM-VI 双鱼眼 + IMU 已在 room1 真实数据集验证；TartanAir RGB-D 惯性配置已在
`ArchVizTinyHouseDay/Data_easy` 的 P000、P003 完成数据集级验证，详细
fixed-scale SE(3) 指标见 `06-testing-and-benchmarking.md`。TartanAir
本身是合成数据集，此结果不等同于真实相机与 IMU 的标定或精度验证。

ConfigLoader 读取相机内参、畸变、图像尺寸、ORB 参数、双目基线、IMU
噪声以及系统加载/保存选项。配置中的相机模型必须与输入图像一致：
Pinhole 用于针孔模型，Kannala-Brandt 用于鱼眼模型。

配置调整原则：

1. 时间戳统一使用数据集或消息提供的原始时钟，图像与 IMU 必须处于同一
   时间基准。
2. 双目左右相机应提供一致的分辨率和正确的外参。
3. IMU 噪声和随机游走参数必须来自标定，不应通过轨迹结果反向调参。
4. ORB 特征数量和金字塔层数会直接影响 CPU、内存和弱纹理场景表现。
5. 修改配置后至少运行一条普通序列和一条困难序列。

安装时 YAML 与 FBow 文件会复制到
`share/autonomy/localization/conf/atlas`。

## 稠密地图、导航点云与占用栅格

地面机器人配置应显式给出相机/传感器离地高度，并把导航点云限制在地面
上下 `[-1.0, 1.0] m`：

```yaml
Atlas.Map.VoxelSize: 0.03
Atlas.Map.MaximumVoxels: 3000000
Atlas.Map.MaximumNavigationVoxels: 1500000
Atlas.Map.NavigationRadius: 30.0
Atlas.Map.NavigationRecenterDistance: 5.0
Atlas.Map.GridResolution: 0.10
Atlas.Map.SensorHeight: 1.65
Atlas.Map.EstimateGroundPlane: 1
Atlas.Map.MinHeight: 0.10
Atlas.Map.MaxHeight: 1.0
Atlas.Map.NavigationMinHeight: -1.0
Atlas.Map.NavigationMaxHeight: 1.0
```

未配置 `SensorHeight` 时，系统只提供保守的相对传感器高度回退：高于
`MaxHeight` 的点保持 unknown，不参与 free ray；低于 `MinHeight` 的点才可
作为 ground；导航点云仍按 `NavigationMinHeight..NavigationMaxHeight` 过滤。
该模式不能替代生产导航所需的地面相对高度配置，启动时会给出警告。

`NavigationRadius` 大于 0 时，导航点云使用以机器人近期位置为中心的滚动
球形窗口；机器人移动超过 `NavigationRecenterDistance` 后才重定中心并裁剪，
避免逐帧扫描全部 voxel。`NavigationRecenterDistance` 不得大于窗口半径。
全局稠密点云和占用栅格不受该窗口影响。`NavigationRadius: 0` 保持原有
全局导航点云行为。

## 长序列与回环重建

最近帧驻留内存，较老局部云和单目参考图像写入每个 MappingPipeline 实例
自己的磁盘目录。回环或全局 BA 后，系统从完整可用历史事务性重建稠密图、
导航点云和占用栅格；只有三张新地图全部成功才替换当前快照。

```yaml
Atlas.Map.MaximumCachedFrames: 400
Atlas.Map.MaximumHistoryCacheBytes: 8589934592
Atlas.Map.HistoryCacheDirectory: "/tmp"
```

`HistoryCacheDirectory` 必须是已存在且可写的父目录。字节上限设为 0 可禁用
磁盘冷历史；一旦内存历史不完整，系统会保留旧完整地图而不做破坏性部分重建。

## 导航代价膨胀

```yaml
Atlas.Map.RobotRadius: 0.30
Atlas.Map.InflationRadius: 0.60
Atlas.Map.InflationCostScaling: 5.0
```

膨胀只作用于发布快照中的已知 free 栅格，使用线性复杂度的欧氏距离变换。
occupied 保持 100，unknown 保持 -1，膨胀代价为 1..99；内部 log-odds
不会被修改。`InflationRadius` 必须不小于 `RobotRadius`，启用外圈时
`InflationCostScaling` 必须大于 0；三个参数都为 0 时关闭膨胀。
