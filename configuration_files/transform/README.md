# 静态TF变换配置说明

## 概述

静态TF变换用于定义机器人各个传感器和坐标系之间的固定空间关系。该系统支持从YAML配置文件加载静态变换，并周期性地发布到TF树中。

## 配置文件格式

### YAML格式 (推荐)

配置文件位置: `configuration_files/transform/static_transform.yaml`

```yaml
static_transforms:
  # 变换名称
  - name: camera
    enabled: true                    # 是否启用
    frame_id: "base_link"           # 父坐标系
    child_frame_id: "camera_link"   # 子坐标系
    translation:                     # 平移向量 (米)
      x: 0.15
      y: 0.0
      z: 0.12
    rotation:                        # 旋转四元数 (x, y, z, w)
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0

settings:
  publish_rate: 10.0                              # 发布频率 (Hz)
  tf_prefix: ""                                   # TF前缀
  print_transforms_on_startup: true               # 启动时打印变换
  validate_quaternion: true                       # 验证四元数归一化
```

## 坐标系说明

### 标准坐标系

| 坐标系名称 | 说明 |
|-----------|------|
| `map` | 全局地图坐标系，固定参考系 |
| `odom` | 里程计坐标系，相对于起点的估计位置 |
| `base_link` | 机器人基座坐标系，通常位于机器人几何中心 |
| `base_footprint` | 机器人地面投影坐标系 |

### 传感器坐标系

| 传感器 | 坐标系名称 | 说明 |
|--------|-----------|------|
| Lidar | `velodyne128`, `laser_link` | 激光雷达坐标系 |
| Camera | `camera_link`, `camera_optical_frame` | 相机坐标系 |
| IMU | `imu_link` | 惯性测量单元坐标系 |
| GPS | `gps_link` | GPS接收器坐标系 |

## 使用方法

### 1. 配置静态变换

编辑 `static_transform.yaml` 文件，添加或修改变换配置：

```yaml
static_transforms:
  - name: my_sensor
    enabled: true
    frame_id: "base_link"
    child_frame_id: "sensor_link"
    translation:
      x: 0.1
      y: 0.0
      z: 0.2
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
```

### 2. 在代码中使用

```cpp
#include "autonomy/transform/static_transform.hpp"

using namespace autonomy::transform;

// 创建组件
auto static_tf = std::make_shared<StaticTransformComponent>();

// 初始化（加载配置）
std::string yaml_path = "configuration_files/transform/static_transform.yaml";
if (static_tf->Initialize(yaml_path)) {
    // 启动TF发布
    static_tf->Start();
    
    LOG(INFO) << "Loaded " << static_tf->GetTransformCount() << " transforms";
}

// 停止发布
static_tf->Stop();
```

## 四元数说明

### 常用旋转四元数

| 旋转 | x | y | z | w | 说明 |
|-----|---|---|---|---|------|
| 无旋转 | 0.0 | 0.0 | 0.0 | 1.0 | 单位四元数 |
| 绕X轴90° | 0.707 | 0.0 | 0.0 | 0.707 | - |
| 绕Y轴90° | 0.0 | 0.707 | 0.0 | 0.707 | - |
| 绕Z轴90° | 0.0 | 0.0 | 0.707 | 0.707 | - |
| 绕Z轴180° | 0.0 | 0.0 | 1.0 | 0.0 | 后向朝前 |

### 欧拉角转四元数

如果你有欧拉角 (roll, pitch, yaw)，可以使用以下公式转换为四元数：

```python
import math

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w
```

## 示例配置

### 示例1: 前置相机

```yaml
- name: front_camera
  enabled: true
  frame_id: "base_link"
  child_frame_id: "camera_link"
  translation:
    x: 0.15    # 相机在机器人前方15cm
    y: 0.0
    z: 0.12    # 相机高度12cm
  rotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0     # 无旋转
```

### 示例2: 顶置激光雷达

```yaml
- name: velodyne
  enabled: true
  frame_id: "base_link"
  child_frame_id: "velodyne128"
  translation:
    x: 0.0
    y: 0.0
    z: 0.3     # 雷达在顶部30cm高
  rotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

### 示例3: 后置激光雷达（180度旋转）

```yaml
- name: rear_laser
  enabled: true
  frame_id: "base_link"
  child_frame_id: "rear_laser_link"
  translation:
    x: -0.3    # 雷达在机器人后方30cm
    y: 0.0
    z: 0.1
  rotation:
    x: 0.0
    y: 0.0
    z: 1.0     # 绕Z轴旋转180度
    w: 0.0
```

## 调试

### 打印变换信息

设置 `print_transforms_on_startup: true` 可以在启动时查看所有加载的变换。

### 验证四元数

设置 `validate_quaternion: true` 会自动验证并归一化四元数，确保其长度为1。

### 查看TF树

```bash
# 使用RViz或其他工具可视化TF树
# 确认各个坐标系的相对关系是否正确
```

## 常见问题

### Q: 如何确定传感器的位置？

A: 
1. 使用测量工具测量传感器相对于机器人基座的实际位置
2. 参考传感器的机械图纸
3. 使用标定工具（如手眼标定）

### Q: 四元数归一化失败怎么办？

A: 确保四元数的平方和为1，或者启用 `validate_quaternion` 让系统自动归一化。

### Q: 如何临时禁用某个变换？

A: 将该变换的 `enabled` 字段设置为 `false`。

### Q: 可以动态修改变换吗？

A: 静态变换通常在启动时加载。如需动态变换，应使用动态TF发布器。

## 参考

- [TF2 Documentation](http://wiki.ros.org/tf2)
- [Quaternion Calculator](https://quaternions.online/)
- [REP 105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)

