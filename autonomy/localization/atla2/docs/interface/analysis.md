# 接口分析

## 1. 分层接口一览

| 层 | 头文件 | 核心类型 / 方法 |
|----|--------|-----------------|
| Sensor | `sensor/interface/sensor_base.hpp` | `Init/Start/Stop/LatestTime` |
| Sensor data | `sensor/interface/types.hpp` | `SensorData` `ImuSample` `ImageFrame` `LidarScan` |
| Frontend | `frontend/interface/frontend_base.hpp` | `Process` `GetResult` → `OdometryResult` |
| Backend | `backend/interface/optimizer_base.hpp` | `Update` `UpdateWithSensors` `GetState` |
| Pipeline | `pipeline/slam_system.hpp` | `Push*` `Step` `GetOdometry` |
| Fusion | `fusion/degradation/degradation.hpp` 等 | `Evaluate` `RecommendedMode` |
| Map | `map/map_manager.hpp` | `AddKeyframe` `UpdateFromOdometry` |

命名空间：`autonomy::localization::atla2`。

## 2. SensorBase

```cpp
class SensorBase {
  virtual SensorKind Kind() const = 0;
  virtual bool Init(const Atla2Config& cfg) = 0;
  virtual bool Start() = 0;
  virtual void Stop() = 0;
  virtual TimeStamp LatestTime() const = 0;
};
```

**契约**：核心只依赖接口；ROS/SDK 适配在实现类内部。失败返回 `false`，不抛异常。

`SensorSuite` 按 mode 装配 camera/imu/lidar/…，`Poll` → sync。

## 3. FrontendBase / OdometryResult

```cpp
struct OdometryResult {
  TimeStamp t;
  SE3 pose;          // T_W_B
  Vec3 velocity;
  Vec3 gyro_bias, accel_bias;
  Covariance6 cov;
  std::vector<Landmark> landmarks;  // VIO/LIVO
  PointCloud local_map;             // LIO/LIVO
  bool valid;
};

class FrontendBase {
  virtual bool Init(const Atla2Config&) = 0;
  virtual bool Process(const SensorData&) = 0;
  virtual bool GetResult(OdometryResult*) = 0;
  virtual void Reset() = 0;
  virtual FrontendMode Mode() const = 0;
};
```

工厂：`CreateFrontend(cfg)`（`frontend/factory`）。

**不变量**：`Process` 成功后 `GetResult` 在 `valid==true` 时可用；失败由 pipeline 置 Lost。

## 4. OptimizerBase

```cpp
class OptimizerBase {
  virtual bool Init(const Atla2Config&) = 0;
  virtual bool Update(const OdometryResult&) = 0;
  virtual bool UpdateWithSensors(const OdometryResult&, const SensorData&);
  virtual bool GetState(OdometryResult*) const = 0;
  virtual void Reset() = 0;
};
```

- IEKF：协方差加权融合前端 odom  
- Graph：滑动窗口 + IMU/相对位姿/视觉因子  

工厂：`CreateOptimizer(cfg)`。

## 5. SlamSystem（对外主 API）

| 方法 | 说明 |
|------|------|
| `Init` / `InitFromFile` | 加载配置并创建前后端/地图 |
| `PushImu/Image/Lidar` | 入同步队列 |
| `Step()` / `Step(SensorData)` | 处理一包 |
| `GetOdometry` | 最新融合位姿 |
| `state()` | `SlamState` |
| `map()` / `extrinsics()` | 地图与外参访问 |
| `Reset` | 清空状态 |

适配层（ROS/gRPC）应 **只调用 SlamSystem**，不直连前端。

## 6. 配置接口

`LoadConfig(path, &Atla2Config)` 解析平台 YAML 关键字段：`frontend.mode/fusion`、`backend.*`、`map.*`、`sync_tol_ms`、`sensor.gps.enabled`。

扩展字段应保持向后兼容：未知键忽略。

## 7. 进程入口

| Binary | 角色 |
|--------|------|
| `atla2_offline` | 合成/离线冒烟 |
| `atla2_node` | 在线 Push/Step 壳 |
| `atla2_benchmark` | 步进计时 |
| `atla2_*_calib` | 标定 stub |

## 8. 接口演进规则

1. 不破坏 `OdometryResult` 字段语义；新增字段给默认值。  
2. 虚接口只增不改签名；废弃用注释标记。  
3. 跨语言 / ROS msg 在 `interface/`（规划）做映射，不反渗算法头文件。  

## 9. 相关代码

- `pipeline/slam_system.hpp`  
- `frontend/interface/frontend_base.hpp`  
- `backend/interface/optimizer_base.hpp`  
- `sensor/interface/sensor_base.hpp`  
