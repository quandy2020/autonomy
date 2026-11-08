# Voxel-SLAM 独立库 · 第 1 期：common 与 sensor

## 程序决定

八期独立库，位置 `autonomy/localization/voxel_slam/`，命名空间 `voxel_slam`。算法按新接口从 Argus `system/voxel` 搬运，不把 Argus、Atlas、PCL、protobuf、ROS 编成依赖。GTSAM 只在第 6、8 期出现。

本库使用 Google C++ Style：类型 PascalCase，函数 PascalCase，成员 `snake_case_`，常量 `kPascalCase`，文件 `snake_case.h` / `snake_case.cc`。这与仓库其余目录的 `.hpp` / `.cpp` 和 camelCase 方法不同，只适用于本库。文件头仍用仓库的 Apache 2.0 / Openbot 版权块。

| 期 | 内容 | 状态 |
|----|------|------|
| 1 | `common` + `sensor` + 文本回放 | 本文 |
| 2 | `AdaptiveVoxelMap` | 未写 |
| 3 | Initialization | 未写 |
| 4 | Odometry | 未写 |
| 5 | Local Mapping | 未写 |
| 6 | FactorGraphManager | 未写 |
| 7 | LoopClosureDetector | 未写 |
| 8 | BackendRunner + `main` | 未写 |

## 目标

第 1 期给出后续各期不再改签名的基础类型，以及一个不依赖数据集的回放源。验收：单独配置本目录，测试可执行文件按时间吐出扫描和 IMU，链接库只有 Eigen 和 pthread。

## 非目标

地图、前端、后端、`main`、KITTI / bag 读取、接入 `autonomy/localization/CMakeLists.txt`。父工程会拉取 OpenCV 和 PCL，第 1 期不进入该图。

## 目录与构建

```text
autonomy/localization/voxel_slam/
├── CMakeLists.txt
├── include/voxel_slam/
│   ├── common/
│   │   ├── bounded_queue.h
│   │   ├── ids.h
│   │   ├── keyframe.h
│   │   ├── so3.h
│   │   ├── state.h
│   │   └── time.h
│   └── sensor/
│       ├── file_replay_source.h
│       ├── sensor_source.h
│       └── sensor_types.h
├── src/sensor/file_replay_source.cc
└── test/
    ├── bounded_queue_test.cc
    ├── file_replay_test.cc
    ├── so3_test.cc
    ├── state_test.cc
    └── time_test.cc
```

`time.h`、`so3.h`、`state.h`、`ids.h`、`keyframe.h`、`bounded_queue.h`、`sensor_types.h`、`sensor_source.h` 只有头文件。唯一翻译单元是 `file_replay_source.cc`。

CMake：`cmake_minimum_required(VERSION 3.16)`，`project(voxel_slam LANGUAGES CXX)`，`CMAKE_CXX_STANDARD 17`。`find_package(Eigen3 REQUIRED)`，`find_package(Threads REQUIRED)`。静态库目标名 `voxel_slam`，公开包含 `include/`，链接 `Eigen3::Eigen` 与 `Threads::Threads`。

`VOXEL_SLAM_BUILD_TESTS`：作为顶层工程时默认 ON，被 `add_subdirectory` 时默认 OFF。测试是返回 0/1 的可执行文件，用 `add_test` 注册。不引入 GTest。

配置与运行：

```bash
cmake -S autonomy/localization/voxel_slam -B build/voxel_slam
cmake --build build/voxel_slam
ctest --test-dir build/voxel_slam --output-on-failure
```

## 标识

`include/voxel_slam/common/ids.h`：

| 别名 | 类型 | 无效值 |
|------|------|--------|
| `KeyframeId` | `std::int64_t` | `kInvalidKeyframeId = -1` |
| `SessionId` | `std::int32_t` | `kInvalidSessionId = -1` |
| `SubmapId` | `std::int64_t` | `kInvalidSubmapId = -1` |

第 1 期没有子图对象。`SubmapId` 只是整数，避免第 2 期改 `Keyframe`。

## 时间

`voxel_slam::common`，从 `argus/common/time.hpp` 按公式搬运，去掉 Argus 命名：

- `NowSeconds()`：`system_clock` 的 Unix 秒。
- `SteadyNowSeconds()`：`steady_clock` 秒，只用于测耗时。
- `SecondsToStamp(double timestamp_seconds, int32_t* seconds, uint32_t* nanoseconds)`：纳秒进位与 Argus 相同。任一指针为空则返回，不写。只规定 `timestamp_seconds >= 0`。
- `StampToSeconds(int32_t seconds, uint32_t nanoseconds)`：`seconds + nanoseconds * 1e-9`。

传感器时间戳一律是 `double` 秒，不出现 ROS 的 `sec/nsec` 类型。

## SO(3)

`voxel_slam::math`，公式来自 `argus/common/so3.hpp` 与 `imu_state.hpp` 的 `Jr` / `JrInv`。不定义宏。

- `SkewSymmetric(v)`
- `Exp(angle_axis)`，范数 `<= 1e-7` 时返回单位阵
- `Exp(angular_velocity, dt)`
- `Exp(v1, v2, v3)`，范数 `<= 1e-5` 时返回单位阵（与源码阈值一致，不统一成 `1e-7`）
- `Log(R)`
- `RotationMatrixToRollPitchYaw(R)`，ZYX，万向节奇异时 yaw = 0
- `RightJacobian(phi)`，来自 `Jr`，范数 `< 1e-9` 返回单位阵
- `RightJacobianInverse(R)`，来自 `JrInv`

## 状态

`voxel_slam::common::State` 对应 Argus `vxslam::ImuState`，不对应 `argus::common::State`。重力常数取体素里程计的 `9.8`，不用 `9.81`。

```cpp
constexpr double kGravity = 9.8;
constexpr int kStateDim = 15;

struct State {
  double t = 0.0;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  Eigen::Vector3d g = Eigen::Vector3d(0, 0, -kGravity);
  Eigen::Matrix<double, kStateDim, kStateDim> cov =
      Eigen::Matrix<double, kStateDim, kStateDim>::Identity();

  State() { Reset(); }
  void Reset();
};
```

`Reset()` 与 Argus 相同：位姿、速度、偏置、时间为零，`g` 恢复为 `(0,0,-kGravity)`，协方差先单位阵再乘 `1e-4`，右下偏置 6×6 块再设为 `1e-5`。默认构造调用 `Reset()`，因此默认协方差不是成员初始值里的单位阵。

`operator+=(dx)`：`dx` 为 15 维，顺序是旋转、位置、速度、陀螺偏置、加计偏置。旋转用右乘 `Exp(dx.head<3>())`。不更新 `g` 和 `t`。`operator-(other)` 用 `Log(other.R.transpose() * R)` 及位置、速度、偏置之差，同样不含 `g` 和 `t`。

`State` 含 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`。它不嵌入点云或地图。

## 传感器类型

`voxel_slam::sensor`。

`ImuMeasurement`：`timestamp`（秒），`angular_velocity`（rad/s，IMU 系），`linear_acceleration`（比力，m/s²，IMU 系）。无 `frame_id`。含对齐宏。容器为

```cpp
using ImuMeasurementList =
    std::vector<ImuMeasurement, Eigen::aligned_allocator<ImuMeasurement>>;
```

`LidarPoint` 是普通 float 结构，不用 Eigen 成员，从而 `std::vector<LidarPoint>` 不需要对齐分配器：

| 字段 | 单位 | 含义 |
|------|------|------|
| `x, y, z` | m | 发射时刻的雷达系坐标 |
| `intensity` | — | 缺失时为 0 |
| `relative_time` | s | 相对该扫描起点的时间，对应 Argus 预处理后的 `curvature` |

`LidarScan::timestamp` 是扫描起点 `pcl_beg_time`（秒）。点的绝对时间是 `timestamp + relative_time`。点落在雷达系，去畸变由第 4 期做。原始点不带法向和协方差；`PointVar` 属于里程计，不放进传感器契约。

## 关键帧

```cpp
struct Keyframe {
  KeyframeId id = kInvalidKeyframeId;
  SessionId session_id = 0;
  SubmapId submap_id = kInvalidSubmapId;
  State state;
  std::shared_ptr<const LidarScan> scan;
  ImuMeasurementList imu;
};
```

`session_id = 0` 表示第一段会话，与“无效”不同。`scan` 为空、`imu` 为空都合法。`imu` 是形成该关键帧时使用的测量，供第 5 期滑窗使用，第 1 期不解释其内容。

## 有界队列

`voxel_slam::common::BoundedQueue<T>`。Argus 的同名队列不设容量，本库不搬运它。

- 构造要求 `capacity > 0`，否则抛 `std::invalid_argument`。
- 不可拷贝、不可移动。
- `TryPush(const T&)` / `TryPush(T&&)`：队列未关闭且未满时放入并唤醒一个等待者，返回 true；已满或已关闭返回 false。调用线程不阻塞。
- `TryPop(T*)`：空指针、或队列为空时返回 false，不写出元素。队列已关闭但里面还有元素时，仍取出并返回 true。
- `WaitPop(T*)`：有元素则取出并返回 true，关闭与否都一样。队列为空且未关闭时阻塞。队列为空且已关闭时返回 false，不写出元素。空指针返回 false，不阻塞。
- `Close()`：可重复调用。把队列标为关闭并唤醒全部等待者。之后的 `TryPush` 返回 false。关闭前已经入队的元素仍可被 `TryPop` / `WaitPop` 取完。
- 析构调用 `Close()`。
- `T` 须可移动构造。

前端以后只用 `TryPush`。队列满时由调用方丢帧，优化线程不能把前端堵住。

## 数据源

```cpp
class SensorSource {
 public:
  virtual ~SensorSource() = default;
  virtual bool HasNext() const = 0;
  virtual bool GetNextLidarScan(LidarScan* scan) = 0;
  virtual bool GetImuMeasurements(double start_time, double end_time,
                                  ImuMeasurementList* measurements) = 0;
};
```

单线程使用，类本身不加锁。

`HasNext()` 只看是否还有未消费的扫描。

`GetNextLidarScan` 按时间升序消费一帧。`scan == nullptr` 或已耗尽时返回 false，且不修改 `*scan`。成功时覆盖 `*scan`。

`GetImuMeasurements` 不消费。写入 `timestamp ∈ [start_time, end_time)` 的样本，按时间升序。空区间是成功：清空输出向量并返回 true。`measurements == nullptr`，或 `end_time < start_time`，返回 false 且不修改输出。`end_time == start_time` 得到空向量并返回 true。半开区间让相邻扫描的共享端点只属于后一次查询。调用方在扫描起点语义下传入 `[上一帧.timestamp, 当前帧.timestamp)`。

## 文本回放

`FileReplaySource` 实现 `SensorSource`。

```cpp
std::unique_ptr<SensorSource> CreateFileReplaySource(
    const std::string& directory, std::string* error);
```

`error` 可为空。失败返回空指针；`error` 非空时写入原因。不抛解析异常。成功后 IMU 与扫描索引都留在内存里。

目录：

```text
<directory>/imu.txt
<directory>/scans.txt
<directory>/scans/<name>.txt
```

`imu.txt` 每行：`timestamp wx wy wz ax ay az`。`scans.txt` 每行：`timestamp relative_path`，路径相对 `directory`。扫描文件每行：`x y z intensity relative_time`。`#` 开头和空行忽略。

`Open` 语义（工厂内部调用，失败即返回空指针）：

- `imu.txt` 或 `scans.txt` 打不开：失败，说明路径。
- 两个文件都存在且合法，即使行数为 0：成功。`HasNext()` 为 false。
- 词法错误、非有限数、列数不对：失败，说明路径和行号。
- `relative_time < 0`：失败。
- `scans.txt` 指向的文件打不开或内容非法：在 `Open` 时失败，不拖到播放中途。
- IMU 与扫描都按时间戳稳定排序后再提供。相同时间戳保持文件中的相对顺序。

播放：`GetNextLidarScan` 按排序后的扫描顺序。`GetImuMeasurements` 在已排序的 IMU 上做区间查询，可重复调用。

## 错误处理

| 情况 | 行为 |
|------|------|
| 回放文件打不开或行非法 | 工厂返回空指针，可选错误字符串 |
| 输出指针为空 | 对应函数返回 false |
| IMU 区间反转 | 返回 false |
| 队列容量 ≤ 0 | 抛 `std::invalid_argument` |
| 队列已满 | `TryPush` 返回 false |
| 队列已关闭且为空 | `WaitPop` 返回 false |

第 1 期不链接 glog。

## 测试

`state_test`：`Reset` 后重力为 `(0,0,-9.8)`，协方差 `(0,0)` 为 `1e-4`，偏置块为 `1e-5`。对一个非零 `dx`，`+=` 后的旋转等于 `Exp` 右乘，位置与两个偏置按分量相加。`a - b` 的旋转块等于 `Log(b.R^T a.R)`。

`so3_test`：`Exp(0)` 为单位阵。取 `phi = (0.3, -0.2, 0.4)`，`Log(Exp(phi))` 与 `phi` 的差小于 `1e-9`。`RightJacobian` 在零向量处为单位阵。

`time_test`：`1.5` 秒经 `SecondsToStamp` 再 `StampToSeconds`，绝对误差小于 `1e-9`。小数部分舍入到 `1e9` 纳秒时秒加一、纳秒回绕。空指针调用不崩溃。

`bounded_queue_test`：容量 1 时第二次 `TryPush` 失败；弹出后可再推入；顺序为 FIFO。另一线程 `WaitPop` 时 `Close()`，该调用返回 false。关闭前已入队的元素仍能 `TryPop`。

`file_replay_test` 在临时目录写三帧扫描和穿插的 IMU：

- 扫描按时间而不是文件书写顺序输出。
- 对相邻两帧起点做 `[t0, t1)` 查询时，时间戳等于 `t1` 的 IMU 不出现，等于 `t0` 的出现。
- 同一区间查询两次，结果相同。
- 三帧之后 `HasNext()` 为 false，再取扫描返回 false。
- 缺 `imu.txt` 时工厂返回空指针，且 `error` 非空。
- IMU 文件乱序写入时，查询结果仍按时间升序。

## 自检

- 无未定项。扫描时间是起点；IMU 区间是半开；点是 float；相对时间单位是秒；重力是 9.8；队列推入不阻塞。
- 第 1 期头文件不包含 GTSAM、PCL、OpenCV、protobuf。
- `Keyframe` 已带 `submap_id` 与 `imu`，第 2–5 期只填值，不改这个结构。
