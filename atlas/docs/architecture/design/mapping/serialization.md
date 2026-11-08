# Mapping 地图与会话持久化

## 模块职责

`mapping/serialization/` 把运行时的 session、轨迹状态、关键帧观测、体素地图快照和图约束转换成稳定、可验证、可升级的持久化格式。serializer 不决定加载哪些历史 session、不修改运行时 map，也不依赖 ROS 参数或 AutoMsgs wire message。session 的发现、加载顺序和“是否发布到系统”由 `backend/global_mapping/PreviousSessionLoader` 控制。

## 目标目录和文件名

```text
mapping/serialization/
├── session_file_header.hpp                  # 会话文件头和值类型
├── session_file_version.hpp                   # 格式版本和支持范围
├── session_file_reader.hpp/.cpp             # 校验并读取一个完整会话文件
├── session_read_result.hpp             # 一次会话文件读取结果和值错误
├── session_file_writer.hpp/.cpp               # 写临时文件、flush、原子发布
├── session_write_result.hpp            # 一次会话文件写入结果和值错误
├── trajectory_serializer.hpp/.cpp        # 轨迹和位姿序列
├── keyframe_serializer.hpp/.cpp          # 关键帧元数据与观测引用
├── voxel_map_serializer.hpp/.cpp         # 体素键、点簇和平面统计
├── pose_constraint_serializer.hpp/.cpp    # 位姿图约束、ID、测量和不确定度
└── internal/
    ├── binary_reader.hpp/.cpp             # 边界检查和字节序处理
    ├── binary_writer.hpp/.cpp
    ├── session_file_checksum.hpp/.cpp       # 会话文件完整性校验
    ├── atomic_file_writer.hpp/.cpp         # 同目录临时文件与 rename
    └── legacy_session_importer.hpp/.cpp     # 原始 Voxel-SLAM 格式导入
```

读写职责分别由 `SessionFileReader` 和 `SessionFileWriter` 承担。若格式非常简单，也可以将小型编码函数放在各自实现文件中；不额外创建只转发这些函数的 serializer facade，也不为纯值类型强行配 `.cpp`。

## 格式和 API 设计

| 文件 | 类型 | 责任 |
|---|---|---|
| `session_file_header.hpp` | `SessionFileHeader` | 格式版本、session ID、坐标系、地图分辨率、记录数、生成时间和能力标志 |
| `session_file_version.hpp` | `SessionFileVersion` | 明确 major/minor 兼容规则及可读取版本范围 |
| `session_file_reader.*` | `SessionFileReader` | 先校验文件头、长度、计数、有限数值和 checksum，再返回完整临时结果 |
| `session_read_result.hpp` | `SessionReadResult` | 表达读取成功的会话快照或结构化失败原因 |
| `session_file_writer.*` | `SessionFileWriter` | 写临时会话文件、flush/close、完整性校验后原子 rename |
| `session_write_result.hpp` | `SessionWriteResult` | 表达写入状态、目标位置和结构化失败原因；失败不破坏旧文件 |
| `trajectory_serializer.*` | `TrajectorySerializer` | 时间戳、位置、旋转、速度、偏置、重力和协方差等有版本的轨迹字段 |
| `keyframe_serializer.*` | `KeyframeSerializer` | keyframe ID/session ID、pose、时间戳、点云/描述子资源关联 |
| `voxel_map_serializer.*` | `VoxelMapSerializer` | 空间键层级、点簇矩统计、平面有效性和地图版本 |
| `pose_constraint_serializer.*` | `PoseConstraintSerializer` | 端点 ID、变换方向、来源、协方差和验证元数据 |
| `legacy_session_importer.*` | `LegacySessionImporter` | 只读兼容原格式并转换到新值模型，不承担新格式写入 |

读取接口应返回包含错误码/上下文的 `SessionReadResult`，而不是 `bool` 并让输出参数留下部分数据。异常文件不得修改调用者已经发布的 session/map；先读入临时对象、全部验证后再交给 GlobalMapping 原子安装。

## 新会话文件布局

建议每个会话使用独立目录，保存稳定文件头和会话数据，例如：

```text
session/
├── header
├── trajectory
├── keyframes/
├── voxel_map
└── constraints
```

编码可由 Atlas 现有依赖和兼容要求最终确定。无论采用自有二进制、Protobuf 或其他编码，格式都必须规定字节序、浮点精度、记录数/长度上限、坐标系、四元数顺序、单位、checksum 与升级策略。格式不能直接序列化 C++ struct 的原始内存布局，也不能假设当前编译器 ABI 稳定。

## 旧 Voxel-SLAM 数据兼容

原始实现主要使用 `alidarState.txt`、`edge.txt`、按索引命名的 `.bin` body-frame 点云和 PCD。Atlas 当前 `voxel_map_serializer.*` 提供 TUM pose、Voxel-SLAM pose 文本、XYZ 点云和自定义二进制 XYZI IO。迁移策略：

1. `legacy_session_importer` 独立解析旧文本/二进制，校验每条记录的字段数量、有限数值、ID 范围和文件长度。
2. 将不同 legacy 文件组装为内存 `SessionSnapshot`；不让后续 SLAM 算法直接依赖 `alidarState.txt` 或 `.bin` 命名。
3. 成功后由新 `SessionFileWriter` 写入新版本；旧格式保留只读导入器，不再增加新格式写入分支。
4. 跨端格式显式规定 endian、整数宽度和 alignment，避免当前 `uint64_t + float[4]` 的 host-endian 原始写法成为长期 ABI。
5. 加载历史 session 由 backend 发布之前完成整包校验；失败不得留下半加载 keyframe、描述子索引或 pose graph 边。

## 线程、文件安全和性能

- serializer 处理不可变 snapshot；长文件 IO 不占用 frontend 地图锁。
- 写入使用同目录临时路径、flush/close、可选 fsync 和原子 rename；临时文件带 session/version，崩溃后可识别和清理。
- 加载分块 reserve，校验长度和数量上限，避免恶意/损坏文件触发超大分配；错误包含文件和记录偏移。
- checksum 与版本校验在发布前完成；同 session 并发保存必须通过 per-session 写入串行化或版本命名避免互相覆盖。
- 文件名、schema version、session ID 与 map version 都是值模型的一部分，禁止依赖目录遍历顺序分配 session ID。

## 文件职责归属

- `PreviousSessionLoader` 属于 `backend/global_mapping/`：它决定加载哪些 session、加载失败是否中止启动、何时把结果安装到 PoseGraph/descriptor index。
- `SessionFileReader/Writer` 属于本模块：它们只负责格式编解码和完整性。
- AutoMsgs 是在线消息适配，不是磁盘 schema；会话文件不应直接序列化消息 envelope。
- ROS bag/MCAP 转换不属于当前 Atlas 工具集；如需恢复，应作为独立的可选应用适配器实现，并调用公共读写 API，不混入核心库。
