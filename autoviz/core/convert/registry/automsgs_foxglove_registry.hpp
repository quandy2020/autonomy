// automsgs protobuf 与 Foxglove 官方 schemas（见 foxglove-sdk/schemas/README.md）的对照与策略。
#pragma once

#include <string>

namespace autoviz {
namespace converter {

/// 在 Studio 中展示 automsgs 消息的推荐方式（与 README 中各 schema 对应）。
enum class AutomsgsFoxgloveStrategy {
  /// 保持 autolink/protobuf 透传：schema 为 FileDescriptorSet，面板多用 Raw Message / 自定义。
  kPassthroughProtobuf = 0,
  /// 转为 foxglove.SceneUpdate（LinePrimitive / CubePrimitive / ArrowPrimitive 等），用 3D 面板。
  kConvertToSceneUpdate,
  /// 目标为 foxglove.LaserScan（需字段级转换，待实现）。
  kConvertToLaserScan,
  /// 目标为 foxglove.PointCloud（待实现）。
  kConvertToPointCloud,
  /// 目标为 foxglove.CompressedImage / RawImage（待实现）。
  kConvertToImage,
  /// 目标为 foxglove.Grid（如 OccupancyGrid，待实现）。
  kConvertToGrid,
  /// 目标为 foxglove.LocationFix（如 NavSatFix，待实现）。
  kConvertToLocationFix,
  /// foxglove.PoseInFrame（PoseStamped、Odometry.pose 等）。
  kConvertToPoseInFrame,
  /// foxglove.FrameTransform（TransformStamped）。
  kConvertToFrameTransform,
  /// foxglove.PosesInFrame（PoseArray）。
  kConvertToPosesInFrame,
  /// 转为 foxglove.Log（通用：TextFormat 或专用摘要；保证每条 automsgs 消息都有可运行转换路径）。
  kConvertToLog,
};

struct AutomsgsFoxgloveEntry {
  /// automsgs 消息全名，如 automsgs.msgs.nav_msgs.Path
  std::string proto_full_name;
  /// Foxglove 官方 schema 名（与 foxglove-sdk C++ `foxglove::schemas::*::schema().name` 一致，如 foxglove.SceneUpdate）。
  std::string foxglove_canonical_name;
  AutomsgsFoxgloveStrategy strategy{AutomsgsFoxgloveStrategy::kPassthroughProtobuf};
  /// 在 Foxglove Studio 中建议使用的面板或说明。
  std::string studio_hint;
};

// 若存在映射则返回指针（静态存储）；否则 nullptr。
const AutomsgsFoxgloveEntry* FindFoxgloveRule(const std::string& proto_full_name);

}  // namespace converter
}  // namespace autoviz
