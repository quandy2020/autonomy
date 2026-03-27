#include "autonomy/autoviz/core/convert/message/convert_nav_path_to_scene.hpp"

#if defined(AUTOVIZ_HAS_FOXGLOVE) && AUTOVIZ_HAS_FOXGLOVE && \
    defined(AUTOVIZ_FOXGLOVE_HAS_ENCODE) && AUTOVIZ_FOXGLOVE_HAS_ENCODE

namespace autoviz {
namespace converter {
namespace {

foxglove::schemas::Timestamp StampFromHeader(
    const automsgs::msgs::std_msgs::Header& header) {
  foxglove::schemas::Timestamp ts;
  if (!header.has_stamp()) {
    return ts;
  }
  const auto& st = header.stamp();
  if (st.sec() < 0) {
    ts.sec = 0;
  } else {
    ts.sec = static_cast<uint32_t>(st.sec());
  }
  ts.nsec = st.nanosec();
  return ts;
}

void SetEntityHeader(foxglove::schemas::SceneEntity& entity,
                       const automsgs::msgs::std_msgs::Header& header) {
  entity.timestamp = StampFromHeader(header);
  entity.frame_id = header.frame_id();
}

foxglove::schemas::Color LineColorBlue() {
  foxglove::schemas::Color c;
  c.r = 0.0;
  c.g = 0.0;
  c.b = 1.0;
  c.a = 0.85;
  return c;
}

}  // namespace

foxglove::schemas::SceneUpdate PathToSceneUpdate(const automsgs::msgs::nav_msgs::Path& path) {
  foxglove::schemas::SceneUpdate scene_update;
  if (path.poses_size() == 0) {
    return scene_update;
  }

  foxglove::schemas::SceneEntity entity;
  SetEntityHeader(entity, path.header());
  entity.id = "path";
  entity.frame_locked = false;

  foxglove::schemas::LinePrimitive line;
  line.type = foxglove::schemas::LinePrimitive::LineType::LINE_STRIP;
  line.thickness = 0.08;
  line.scale_invariant = false;
  line.color = LineColorBlue();

  for (int i = 0; i < path.poses_size(); ++i) {
    const auto& ps = path.poses(i);
    if (!ps.has_pose() || !ps.pose().has_position()) {
      continue;
    }
    const auto& p = ps.pose().position();
    foxglove::schemas::Point3 pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
    line.points.push_back(pt);
  }

  if (!line.points.empty()) {
    entity.lines.push_back(std::move(line));
    scene_update.entities.push_back(std::move(entity));
  }

  return scene_update;
}

}  // namespace converter
}  // namespace autoviz

#endif
