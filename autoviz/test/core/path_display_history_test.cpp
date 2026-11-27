// Copyright 2026 The Openbot Authors
#include <cstdio>

#include "autoviz/common/display_context.hpp"
#include "autoviz/plugins/displays/path/path_display.hpp"

class PathProbe : public autoviz::display::PathDisplay {
 public:
  PathProbe() : PathDisplay("/trajectory") {}
  using PathDisplay::processMessage;
  using PathDisplay::onDraw;
};

int main() {
  autoviz::common::DisplayContext context;
  context.fixed_frame = "map";
  PathProbe display;
  display.setContext(&context);
  display.setPropertyValue("prune_behind_robot", "false");
  display.setPropertyValue("hide_when_arrived", "false");
  display.setPropertyValue("buffer_length", "1");
  automsgs::msgs::nav_msgs::Path path;
  path.mutable_header()->set_frame_id("map");
  auto check = [&](int count, float first_x, float last_x) {
    display.processMessage(path);
    autoviz::rendering::SceneOverlay scene;
    display.onDraw(scene);
    const auto& vertices = scene.lineVertices();
    return vertices.size() == static_cast<size_t>((count - 1) * 2) &&
           vertices.front().position.x() == first_x && vertices.back().position.x() == last_x;
  };
  for (int i = 0; i < 20005; ++i) {
    auto* pose = path.add_poses();
    pose->mutable_pose()->mutable_position()->set_x(i);
    pose->mutable_pose()->mutable_orientation()->set_w(1);
  }
  if (!check(20005, 0, 20004)) return 1;
  path.mutable_poses(0)->mutable_pose()->mutable_position()->set_x(-5);
  path.mutable_poses(10000)->mutable_pose()->mutable_position()->set_y(3);
  auto* pose = path.add_poses();
  pose->mutable_pose()->mutable_position()->set_x(20005);
  pose->mutable_pose()->mutable_orientation()->set_w(1);
  if (!check(20006, -5, 20005)) return 1;
  autoviz::rendering::SceneOverlay optimized;
  display.onDraw(optimized);
  if (optimized.lineVertices()[20000].position.y() != 3.f) return 1;
  path.clear_poses();
  display.processMessage(path);
  autoviz::rendering::SceneOverlay cleared;
  display.onDraw(cleared);
  if (!cleared.lineVertices().empty()) return 1;
  std::puts("PASS complete 20005-pose trajectory, optimized history replacement and clear");
  return 0;
}
