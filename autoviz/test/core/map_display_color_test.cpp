// Copyright 2026 The Openbot Authors

#include <cstdio>

#include <QColor>

#include "autoviz/common/display_context.hpp"
#include "autoviz/plugins/displays/map/map_display.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace {

class MapProbe : public autoviz::display::MapDisplay {
 public:
  MapProbe() : MapDisplay("/map") {}
  using MapDisplay::onDraw;
  using MapDisplay::processMessage;
};

int failures = 0;

void Check(bool condition, const char* description) {
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", description);
    ++failures;
  }
}

QImage Render(MapProbe* display) {
  autoviz::rendering::SceneOverlay scene;
  display->onDraw(scene);
  Check(scene.texturedBatches().size() == 1, "one occupancy texture is produced");
  if (scene.texturedBatches().empty()) {
    return {};
  }
  Check(scene.texturedBatches()[0].filter_mode ==
            autoviz::rendering::SceneOverlay::TextureFilterMode::kNearest,
        "nearest filtering preserves occupancy palette colors");
  return scene.texturedBatches()[0].image;
}

}  // namespace

int main() {
  autoviz::common::DisplayContext context;
  context.fixed_frame = "map";
  MapProbe display;
  display.setContext(&context);

  automsgs::msgs::map_msgs::OccupancyGrid grid;
  grid.mutable_header()->set_frame_id("map");
  grid.mutable_info()->set_width(8);
  grid.mutable_info()->set_height(2);
  grid.mutable_info()->set_resolution(0.05f);
  grid.mutable_info()->mutable_origin()->mutable_orientation()->set_w(1);
  for (int cell : {0, 1, 25, 50, 75, 99, 100, -1,
                   -128, -64, -2, 101, 127, 255, 0, 100}) {
    grid.add_data(cell);
  }
  display.processMessage(grid);
  const QImage defaults = Render(&display);
  Check(!defaults.isNull() && defaults.pixelColor(0, 1).alpha() == 179,
        "RViz Map default Alpha is 0.7");

  display.setPropertyValue("alpha", "1.0");
  const QImage opaque = Render(&display);
  // ROS RViz noetic MapDisplay::makeMapPalette(), including diagnostic colors.
  const QColor expected[] = {
      {255, 255, 255}, {253, 253, 253}, {192, 192, 192}, {128, 128, 128},
      {64, 64, 64}, {3, 3, 3}, {0, 0, 0}, {112, 137, 134},
      {255, 0, 0}, {255, 129, 0}, {255, 255, 0}, {0, 255, 0},
      {0, 255, 0}, {112, 137, 134}, {255, 255, 255}, {0, 0, 0}};
  for (int i = 0; i < 16; ++i) {
    if (opaque.isNull() || opaque.pixelColor(i % 8, 1 - i / 8) != expected[i]) {
      std::fprintf(stderr, "FAIL: RViz palette cell %d\n", i);
      ++failures;
    }
  }
  display.setPropertyValue("alpha", "0.5");
  display.setPropertyValue("color_scheme", "costmap");
  const QImage costmap = Render(&display);
  Check(costmap.pixelColor(0, 1).alpha() == 0,
        "costmap free space remains transparent");
  Check(costmap.pixelColor(5, 1) == QColor(0, 255, 255, 128) &&
            costmap.pixelColor(6, 1) == QColor(255, 0, 255, 128),
        "costmap inscribed/lethal colors preserve display Alpha");
  display.setPropertyValue("color_scheme", "raw");
  const QImage raw = Render(&display);
  Check(raw.pixelColor(0, 0) == QColor(128, 128, 128, 128) &&
            raw.pixelColor(7, 1) == QColor(255, 255, 255, 128),
        "raw palette preserves signed byte and unknown colors");
  display.setPropertyValue("color_scheme", "map");
  display.setPropertyValue("decimate", "2");
  const QImage decimated = Render(&display);
  Check(decimated.size() == QSize(4, 1) &&
            decimated.pixelColor(0, 0) == QColor(253, 253, 253, 128),
        "decimation retains the highest known occupancy in each block");
  if (failures != 0) {
    return 1;
  }
  std::puts("PASS RViz map colors, signed cells, unknown alias, Alpha and nearest filtering");
  return 0;
}
