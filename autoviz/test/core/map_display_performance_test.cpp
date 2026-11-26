// Copyright 2026 The Openbot Authors
#include <chrono>
#include <cstdio>
#include "autoviz/common/display_context.hpp"
#include "autoviz/plugins/displays/map/map_display.hpp"
#include "autoviz/rendering/scene_overlay.hpp"
class MapProbe : public autoviz::display::MapDisplay {
 public:
  MapProbe() : MapDisplay("/map") {}
  using MapDisplay::onDraw;
  using MapDisplay::processMessage;
};
int main() {
  autoviz::common::DisplayContext context;
  context.fixed_frame="map";
  MapProbe display;
  display.setContext(&context);
  automsgs::msgs::map_msgs::OccupancyGrid grid;
  grid.mutable_header()->set_frame_id("map");
  grid.mutable_info()->set_width(2048); grid.mutable_info()->set_height(2560);
  grid.mutable_info()->set_resolution(.05f);
  grid.mutable_info()->mutable_origin()->mutable_orientation()->set_w(1);
  grid.mutable_data()->Resize(2048*2560,0);
  for (int i=0; i<grid.data_size(); ++i) grid.set_data(i,i%102-1);
  qint64 last_key = 0;
  for (int n=0;n<5;++n) {
    grid.set_data(0,n);
    auto start=std::chrono::steady_clock::now();
    display.processMessage(grid);
    autoviz::rendering::SceneOverlay scene;
    display.onDraw(scene);
    if (scene.texturedBatches().size()!=1 || scene.texturedBatches()[0].image.size()!=QSize(2048,2560)) return 1;
    const QImage& image = scene.texturedBatches()[0].image;
    if (image.cacheKey() == last_key || image.pixelColor(0, 2559).red() != 255 - 255*n/100) return 1;
    last_key = image.cacheKey();
    std::printf("map_5242880_cells_ms=%.3f\n",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
  }
  display.processMessage(grid);
  autoviz::rendering::SceneOverlay identical;
  display.onDraw(identical);
  if (identical.texturedBatches()[0].image.cacheKey() != last_key) return 1;
  std::puts("PASS large-map coloring and identical-publish texture reuse");
}

