// Copyright 2026 The Openbot Authors
#include <atomic>
#include <cmath>
#include <cstdio>
#include <limits>
#include <unistd.h>
#include <QApplication>
#include <zlib.h>
#include "autoviz/common/display_catalog.hpp"
#include "autoviz/common/display_factory.hpp"
#include "autoviz/common/display_context.hpp"
#include "autoviz/plugins/displays/submaps/submaps_display.hpp"
#include "autoviz/rendering/scene_overlay.hpp"
#include "autoviz/transform/buffer.hpp"

namespace proto = autonomy::localization::cartographer::proto;
namespace {
class Probe : public autoviz::display::SubmapsDisplay {
 public:
  explicit Probe(std::string channel) : SubmapsDisplay(std::move(channel)) {}
  using SubmapsDisplay::processMessage;
  using SubmapsDisplay::onDraw;
  using SubmapsDisplay::onUpdate;
};
std::atomic<int> failures{0};
void Check(bool ok, const char* what) {
  if (!ok) { ++failures; std::fprintf(stderr, "FAIL: %s\n", what); }
}
std::string Gzip(const std::string& raw) {
  z_stream stream{};
  if (deflateInit2(&stream, Z_DEFAULT_COMPRESSION, Z_DEFLATED, 16 + MAX_WBITS, 8, Z_DEFAULT_STRATEGY) != Z_OK) return {};
  std::string out(256, '\0');
  stream.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(raw.data()));
  stream.avail_in = raw.size();
  stream.next_out = reinterpret_cast<Bytef*>(out.data());
  stream.avail_out = out.size();
  Check(deflate(&stream, Z_FINISH) == Z_STREAM_END, "fixture compresses");
  out.resize(stream.total_out);
  deflateEnd(&stream);
  return out;
}
autoviz::rendering::SceneOverlay Render(Probe& probe) {
  autoviz::rendering::SceneOverlay scene;
  probe.onDraw(scene);
  return scene;
}
}  // namespace

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  autoviz::integration::AutolinkContext link;
  const auto suffix = std::to_string(getpid());
  if (!link.initialize(argv[0], "autoviz_submaps_test_" + suffix)) return 1;
  autoviz::integration::ChannelReaderRegistry::instance().setNode(link.node());
  std::atomic<int> queries{0}, version{1};
  std::atomic<bool> malformed{false};
  const std::string service_name = "/submaps_test_query_" + suffix;
  auto service = link.node()->CreateService<proto::SubmapQueryRequest, proto::SubmapQueryResponse>(
      service_name, [&](const auto& request, auto& response) {
        ++queries;
        Check(request->trajectory_id() == 2 && request->submap_index() == 7, "correct submap queried");
        response->set_submap_version(version);
        auto* texture = response->add_textures();
        texture->set_width(malformed ? std::numeric_limits<int>::max() : 2);
        texture->set_height(2);
        texture->set_resolution(.5);
        texture->mutable_slice_pose()->mutable_position()->set_x(1);
        texture->mutable_slice_pose()->mutable_position()->set_y(2);
        texture->mutable_slice_pose()->mutable_orientation()->set_w(1);
        texture->set_cells(Gzip(std::string({127, 0, 0, 127, 0, 0, 30, 50})));
        auto* low = response->add_textures();
        *low = *texture;
        low->set_width(1); low->set_height(1); low->set_resolution(1.);
        low->set_cells(Gzip(std::string({90, 100})));
      });
  {
    autoviz::common::DisplayContext context;
    context.autolink = &link;
    context.fixed_frame = "map";
    context.tf_buffer = &autoviz::transform::Buffer::instance();
    Probe probe("/submaps_test_list_" + suffix);
    probe.setContext(&context);
    probe.setPropertyValue("submap_query_service", service_name);
    probe.setPropertyValue("tracking_frame", "map");
    proto::SubmapList list;
    list.mutable_header()->set_frame_id("map");
    auto* entry = list.add_submap();
    entry->set_trajectory_id(2); entry->set_submap_index(7); entry->set_submap_version(1);
    auto* pose = entry->mutable_pose();
    pose->mutable_position()->set_x(10); pose->mutable_position()->set_y(20); pose->mutable_position()->set_z(1);
    pose->mutable_orientation()->set_z(std::sqrt(.5)); pose->mutable_orientation()->set_w(std::sqrt(.5));
    probe.processMessage(list);
    auto wait_for_image = [&] {
      for (int n = 0; n < 300; ++n) {
        probe.onUpdate();
        auto scene = Render(probe);
        if (!scene.texturedBatches().empty()) return true;
        usleep(10000);
      }
      return false;
    };
    Check(wait_for_image(), "async RPC loads submap textures");
    auto scene = Render(probe);
    if (scene.texturedBatches().empty()) return 1;
    const auto& batch = scene.texturedBatches()[0];
    const auto key = batch.image.cacheKey();
    Check(batch.premultiplied_alpha && !batch.depth_write, "RViz premultiplied X-ray blending");
    Check(batch.image.pixelColor(0, 0) == QColor(127, 127, 127, 0) &&
          batch.image.pixelColor(1, 0) == QColor(0, 0, 0, 127) &&
          batch.image.pixelColor(0, 1) == QColor(0, 0, 0, 0), "intensity/alpha and unknown cells preserved");
    Check((batch.vertices[0].position - QVector3D(8, 21, 1)).length() < 1e-5 &&
          (batch.vertices[1].position - QVector3D(8, 20, 1)).length() < 1e-5 &&
          (batch.vertices[2].position - QVector3D(9, 20, 1)).length() < 1e-5,
          "Cartographer's transposed texture axes and nested poses match RViz");
    Check(scene.lineVertices().size() == 6 && scene.expandedTexturedBatches(QMatrix4x4()).size() == 2,
          "submap axes and ID label drawn");
    const int before = queries;
    pose->mutable_position()->set_x(11); pose->mutable_position()->set_z(2);
    probe.processMessage(list);
    probe.onUpdate();
    auto moved = Render(probe);
    Check(moved.texturedBatches()[0].image.cacheKey() == key && queries == before,
          "pose-only updates reuse texture without RPC");
    Check(std::abs(moved.texturedBatches()[0].vertices[0].color.w() - .5f) < 1e-5,
          "RViz height fade begins at one metre and ends at three metres");
    probe.setPropertyValue("alpha", "0.4");
    auto faded = Render(probe);
    Check(std::abs(faded.texturedBatches()[0].vertices[0].color.x() - .2f) < 1e-5 &&
          faded.texturedBatches()[0].image.cacheKey() == key, "fade changes tint without rebuilding texture");
    probe.setPropertyValue("trajectory_id", "3");
    Check(Render(probe).texturedBatches().empty(), "trajectory filter hides other trajectories");
    probe.setPropertyValue("trajectory_id", "2");
    probe.setPropertyValue("high_resolution", "false"); probe.setPropertyValue("low_resolution", "true");
    Check(Render(probe).texturedBatches()[0].image.size() == QSize(1, 1), "low-resolution slice selected");
    probe.setPropertyValue("high_resolution", "true");
    Check(Render(probe).texturedBatches().size() == 2, "both resolution slices can be shown");
    version = 0; entry->set_submap_version(0);
    probe.processMessage(list);
    Check(Render(probe).texturedBatches().empty(), "version rollback clears restarted Cartographer's cache");
    Check(wait_for_image() && queries > before, "restart fetches fresh textures");
    malformed = true; version = 2; entry->set_submap_version(2);
    probe.reset(); probe.processMessage(list);
    for (int n = 0; n < 30; ++n) { probe.onUpdate(); usleep(10000); }
    Check(Render(probe).texturedBatches().empty(), "oversized or malformed texture is rejected safely");
    list.clear_submap(); probe.processMessage(list);
    Check(Render(probe).texturedBatches().empty(), "trimmed submaps release cached geometry");
    auto config = autoviz::common::DisplayFactory::defaultForType("Submaps");
    Check(config.channel == "/submap_list" && autoviz::common::DisplayFactory::create(config) != nullptr,
          "Submaps is registered and defaults to the native channel");
  }
  autoviz::integration::ChannelReaderRegistry::instance().setNode(nullptr);
  if (!failures) std::puts("PASS Submaps RPC, RViz pixels/poses/fade, version cache, filters, restart and malformed data");
  return failures ? 1 : 0;
}
