// Copyright 2026 The Openbot Authors
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <limits>
#include <unistd.h>
#include <QApplication>
#include <zlib.h>
#include "autoviz/plugins/displays/submaps/constraints_display.hpp"
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
class ConstraintProbe : public autoviz::display::CartographerConstraintsDisplay {
 public:
  ConstraintProbe() : CartographerConstraintsDisplay("/constraint_test") {}
  using CartographerConstraintsDisplay::processMessage;
  using CartographerConstraintsDisplay::onDraw;
};
int failures = 0;
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
QVector3D Center(const autoviz::rendering::SceneOverlay::TexturedBatch& batch) {
  return .5f * (batch.vertices[0].position + batch.vertices[2].position);
}
}  // namespace

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  autoviz::integration::AutolinkContext link;
  const auto suffix = std::to_string(getpid());
  if (!link.initialize(argv[0], "autoviz_submaps_test_" + suffix)) return 1;
  auto& registry = autoviz::integration::ChannelReaderRegistry::instance();
  registry.setNode(link.node());
  const std::string channel = "/submaps_test_list_" + suffix;
  auto writer = link.node()->CreateWriter<proto::SubmapList>(channel);
  proto::SubmapList list;
  list.mutable_header()->set_frame_id("map");
  auto* entry = list.add_submap();
  entry->set_trajectory_id(2); entry->set_submap_index(7); entry->set_submap_version(1);
  auto* pose = entry->mutable_pose();
  pose->mutable_position()->set_x(10); pose->mutable_position()->set_y(20); pose->mutable_position()->set_z(1);
  pose->mutable_orientation()->set_z(std::sqrt(.5)); pose->mutable_orientation()->set_w(std::sqrt(.5));
  auto* other = list.add_submap();
  *other = *entry; other->set_trajectory_id(3);
  writer->Write(list);
  {
    autoviz::common::DisplayContext context;
    context.autolink = &link;
    context.fixed_frame = "map";
    context.tf_buffer = autoviz::transform::Buffer::Instance();
    context.tf_buffer->clear();
    Probe probe(channel);
    probe.setContext(&context);
    probe.setEnabled(true);
    autoviz::rendering::SceneOverlay scene;
    for (int n = 0; n < 200; ++n) {
      writer->Write(list); probe.onUpdate(); scene = Render(probe);
      if (scene.lineVertices().size() == 12) break;
      usleep(10000);
    }
    Check(scene.lineVertices().size() == 12, "late subscriber receives existing submap poses without RPC");
    auto labels = scene.expandedTexturedBatches(context.view_matrix);
    Check(labels.size() == 2 && !scene.triangleVertices().empty(), "IDs and solid RGB axes are rendered");
    if (labels.size() != 2 || scene.lineVertices().size() != 12) return 1;
    Check((scene.lineVertices()[1].position - QVector3D(10, 20.3, 1)).length() < 1e-5 &&
          scene.lineVertices()[0].color == QVector4D(1,0,0,1) &&
          scene.lineVertices()[2].color == QVector4D(0,1,0,1) &&
          scene.lineVertices()[4].color == QVector4D(0,0,1,1), "0.3 m RGB axes follow the submap orientation");
    Check((Center(labels[0]) - QVector3D(10, 20.1, 1.3)).length() < 1e-5 &&
          std::abs((labels[0].vertices[0].position-labels[0].vertices[5].position).length()-.2f) < 1e-5,
          "RViz label size and above-centred local offset match");
    bool red = false;
    for (int y=0;y<labels[0].image.height();++y) for (int x=0;x<labels[0].image.width();++x) {
      const auto c = labels[0].image.pixelColor(x,y);
      if (c.alpha()>0 && c.red()>200 && c.green()==0 && c.blue()==0) red=true;
    }
    Check(red, "submap IDs use RViz red");
    const auto key = labels[0].image.cacheKey();
    pose->mutable_position()->set_x(11);
    pose->mutable_orientation()->set_z(0); pose->mutable_orientation()->set_x(std::sqrt(.5));
    probe.processMessage(list);
    auto moved = Render(probe).expandedTexturedBatches(context.view_matrix);
    Check(moved[0].image.cacheKey() == key &&
          (Center(moved[0])-QVector3D(11,19.8,1)).length() < 1e-5,
          "pose optimization reuses labels and rotates their local offset");
    probe.setPropertyValue("trajectory_id", "2");
    Check(Render(probe).lineVertices().size()==6 && Render(probe).expandedTexturedBatches(context.view_matrix).size()==1,
          "trajectory filter selects only the requested poses");
    probe.setPropertyValue("show_ids", "false");
    Check(Render(probe).expandedTexturedBatches(context.view_matrix).empty(), "IDs toggle independently");
    probe.setPropertyValue("show_axes", "false");
    Check(Render(probe).lineVertices().empty() && Render(probe).triangleVertices().empty(), "axes toggle independently");
    probe.setPropertyValue("show_axes", "true");
    automsgs::msgs::geometry_msgs::TransformStamped tf;
    tf.mutable_header()->set_frame_id("map"); tf.mutable_header()->mutable_stamp()->set_sec(1);
    tf.set_child_frame_id("odom"); tf.mutable_transform()->mutable_translation()->set_x(5);
    tf.mutable_transform()->mutable_rotation()->set_w(1);
    context.tf_buffer->setTransform(tf,"test");
    list.mutable_header()->set_frame_id("odom"); probe.processMessage(list);
    Check((Render(probe).lineVertices()[0].position-QVector3D(16,20,1)).length()<1e-5, "map TF is composed with each submap pose");
    list.mutable_header()->set_frame_id("missing"); probe.processMessage(list);
    Check(Render(probe).lineVertices().empty() && probe.status().level==autoviz::display::DisplayStatusLevel::kWarn,
          "missing frame transform suppresses misplaced geometry");
    list.mutable_header()->set_frame_id("map");
    pose->mutable_position()->set_x(std::numeric_limits<double>::quiet_NaN()); probe.processMessage(list);
    Check(Render(probe).lineVertices().empty(), "nonfinite poses are rejected");
    probe.setPropertyValue("trajectory_id", "-1");
    probe.setPropertyValue("show_ids", "true");
    Check(Render(probe).expandedTexturedBatches(context.view_matrix).size()==1, "valid unfiltered submap remains visible");
    list.clear_submap(); probe.processMessage(list);
    Check(Render(probe).expandedTexturedBatches(context.view_matrix).empty(), "trimmed submaps release labels");
    auto config = autoviz::common::DisplayFactory::defaultForType("Submaps");
    const auto specs = probe.propertySpecs();
    Check(config.channel=="/submap_list" && autoviz::common::DisplayFactory::create(config)!=nullptr &&
          std::any_of(specs.begin(), specs.end(), [](const auto& spec) { return spec.key == "submaps"; }),
          "factory exposes submap visualization options");
    probe.setEnabled(false);
    proto::SubmapList textured;
    textured.mutable_header()->set_frame_id("map");
    auto* submap = textured.add_submap();
    submap->set_trajectory_id(2); submap->set_submap_index(7); submap->set_submap_version(1);
    submap->set_texture_version(1);
    auto* p = submap->mutable_pose();
    p->mutable_position()->set_x(10); p->mutable_position()->set_y(20); p->mutable_position()->set_z(1);
    p->mutable_orientation()->set_z(std::sqrt(.5)); p->mutable_orientation()->set_w(std::sqrt(.5));
    auto* texture = submap->add_textures();
    texture->set_width(2); texture->set_height(2); texture->set_resolution(.5);
    texture->mutable_slice_pose()->mutable_position()->set_x(1);
    texture->mutable_slice_pose()->mutable_position()->set_y(2);
    texture->mutable_slice_pose()->mutable_orientation()->set_w(1);
    texture->set_cells(Gzip(std::string({127,0,0,127,0,0,30,50})));
    auto* low = submap->add_textures(); *low = *texture;
    low->set_width(1); low->set_height(1); low->set_resolution(1.);
    low->set_cells(Gzip(std::string({90,100})));
    Probe textures("/texture_test"); textures.setContext(&context);
    textures.setPropertyValue("show_axes","false"); textures.setPropertyValue("show_ids","false");
    textures.processMessage(textured);
    auto image_scene = Render(textures);
    Check(image_scene.texturedBatches().size()==1,"topic-carried texture loads without a query client");
    if (image_scene.texturedBatches().empty()) return 1;
    const auto& batch = image_scene.texturedBatches()[0];
    const auto image_key = batch.image.cacheKey();
    Check(batch.premultiplied_alpha && batch.image.pixelColor(0,0)==QColor(127,127,127,0) &&
          batch.image.pixelColor(1,0)==QColor(0,0,0,127),"RViz intensity/alpha blending preserves free and occupied pixels");
    Check((batch.vertices[0].position-QVector3D(8,21,1)).length()<1e-5 &&
          (batch.vertices[1].position-QVector3D(8,20,1)).length()<1e-5,
          "texture axes and nested submap/slice transforms match RViz");
    submap->clear_textures(); submap->mutable_pose()->mutable_position()->set_x(11);
    textures.processMessage(textured);
    Check(Render(textures).texturedBatches()[0].image.cacheKey()==image_key,
          "metadata-only replay preserves the cached texture");
    textures.setPropertyValue("submap_index","8");
    Check(Render(textures).texturedBatches().empty(),"individual submap filter hides other IDs");
    textures.setPropertyValue("submap_index","7");
    textures.setPropertyValue("high_resolution","false"); textures.setPropertyValue("low_resolution","true");
    Check(Render(textures).texturedBatches()[0].image.size()==QSize(1,1),"low-resolution slice can be selected");
    textures.setPropertyValue("high_resolution","true"); textures.setPropertyValue("alpha","0.4");
    Check(Render(textures).texturedBatches().size()==2 &&
          std::abs(Render(textures).texturedBatches()[0].vertices[0].color.x()-.4f)<1e-5,
          "both slices use the global premultiplied opacity");

    Check(textures.propertyValue("tracking_frame", "") == "base_link" &&
          textures.propertyValue("fade_out_distance", "") == "1.0",
          "height fading defaults match RViz");
    automsgs::msgs::geometry_msgs::TransformStamped tracking;
    tracking.mutable_header()->set_frame_id("map"); tracking.set_child_frame_id("base_link");
    tracking.mutable_transform()->mutable_rotation()->set_w(1);
    int tracking_stamp = 10;
    auto set_tracking_height = [&](double z) {
      tracking.mutable_header()->mutable_stamp()->set_sec(tracking_stamp++);
      tracking.mutable_transform()->mutable_translation()->set_z(z);
      context.tf_buffer->setTransform(tracking, "height_fade_test");
    };
    auto check_faded_texture = [&](float expected) {
      const auto scene = Render(textures);
      Check(scene.texturedBatches().size() == (expected > 0.f ? 2u : 0u),
            "height fading applies to both resolutions and omits fully transparent quads");
      for (const auto& slice : scene.texturedBatches())
        for (const auto& vertex : slice.vertices)
          Check((vertex.color - QVector4D(expected, expected, expected, expected)).length() < 1e-5,
                "height fading scales premultiplied RGB and alpha together");
      if (!scene.texturedBatches().empty())
        Check(scene.texturedBatches()[0].image.cacheKey() == image_key,
              "tracking motion reuses texture pixels without decoding or rewriting");
    };
    for (const auto [z, opacity] : {std::pair{1., .4f}, {2., .4f}, {3., .2f},
                                  {4., 0.f}, {-1., .2f}, {-2., 0.f}}) {
      set_tracking_height(z);
      check_faded_texture(opacity);
    }
    textures.setPropertyValue("show_axes", "true"); textures.setPropertyValue("show_ids", "true");
    const auto faded_pose = Render(textures);
    Check(faded_pose.lineVertices().size() == 6 &&
          faded_pose.expandedTexturedBatches(context.view_matrix).size() == 1,
          "fully faded textures retain pose axes and ID labels");
    textures.setPropertyValue("show_axes", "false"); textures.setPropertyValue("show_ids", "false");
    set_tracking_height(3.);
    automsgs::msgs::geometry_msgs::TransformStamped tilted;
    tilted.mutable_header()->set_frame_id("tilted_fixed"); tilted.set_child_frame_id("map");
    tilted.mutable_transform()->mutable_translation()->set_z(100);
    tilted.mutable_transform()->mutable_rotation()->set_x(std::sqrt(.5));
    tilted.mutable_transform()->mutable_rotation()->set_w(std::sqrt(.5));
    context.tf_buffer->setTransform(tilted, "height_fade_test", true);
    context.fixed_frame = "tilted_fixed";
    check_faded_texture(.2f);
    context.fixed_frame = "map";
    textures.setPropertyValue("tracking_frame", "missing_tracking_frame");
    check_faded_texture(.4f);
    Check(textures.status().level == autoviz::display::DisplayStatusLevel::kWarn &&
          textures.status().message.find("Height fade unavailable") != std::string::npos,
          "missing tracking TF warns and preserves the unmodified opacity");
    textures.setPropertyValue("fade_out_distance", "0");
    check_faded_texture(.4f);
    Check(textures.status().level == autoviz::display::DisplayStatusLevel::kOk,
          "zero distance disables fading without requiring tracking TF");
    textures.setPropertyValue("tracking_frame", "map"); textures.setPropertyValue("fade_out_distance", "0.5");
    check_faded_texture(.2f);
    textures.setPropertyValue("tracking_frame", "base_link");
    for (const auto* invalid_distance : {"nan", "inf", "-1", "invalid"}) {
      textures.setPropertyValue("fade_out_distance", invalid_distance);
      check_faded_texture(.2f);
    }
    textures.setPropertyValue("fade_out_distance", "1.0");
    check_faded_texture(.2f);
    Check(textures.status().level == autoviz::display::DisplayStatusLevel::kOk,
          "tracking frame recovery resumes fading");
    auto* invalid = submap->add_textures();
    invalid->set_width(std::numeric_limits<int>::max()); invalid->set_height(2);
    invalid->set_resolution(.5); invalid->mutable_slice_pose()->mutable_orientation()->set_w(1);
    invalid->set_cells("invalid"); submap->set_submap_version(2); submap->set_texture_version(2);
    textures.processMessage(textured);
    Check(Render(textures).texturedBatches()[0].image.cacheKey()==image_key,
          "invalid or oversized replacement cannot destroy the last valid texture");
    submap->clear_textures(); submap->set_submap_version(0); textures.processMessage(textured);
    Check(Render(textures).texturedBatches().empty(),"version rollback clears restarted map textures");

    ConstraintProbe constraints; constraints.setContext(&context);
    automsgs::msgs::visualization_msgs::MarkerArray markers;
    auto* marker = markers.add_markers();
    marker->mutable_header()->set_frame_id("map"); marker->set_ns("Inter constraints, same trajectory");
    marker->set_type(automsgs::msgs::visualization_msgs::Marker::LINE_LIST);
    marker->mutable_scale()->set_x(.025); marker->mutable_pose()->mutable_orientation()->set_w(1);
    marker->mutable_pose()->mutable_position()->set_x(2); marker->mutable_pose()->mutable_position()->set_z(.1);
    marker->mutable_color()->set_a(1);
    for (int n=0;n<2;++n) {
      marker->add_points()->set_x(n);
      auto* c=marker->add_colors(); c->set_r(1); c->set_g(1); c->set_a(1);
    }
    constraints.processMessage(markers);
    autoviz::rendering::SceneOverlay constraint_scene; constraints.onDraw(constraint_scene);
    auto triangles = constraint_scene.expandedFlatTriangles(QMatrix4x4());
    Check(triangles.size()==6,"constraint line renders with a metric width");
    if (triangles.size()==6) {
      Check(triangles[0].color==QVector4D(1,1,0,1) && std::abs(triangles[0].position.z()-.1f)<1e-5,
            "per-point loop-closure yellow and marker pose offset are preserved");
      float min_x=100, max_x=-100;
      for (const auto& v:triangles) { min_x=std::min(min_x,v.position.x()); max_x=std::max(max_x,v.position.x()); }
      Check(std::abs(min_x-2)<1e-5 && std::abs(max_x-3)<1e-5,"constraint endpoints include marker translation");
    }
    constraints.setPropertyValue("namespace/Inter constraints, same trajectory","false");
    autoviz::rendering::SceneOverlay hidden; constraints.onDraw(hidden);
    Check(hidden.expandedFlatTriangles(QMatrix4x4()).empty(),"loop closure category can be toggled independently");
    auto constraints_config=autoviz::common::DisplayFactory::defaultForType("CartographerConstraints");
    Check(constraints_config.channel=="/constraint_list" && autoviz::common::DisplayFactory::create(constraints_config)!=nullptr,
          "constraint display is registered with its native marker topic");
  }
  registry.setNode(nullptr);
  if (!failures) std::puts("PASS Submap topics, RViz textures/axes/IDs, height fading, caches, filters, TF and constraint rendering");
  return failures ? 1 : 0;
}
