// Copyright 2026 The Openbot Authors
#include "autoviz/plugins/displays/submaps/submaps_display.hpp"

#ifdef AUTOVIZ_HAS_CARTOGRAPHER
#include <algorithm>
#include <cmath>
#include <limits>
#include <zlib.h>
#include <set>
#include <stdexcept>
#include <QQuaternion>

#include "autoviz/common/display_property.hpp"
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/plugins/displays/common/primitive_mesh.hpp"
#include "autoviz/plugins/displays/common/transform_utils.hpp"
#include "autoviz/rendering/scene_overlay.hpp"
#include "autoviz/rendering/text_raster_utils.hpp"
#include "autoviz/transform/buffer.hpp"

namespace autoviz::display {
namespace {
namespace proto = autonomy::localization::cartographer::proto;
using Base = ChannelDisplay<proto::SubmapList>;

bool ValidPose(const automsgs::msgs::geometry_msgs::Pose& pose) {
  const auto& p = pose.position();
  const auto& q = pose.orientation();
  for (double v : {p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w()})
    if (!std::isfinite(v)) return false;
  return q.x()*q.x() + q.y()*q.y() + q.z()*q.z() + q.w()*q.w() > 1e-12;
}

QMatrix4x4 PoseMatrix(const automsgs::msgs::geometry_msgs::Pose& pose) {
  QMatrix4x4 matrix;
  matrix.translate(pose.position().x(), pose.position().y(), pose.position().z());
  const auto& q = pose.orientation();
  matrix.rotate(QQuaternion(q.w(), q.x(), q.y(), q.z()).normalized());
  return matrix;
}
QImage DecodeTexture(const proto::SubmapTexture& texture) {
  // Bound decompressed data before allocating or inflating incoming textures.
  constexpr uint64_t kMaxPixels = 16 * 1024 * 1024;
  const int width = texture.width(), height = texture.height();
  if (width <= 0 || height <= 0 || !std::isfinite(texture.resolution()) ||
      texture.resolution() <= 0. || !ValidPose(texture.slice_pose()) ||
      texture.resolution() * std::max(width, height) > std::numeric_limits<float>::max() ||
      static_cast<uint64_t>(width) * height > kMaxPixels ||
      texture.cells().size() > std::numeric_limits<uInt>::max()) return {};
  std::vector<unsigned char> cells(static_cast<size_t>(width) * height * 2);
  z_stream stream{};
  stream.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(texture.cells().data()));
  stream.avail_in = static_cast<uInt>(texture.cells().size());
  stream.next_out = cells.data();
  stream.avail_out = static_cast<uInt>(cells.size());
  if (inflateInit2(&stream, 16 + MAX_WBITS) != Z_OK) return {};
  const int result = inflate(&stream, Z_FINISH);
  const bool valid = result == Z_STREAM_END && stream.total_out == cells.size() &&
                     stream.avail_in == 0;
  inflateEnd(&stream);
  if (!valid) return {};
  QImage image(width, height, QImage::Format_RGBA8888);
  if (image.isNull()) return {};
  for (int y = 0; y < height; ++y) {
    auto* row = image.scanLine(y);
    for (int x = 0; x < width; ++x) {
      const size_t offset = (static_cast<size_t>(y) * width + x) * 2;
      row[4*x] = row[4*x+1] = row[4*x+2] = cells[offset];
      row[4*x+3] = cells[offset+1];
    }
  }
  return image;
}
}  // namespace

SubmapsDisplay::SubmapsDisplay(std::string channel)
    : Base("Submaps", std::move(channel), "autonomy.localization.cartographer.proto.SubmapList") {
  setProperties({});
}
SubmapsDisplay::~SubmapsDisplay() { onDisable(); }

std::vector<common::DisplayPropertySpec> SubmapsDisplay::propertySpecs() const {
  using common::DisplayPropertyKind;
  std::vector<common::DisplayPropertySpec> specs = {{"show_axes", "Submap Pose Axes", "true"},
          {"show_ids", "Submap IDs", "true"},
          {"trajectory_id", "Trajectory ID (-1 = All)", "-1"},
          {"submap_index", "Submap Index (-1 = All)", "-1"},
          {"show_textures", "Submap Textures", "true"},
          {"high_resolution", "High Resolution", "true"},
          {"low_resolution", "Low Resolution", "false"},
          {"tracking_frame", "Tracking Frame", "base_link"},
          {"fade_out_distance", "Fade-out Distance (m, 0 = Off)", "1.0"},
          {"alpha", "Alpha", "1.0"}};
  specs.push_back({"submaps", "Submaps", "", {}, DisplayPropertyKind::kCategory});
  common::DisplayPropertySpec all{"submaps/all_enabled", "All Enabled", "true"};
  all.parent_key = "submaps";
  specs.push_back(all);
  int previous_trajectory = -1;
  for (const auto& [key, submap] : submaps_) {
    const std::string trajectory = "submaps/trajectory/" + std::to_string(key.first);
    const std::string inherited = propertyValue("submaps/all_enabled", "true");
    if (key.first != previous_trajectory) {
      common::DisplayPropertySpec group{trajectory, "Trajectory " + std::to_string(key.first), inherited};
      group.parent_key = "submaps";
      specs.push_back(group);
      previous_trajectory = key.first;
    }
    common::DisplayPropertySpec entry{
        "submaps/visible/" + std::to_string(key.first) + "/" + std::to_string(key.second),
        std::to_string(key.second) + "." + std::to_string(submap.metadata.submap_version()),
        propertyValue(trajectory, inherited)};
    entry.parent_key = trajectory;
    specs.push_back(entry);
  }
  return specs;
}

bool SubmapsDisplay::submapVisible(const Key& key) const {
  return common::ParseBoolProperty(propertyValue(
      "submaps/visible/" + std::to_string(key.first) + "/" + std::to_string(key.second),
      propertyValue("submaps/trajectory/" + std::to_string(key.first),
                    propertyValue("submaps/all_enabled", "true"))), true);
}

void SubmapsDisplay::onPropertyChanged(const std::string& key) {
  if (key.rfind("submaps/", 0) != 0) return;
  auto values = properties();
  const bool only = key == "submaps/only";
  const bool all = key == "submaps/all_enabled";
  const bool trajectory = key.rfind("submaps/trajectory/", 0) == 0;
  std::string selected;
  if (only) {
    selected = "submaps/visible/" + propertyValue(key, "");
    values.erase(key);
    bool found = false;
    for (const auto& [id, submap] : submaps_)
      if (selected == "submaps/visible/" + std::to_string(id.first) + "/" + std::to_string(id.second)) found = true;
    if (!found) { setProperties(values); return; }
    values["submaps/all_enabled"] = "false";
  }
  if (all || only || trajectory) {
    const std::string prefix = trajectory
        ? "submaps/visible/" + key.substr(std::string("submaps/trajectory/").size()) + "/"
        : "submaps/visible/";
    for (auto it = values.begin(); it != values.end();) {
      if (it->first.rfind(prefix, 0) == 0 ||
          ((all || only) && it->first.rfind("submaps/trajectory/", 0) == 0)) it = values.erase(it);
      else ++it;
    }
    if (only) values[selected] = "true";
  }
  values["trajectory_id"] = "-1";
  values["submap_index"] = "-1";
  setProperties(values);
  if (context_) context_->queueRender();
}

void SubmapsDisplay::clearReceivedData() {
  submaps_.clear();
  frame_.clear();
}
void SubmapsDisplay::onDisable() {
  clearReceivedData();
  Base::onDisable();
}

void SubmapsDisplay::processMessage(const proto::SubmapList& message) {
  for (const auto& entry : message.submap()) {
    const auto it = submaps_.find({entry.trajectory_id(), entry.submap_index()});
    if (it != submaps_.end() && entry.submap_version() < it->second.metadata.submap_version()) {
      clearReceivedData();
      break;
    }
  }
  frame_ = message.header().frame_id();
  std::set<Key> listed;
  for (const auto& entry : message.submap()) {
    if (entry.trajectory_id() < 0 || entry.submap_index() < 0 || !ValidPose(entry.pose())) continue;
    const Key key{entry.trajectory_id(), entry.submap_index()};
    listed.insert(key);
    auto& submap = submaps_[key];
    submap.metadata = entry;
    submap.metadata.clear_textures();
    if (!entry.textures().empty() && entry.texture_version() >= 0 &&
        (submap.slices.empty() || submap.texture_version != entry.texture_version())) {
      std::vector<Slice> slices;
      for (int i = 0; i < std::min(2, entry.textures_size()); ++i) {
        const auto& texture = entry.textures(i);
        QImage image = DecodeTexture(texture);
        if (image.isNull()) { slices.clear(); break; }
        slices.push_back({std::move(image), PoseMatrix(texture.slice_pose()),
                          static_cast<float>(texture.width() * texture.resolution()),
                          static_cast<float>(texture.height() * texture.resolution())});
      }
      if (!slices.empty()) {
        submap.slices = std::move(slices);
        submap.texture_version = entry.texture_version();
      }
    }
  }
  for (auto it = submaps_.begin(); it != submaps_.end();) {
    if (!listed.count(it->first)) it = submaps_.erase(it);
    else ++it;
  }
}

void SubmapsDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (!context_ || submaps_.empty()) return;
  QMatrix4x4 frame_to_fixed;
  if (!frame_.empty() && frame_ != context_->fixed_frame) {
    try {
      if (!context_->tf_buffer) throw std::runtime_error("No TF buffer");
      frame_to_fixed = transformToMatrix(context_->tf_buffer->lookupTransform(
          context_->fixed_frame, frame_, commsgs::ZeroTime()));
    } catch (...) {
      setStatusWarn("TF missing: " + frame_ + " -> " + context_->fixed_frame);
      return;
    }
  }
  const int selected = common::ParseIntProperty(propertyValue("trajectory_id", "-1"), -1);
  const bool show_axes = common::ParseBoolProperty(propertyValue("show_axes", "true"), true);
  const bool show_ids = common::ParseBoolProperty(propertyValue("show_ids", "true"), true);
  const int selected_submap = common::ParseIntProperty(propertyValue("submap_index", "-1"), -1);
  const bool show_textures = common::ParseBoolProperty(propertyValue("show_textures", "true"), true);
  const float alpha = std::clamp(common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f), 0.f, 1.f);
  const float parsed_distance = common::ParseFloatProperty(propertyValue("fade_out_distance", "1.0"), 1.f);
  const double fade_distance = std::isfinite(parsed_distance) && parsed_distance >= 0.f ? parsed_distance : 1.;
  bool can_fade = false;
  double tracking_z = 0.;
  std::string fade_warning;
  if (show_textures && alpha > 0.f && fade_distance > 0.) {
    const std::string tracking_frame = propertyValue("tracking_frame", "base_link");
    const std::string map_frame = frame_.empty() ? context_->fixed_frame : frame_;
    try {
      if (tracking_frame != map_frame) {
        if (!context_->tf_buffer) throw std::runtime_error("No TF buffer");
        tracking_z = context_->tf_buffer->lookupTransform(
            map_frame, tracking_frame, commsgs::ZeroTime()).transform().translation().z();
      }
      if (!std::isfinite(tracking_z)) throw std::runtime_error("Invalid tracking height");
      can_fade = true;
    } catch (...) {
      fade_warning = "Height fade unavailable: " + tracking_frame + " -> " + map_frame;
    }
  }
  int selected_count = 0, loaded = 0;
  static const ObjMesh axis_mesh = buildCylinderMesh(.06f, .3f);
  const QVector3D directions[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  const QColor colors[] = {Qt::red, Qt::green, Qt::blue};
  const QVector3D camera_up = context_->view_matrix.inverted().column(1).toVector3D().normalized();
  for (auto& item : submaps_) {
    if (!submapVisible(item.first)) continue;
    if (selected >= 0 && item.first.first != selected) continue;
    if (selected_submap >= 0 && item.first.second != selected_submap) continue;
    ++selected_count;
    auto& submap = item.second;
    const QMatrix4x4 pose = frame_to_fixed * PoseMatrix(submap.metadata.pose());
    if (!submap.slices.empty() && submap.texture_version == submap.metadata.submap_version()) ++loaded;
    float submap_alpha = alpha;
    if (can_fade) {
      // RViz compares heights in the map frame, fading from distance d to 3d.
      const double dz = std::abs(submap.metadata.pose().position().z() - tracking_z);
      submap_alpha *= std::clamp(1. - std::max(dz - fade_distance, 0.) / (2. * fade_distance), 0., 1.);
    }
    if (show_textures && submap_alpha > 0.f) {
      for (size_t i = 0; i < submap.slices.size(); ++i) {
        if (!common::ParseBoolProperty(propertyValue(i == 0 ? "high_resolution" : "low_resolution", i == 0 ? "true" : "false"), i == 0)) continue;
        const auto& slice = submap.slices[i];
        const auto transform = pose * slice.pose;
        scene.addTexturedQuad(transform.map(QVector3D()), transform.map({0, -slice.width, 0}),
            transform.map({-slice.height, -slice.width, 0}), transform.map({-slice.height, 0, 0}),
            slice.image, rendering::SceneOverlay::TextureFilterMode::kNearest,
            0, false, true, QVector4D(submap_alpha, submap_alpha, submap_alpha, submap_alpha), true);
      }
    }
    if (show_axes) {
      for (int i = 0; i < 3; ++i) {
        scene.addLine(pose.map(QVector3D()), pose.map(.3f * directions[i]), colors[i]);
        QMatrix4x4 axis = pose;
        axis.translate(.15f * directions[i]);
        axis.rotate(QQuaternion::rotationTo(QVector3D(0, 0, 1), directions[i]));
        scene.addTriangleMeshSolid(axis_mesh, axis, colors[i]);
      }
    }
    if (show_ids) {
      if (submap.label.isNull()) submap.label = rendering::RasterizeTextLabel(
          QString("(%1,%2)").arg(item.first.first).arg(item.first.second), Qt::red, 24);
      // Match RViz's 0.2 m text, above-centred at submap-local Z=0.3 m.
      constexpr float half_height = .1f;
      scene.addViewFacingTexturedQuad(pose.map({0, 0, .3f}) + half_height * camera_up,
          half_height * submap.label.width() / submap.label.height(), half_height, submap.label);
    }
  }
  if (show_textures && loaded < selected_count)
    setStatusWarn("Waiting for submap textures: " + std::to_string(loaded) + "/" + std::to_string(selected_count));
  else if (!fade_warning.empty()) setStatusWarn(fade_warning);
  else setStatusOk("Submaps ready");
}
}  // namespace autoviz::display
#endif
