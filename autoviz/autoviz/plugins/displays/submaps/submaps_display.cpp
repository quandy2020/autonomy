// Copyright 2026 The Openbot Authors
#include "autoviz/plugins/displays/submaps/submaps_display.hpp"

#ifdef AUTOVIZ_HAS_CARTOGRAPHER
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <QQuaternion>
#include <zlib.h>

#include "autoviz/common/display_property.hpp"
#include "autoviz/commsgs/time_utils.hpp"
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
  // Bound decompressed data before allocating or inflating untrusted RPC input.
  constexpr uint64_t kMaxPixels = 16 * 1024 * 1024;
  const int width = texture.width(), height = texture.height();
  if (width <= 0 || height <= 0 || !std::isfinite(texture.resolution()) ||
      texture.resolution() <= 0. || !ValidPose(texture.slice_pose()) ||
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
  return {{"submap_query_service", "Submap Query Service", "/submap_query"},
          {"tracking_frame", "Tracking Frame", "base_link"},
          {"high_resolution", "High Resolution", "true"},
          {"low_resolution", "Low Resolution", "false"},
          {"show_axes", "Submap Pose Axes", "true"},
          {"show_ids", "Submap IDs", "true"},
          {"trajectory_id", "Trajectory ID (-1 = All)", "-1"},
          {"alpha", "Alpha", "1.0"},
          {"fade_distance", "Fade-out Distance (m)", "1.0"}};
}

void SubmapsDisplay::resetClient() {
  if (client_) client_->Destroy();
  client_.reset();
  pending_.clear();
}
void SubmapsDisplay::clearReceivedData() {
  resetClient();
  submaps_.clear();
  frame_.clear();
}
void SubmapsDisplay::onDisable() {
  clearReceivedData();
  Base::onDisable();
}
void SubmapsDisplay::onPropertyChanged(const std::string& key) {
  if (key == "submap_query_service") resetClient();
}
bool SubmapsDisplay::visible(int trajectory) const {
  const int selected = common::ParseIntProperty(propertyValue("trajectory_id", "-1"), -1);
  return selected < 0 || trajectory == selected;
}

void SubmapsDisplay::processMessage(const proto::SubmapList& message) {
  for (const auto& entry : message.submap()) {
    const auto it = submaps_.find({entry.trajectory_id(), entry.submap_index()});
    if (it != submaps_.end() && entry.submap_version() < it->second.metadata.submap_version()) {
      clearReceivedData();  // A lower version indicates a restarted Cartographer.
      break;
    }
  }
  frame_ = message.header().frame_id();
  std::set<Key> listed;
  for (const auto& entry : message.submap()) {
    if (entry.trajectory_id() < 0 || entry.submap_index() < 0 || entry.submap_version() < 0 || !ValidPose(entry.pose())) continue;
    const Key key{entry.trajectory_id(), entry.submap_index()};
    listed.insert(key);
    submaps_[key].metadata = entry;
  }
  bool removed = false;
  for (auto it = submaps_.begin(); it != submaps_.end();) {
    if (!listed.count(it->first)) { it = submaps_.erase(it); removed = true; }
    else ++it;
  }
  if (removed) resetClient();
}

bool SubmapsDisplay::decodeTextures(const proto::SubmapQueryResponse& response, Submap* submap) {
  if (response.status().code() != proto::OK || response.textures().empty()) return false;
  std::vector<Slice> slices;
  // Cartographer provides at most a high- and a low-resolution slice.
  for (int i = 0; i < std::min(2, response.textures_size()); ++i) {
    const auto& texture = response.textures(i);
    Slice slice;
    slice.image = DecodeTexture(texture);
    if (slice.image.isNull()) return false;
    slice.pose = PoseMatrix(texture.slice_pose());
    slice.width = texture.width() * texture.resolution();
    slice.height = texture.height() * texture.resolution();
    slices.push_back(std::move(slice));
  }
  submap->slices = std::move(slices);
  submap->texture_version = response.submap_version();
  return true;
}

void SubmapsDisplay::onUpdate() {
  Base::onUpdate();
  if (submaps_.empty() || !context_ || !context_->autolink || !context_->autolink->node()) return;
  const auto now = Clock::now();
  for (const auto& request : pending_) {
    if (now - request.second.started > std::chrono::seconds(3)) {
      // Retire unanswered requests together; the client has no per-request cancellation.
      client_->Destroy();
      client_->Init();
      pending_.clear();
      setStatusWarn("Submap query timed out; retrying");
      return;
    }
  }
  for (auto it = pending_.begin(); it != pending_.end(); ++it) {
    if (it->second.future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) continue;
    const auto found = submaps_.find(it->first);
    try {
      const auto response = it->second.future.get();
      if (found != submaps_.end() && (!response || !decodeTextures(*response, &found->second)))
        setStatusWarn("Invalid or unavailable submap texture; retrying");
    } catch (const std::future_error&) {
      setStatusWarn("Submap request cancelled; retrying");
    }
    pending_.erase(it);
    break;  // Decode at most one response per UI tick.
  }
  if (!client_) {
    client_ = context_->autolink->node()->CreateClient<proto::SubmapQueryRequest, proto::SubmapQueryResponse>(
        propertyValue("submap_query_service", "/submap_query"));
  }
  if (!client_ || !client_->ServiceIsReady()) {
    setStatusWarn("Waiting for submap query service");
    return;
  }
  for (auto it = submaps_.rbegin(); it != submaps_.rend() && pending_.size() < 6; ++it) {
    auto& submap = it->second;
    if (!visible(it->first.first) || pending_.count(it->first) ||
        submap.texture_version == submap.metadata.submap_version() || now < submap.next_query) continue;
    auto request = std::make_shared<proto::SubmapQueryRequest>();
    request->set_trajectory_id(it->first.first);
    request->set_submap_index(it->first.second);
    auto future = client_->AsyncSendRequest(request);
    if (future.valid()) pending_.emplace(it->first, Pending{std::move(future), now});
    submap.next_query = now + std::chrono::milliseconds(500);
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
  float tracking_z = 0.f;
  bool have_tracking = false;
  const std::string tracking = propertyValue("tracking_frame", "base_link");
  const std::string map_frame = frame_.empty() ? context_->fixed_frame : frame_;
  if (tracking == map_frame) have_tracking = true;
  else if (context_->tf_buffer) {
    try {
      tracking_z = context_->tf_buffer->lookupTransform(map_frame, tracking, commsgs::ZeroTime()).transform().translation().z();
      have_tracking = true;
    } catch (...) {}
  }
  const float fade_distance = std::max(0.f, common::ParseFloatProperty(propertyValue("fade_distance", "1.0"), 1.f));
  const float opacity = std::clamp(common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f), 0.f, 1.f);
  const bool show_axes = common::ParseBoolProperty(propertyValue("show_axes", "true"), true);
  const bool show_ids = common::ParseBoolProperty(propertyValue("show_ids", "true"), true);
  int loaded = 0, selected = 0;
  for (auto& item : submaps_) {
    if (!visible(item.first.first)) continue;
    ++selected;
    auto& submap = item.second;
    const QMatrix4x4 pose = frame_to_fixed * PoseMatrix(submap.metadata.pose());
    const QVector3D origin = pose.map(QVector3D());
    float alpha = opacity;
    if (have_tracking && fade_distance > 0.f) {
      const float distance = std::abs(submap.metadata.pose().position().z() - tracking_z);
      alpha *= std::clamp(1.f - std::max(0.f, distance - fade_distance) / (2.f * fade_distance), 0.f, 1.f);
    }
    if (!submap.slices.empty()) ++loaded;
    for (size_t i = 0; i < submap.slices.size(); ++i) {
      if (!common::ParseBoolProperty(propertyValue(i == 0 ? "high_resolution" : "low_resolution", i == 0 ? "true" : "false"), i == 0)) continue;
      if (alpha <= 0.f) continue;
      const auto& slice = submap.slices[i];
      const auto transform = pose * slice.pose;
      scene.addTexturedQuad(transform.map(QVector3D()), transform.map({0, -slice.width, 0}),
          transform.map({-slice.height, -slice.width, 0}), transform.map({-slice.height, 0, 0}),
          slice.image, rendering::SceneOverlay::TextureFilterMode::kNearest,
          1, false, true, QVector4D(alpha, alpha, alpha, alpha), true);
    }
    if (show_axes) {
      scene.addLine(origin, pose.map({.3f, 0, 0}), Qt::red);
      scene.addLine(origin, pose.map({0, .3f, 0}), Qt::green);
      scene.addLine(origin, pose.map({0, 0, .3f}), Qt::blue);
    }
    if (show_ids) {
      if (submap.label.isNull()) submap.label = rendering::RasterizeTextLabel(
          QString("(%1,%2)").arg(item.first.first).arg(item.first.second), Qt::white, 24);
      const float half_height = .1f;
      scene.addViewFacingTexturedQuad(origin + QVector3D(0, 0, .2f),
          half_height * submap.label.width() / submap.label.height(), half_height, submap.label);
    }
  }
  if (!have_tracking && fade_distance > 0.f) setStatusWarn("Tracking TF missing; height fading unavailable");
  else if (loaded < selected) setStatusWarn("Loading submap textures");
  else setStatusOk("Submaps ready");
}
}  // namespace autoviz::display
#endif
