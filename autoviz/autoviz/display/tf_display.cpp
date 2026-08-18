/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/tf_display.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_set>
#include <vector>

#include <QColor>
#include <QString>

#include "autoviz/common/display_property.hpp"
#include "autoviz/display/arrow_mesh_utils.hpp"
#include "autoviz/display/primitive_mesh.hpp"
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/display/tf_display_utils.hpp"
#include "autoviz/rendering/text_raster_utils.hpp"
#include "autoviz/transform/buffer.hpp"
#include "autoviz/transform/buffer_utils.hpp"

namespace autoviz {
namespace display {
namespace {

double WallSec() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

int64_t WallNs() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

const ObjMesh& UnitTfHubMesh() {
  static const ObjMesh mesh = buildSphereMesh(0.5f);
  return mesh;
}

void DrawTfSegment(rendering::SceneOverlay& scene, const QVector3D& a,
                   const QVector3D& b, const QColor& color, float width,
                   bool for_pick = false) {
  scene.addLine(a, b, color, for_pick);
  if (width > 1e-4f) {
    scene.addViewFacingPolylineStrip({a, b}, width, color);
  }
}

void DrawSolidTfAxis(rendering::SceneOverlay& scene, const QVector3D& origin,
                     const QVector3D& end, const QColor& color,
                     float shaft_diameter, float head_diameter) {
  const QVector3D delta = end - origin;
  const float axis_len = delta.length();
  if (axis_len < 1e-5f) {
    return;
  }
  const QVector3D direction = delta / axis_len;
  const float hub_radius = std::max(shaft_diameter * 1.2f, axis_len * 0.06f);
  const QVector3D shaft_start = origin + direction * (hub_radius * 0.75f);
  std::vector<ColoredMeshInstance> meshes;
  appendSolidArrowMeshes(&meshes, shaft_start, end, color, 0.18f, shaft_diameter,
                         head_diameter);
  for (const auto& mesh : meshes) {
    scene.addTriangleMeshSolid(mesh.mesh, mesh.transform, mesh.color);
  }
}

}  // namespace

TfDisplay::TfDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::tf2_msgs::TFMessage>(
          "TF", std::move(channel), "automsgs.msgs.tf2_msgs.TFMessage") {
  setProperties({});
  refreshCachedProps();
}

TfDisplay::~TfDisplay() {
  transforms_changed_connection_.disconnect();
  transforms_listener_attached_ = false;
}

void TfDisplay::reset() {
  ChannelDisplay::reset();
  frames_.clear();
  frames_dirty_ = true;
  tf_messages_received_ = 0;
  last_msg_wall_ns_ = 0;
}

std::vector<common::DisplayPropertySpec> TfDisplay::propertySpecs() const {
  return {
      {"show_names", "Show Names", "false"},
      {"show_axes", "Show Axes", "true"},
      {"show_arrows", "Show Arrows", "true"},
      {"marker_scale", "Marker Scale", "1.0"},
      {"update_interval", "Update Interval", "0"},
      {"frame_timeout", "Frame Timeout", "15.0"},
      {"filter_whitelist", "Filter (whitelist)", "", {},
       common::DisplayPropertyKind::kRegex},
      {"filter_blacklist", "Filter (blacklist)", "", {},
       common::DisplayPropertyKind::kRegex},
  };
}

std::vector<TfFrameSnapshot> TfDisplay::frameSnapshots() const {
  std::vector<TfFrameSnapshot> out;
  out.reserve(frames_.size());
  for (const auto& entry : frames_) {
    const FrameInfo& info = entry.second;
    TfFrameSnapshot snap;
    snap.name = info.name;
    snap.parent = info.parent;
    snap.enabled = info.enabled;
    snap.position = info.position;
    snap.orientation = info.orientation;
    snap.rel_position = info.rel_position;
    snap.rel_orientation = info.rel_orientation;
    snap.have_fixed_pose = info.have_fixed_pose;
    out.push_back(std::move(snap));
  }
  return out;
}

std::vector<std::pair<std::string, std::string>> TfDisplay::treeEdges() const {
  std::vector<std::pair<std::string, std::string>> edges;
  edges.reserve(frames_.size());
  for (const auto& entry : frames_) {
    edges.emplace_back(entry.first, entry.second.parent);
  }
  return edges;
}

void TfDisplay::markFramesDirty() { frames_dirty_ = true; }

void TfDisplay::ensureTransformsListener() {
  if (transforms_listener_attached_ || context_ == nullptr ||
      context_->tf_buffer == nullptr) {
    return;
  }
  transforms_changed_connection_ =
      context_->tf_buffer->_addTransformsChangedListener(
          [this]() { markFramesDirty(); });
  transforms_listener_attached_ = true;
}

void TfDisplay::onEnable() {
  ChannelDisplay::onEnable();
  frames_dirty_ = true;
  ensureTransformsListener();
}

void TfDisplay::onDisable() {
  transforms_changed_connection_.disconnect();
  transforms_listener_attached_ = false;
  ChannelDisplay::onDisable();
}

void TfDisplay::refreshCachedProps() {
  props_.show_names =
      common::ParseBoolProperty(propertyValue("show_names", "false"), false);
  props_.show_axes =
      common::ParseBoolProperty(propertyValue("show_axes", "true"), true);
  props_.show_arrows =
      common::ParseBoolProperty(propertyValue("show_arrows", "true"), true);
  props_.marker_scale = std::max(
      1e-3f,
      common::ParseFloatProperty(propertyValue("marker_scale", "1.0"), 1.f));
  props_.update_interval =
      common::ParseFloatProperty(propertyValue("update_interval", "0"), 0.f);
  props_.frame_timeout =
      common::ParseFloatProperty(propertyValue("frame_timeout", "15.0"), 15.f);
  props_.filter_whitelist = propertyValue("filter_whitelist", "");
  props_.filter_blacklist = propertyValue("filter_blacklist", "");
}

void TfDisplay::setAllFramesEnabled(bool enabled) {
  if (changing_single_frame_) {
    return;
  }
  all_enabled_ = enabled;
  for (auto& entry : frames_) {
    entry.second.enabled = enabled;
    frame_enabled_from_config_[entry.second.name] = enabled;
  }
  persistEnabledIntoProperties();
  frames_dirty_ = true;
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void TfDisplay::setFrameEnabled(const std::string& frame, bool enabled) {
  changing_single_frame_ = true;
  frame_enabled_from_config_[frame] = enabled;
  auto it = frames_.find(frame);
  if (it != frames_.end()) {
    it->second.enabled = enabled;
  }
  persistEnabledIntoProperties();
  changing_single_frame_ = false;
  frames_dirty_ = true;
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void TfDisplay::applyEnabledFromConfig(FrameInfo* info) {
  if (info == nullptr) {
    return;
  }
  const auto it = frame_enabled_from_config_.find(info->name);
  if (it != frame_enabled_from_config_.end()) {
    info->enabled = it->second;
  } else {
    info->enabled = all_enabled_;
    frame_enabled_from_config_[info->name] = all_enabled_;
  }
}

void TfDisplay::persistEnabledIntoProperties() {
  common::DisplayPropertyMap props = properties();
  props["all_enabled"] = all_enabled_ ? "true" : "false";
  for (auto it = props.begin(); it != props.end();) {
    if (it->first.rfind("frame_enabled.", 0) == 0) {
      it = props.erase(it);
    } else {
      ++it;
    }
  }
  for (const auto& entry : frame_enabled_from_config_) {
    props["frame_enabled." + entry.first] = entry.second ? "true" : "false";
  }
  setProperties(props);
}

void TfDisplay::noteFrameTransformTime(FrameInfo* info, int64_t latest_time_ns) {
  if (info == nullptr) {
    return;
  }
  // RViz: reset wall last_update when latest common time changes, or is zero.
  if (latest_time_ns != info->last_time_to_fixed_ns || latest_time_ns == 0) {
    info->last_update_ns = WallNs();
    info->last_time_to_fixed_ns = latest_time_ns;
  }
}

void TfDisplay::setOkStatus() {
  setStatusOk("Showing " + std::to_string(frames_.size()) + " frames, drew " +
              std::to_string(last_drew_frames_) + ", msgs " +
              std::to_string(tf_messages_received_));
}

void TfDisplay::load(const common::Config& config) {
  Display::load(config);
  all_enabled_ =
      common::ParseBoolProperty(propertyValue("all_enabled", "true"), true);
  frame_enabled_from_config_.clear();
  for (const auto& prop : properties()) {
    static constexpr char kPrefix[] = "frame_enabled.";
    if (prop.first.rfind(kPrefix, 0) != 0) {
      continue;
    }
    frame_enabled_from_config_[prop.first.substr(sizeof(kPrefix) - 1)] =
        common::ParseBoolProperty(prop.second, true);
  }
  refreshCachedProps();
  frames_dirty_ = true;
}

void TfDisplay::save(common::Config config) const {
  const_cast<TfDisplay*>(this)->persistEnabledIntoProperties();
  Display::save(config);
}

void TfDisplay::saveToConfig(common::DisplayConfig* config) const {
  const_cast<TfDisplay*>(this)->persistEnabledIntoProperties();
  Display::saveToConfig(config);
}

void TfDisplay::onPropertyChanged(const std::string& key) {
  refreshCachedProps();
  if (key == "filter_whitelist" || key == "filter_blacklist" ||
      key == "frame_timeout" || key == "update_interval" ||
      key == "marker_scale" || key.rfind("show_", 0) == 0) {
    update_timer_sec_ = 1e9;
    frames_dirty_ = true;
  }
  if (key == "all_enabled") {
    setAllFramesEnabled(
        common::ParseBoolProperty(propertyValue("all_enabled", "true"), true));
  }
}

void TfDisplay::processMessage(
    const automsgs::msgs::tf2_msgs::TFMessage& message) {
  if (context_ == nullptr || context_->tf_buffer == nullptr) {
    return;
  }
  ensureTransformsListener();
  // `/tf` and `/tf_static` are already ingested by transform::Listener.
  // Re-inserting them here as dynamic duplicates cache entries and can
  // promote static URDF joints onto a TimeCache, which desynchronizes
  // rigid links during yaw.
  const std::string& ch = channel();
  const bool already_in_buffer = (ch == "/tf" || ch == "tf" ||
                                  ch == "/tf_static" || ch == "tf_static");
  if (!already_in_buffer) {
    autoviz::transform::ApplyTfMessageToBuffer(context_->tf_buffer, message,
                                                "autoviz");
  }
  last_msg_wall_ns_ = WallNs();
  ++tf_messages_received_;
  // Channel activity keeps frames "fresh" for Frame Timeout (RViz-like).
  for (auto& entry : frames_) {
    entry.second.last_update_ns = last_msg_wall_ns_;
  }
  frames_dirty_ = true;
}

void TfDisplay::onUpdate() {
  ChannelDisplay::onUpdate();  // may set "No messages received"

  ensureTransformsListener();

  if (context_ != nullptr && context_->fixed_frame != fixed_frame_cached_) {
    fixed_frame_cached_ = context_->fixed_frame;
    frames_dirty_ = true;
  }

  const double now = WallSec();
  if (last_wall_sec_ <= 0.0) {
    last_wall_sec_ = now;
  }
  update_timer_sec_ += std::max(0.0, now - last_wall_sec_);
  last_wall_sec_ = now;

  const float interval = props_.update_interval;
  const bool interval_elapsed =
      interval < 1e-4f || update_timer_sec_ >= static_cast<double>(interval);

  if (!interval_elapsed) {
    if (!frames_.empty()) {
      setOkStatus();
    }
    return;
  }

  update_timer_sec_ = 0.0;

  // Hot path: with Update Interval=0, skip expensive buffer scans / lookups
  // when nothing in the TF buffer changed since the last updateFrames().
  if (!frames_dirty_ && !frames_.empty()) {
    setOkStatus();
    return;
  }

  updateFrames();
  frames_dirty_ = false;
}

void TfDisplay::updateFrames() {
  if (context_ == nullptr || context_->tf_buffer == nullptr) {
    setStatusWarn("TF buffer not ready");
    return;
  }

  all_enabled_ =
      common::ParseBoolProperty(propertyValue("all_enabled", "true"), true);
  for (const auto& prop : properties()) {
    static constexpr char kPrefix[] = "frame_enabled.";
    if (prop.first.rfind(kPrefix, 0) != 0) {
      continue;
    }
    frame_enabled_from_config_[prop.first.substr(sizeof(kPrefix) - 1)] =
        common::ParseBoolProperty(prop.second, true);
  }

  const auto stats = context_->tf_buffer->frameStats();
  std::vector<std::string> names;
  names.reserve(stats.size());
  std::map<std::string, transform::TfFrameStats> by_name;
  for (const auto& s : stats) {
    names.push_back(s.frame_id);
    by_name.emplace(s.frame_id, s);
  }

  filter_error_.clear();
  names = FilterTfFrameNames(names, props_.filter_whitelist,
                             props_.filter_blacklist, &filter_error_);

  std::unordered_set<std::string> keep;
  keep.reserve(names.size());
  const auto zero_time = autoviz::commsgs::ZeroTime();
  int posed = 0;

  // Pass 1: fixed-frame poses only (relative derived in pass 2 — no 2nd lookup).
  for (const std::string& name : names) {
    keep.insert(name);
    FrameInfo& info = frames_[name];
    info.name = name;
    applyEnabledFromConfig(&info);

    int64_t latest_time_ns = 0;
    const auto st = by_name.find(name);
    if (st != by_name.end()) {
      info.parent = st->second.parent_id;
      latest_time_ns = st->second.last_stamp_ns;
    } else {
      std::string parent;
      if (context_->tf_buffer->_getParent(name, 0, parent)) {
        info.parent = parent;
      }
    }

    info.have_fixed_pose = false;
    try {
      const auto tf = context_->tf_buffer->lookupTransform(
          context_->fixed_frame, name, zero_time);
      info.position =
          QVector3D(static_cast<float>(tf.transform().translation().x()),
                    static_cast<float>(tf.transform().translation().y()),
                    static_cast<float>(tf.transform().translation().z()));
      const auto& q = tf.transform().rotation();
      info.orientation =
          QQuaternion(static_cast<float>(q.w()), static_cast<float>(q.x()),
                      static_cast<float>(q.y()), static_cast<float>(q.z()));
      info.have_fixed_pose = true;
      ++posed;
      const int64_t stamp_ns = static_cast<int64_t>(
          autoviz::commsgs::TimeToNanoseconds(tf.header().stamp()));
      if (stamp_ns > 0) {
        latest_time_ns = stamp_ns;
      }
    } catch (...) {
      info.have_fixed_pose = false;
    }

    noteFrameTransformTime(&info, latest_time_ns);
  }

  // Pass 2: relative pose from cached fixed poses (UI Frames panel).
  for (const std::string& name : names) {
    FrameInfo& info = frames_[name];
    if (!info.have_fixed_pose || info.parent.empty()) {
      continue;
    }
    const auto parent_it = frames_.find(info.parent);
    if (parent_it == frames_.end() || !parent_it->second.have_fixed_pose) {
      continue;
    }
    const FrameInfo& parent = parent_it->second;
    const QQuaternion inv_parent = parent.orientation.conjugated();
    info.rel_position =
        inv_parent.rotatedVector(info.position - parent.position);
    info.rel_orientation = inv_parent * info.orientation;
  }

  for (auto it = frames_.begin(); it != frames_.end();) {
    if (keep.find(it->first) == keep.end()) {
      it = frames_.erase(it);
    } else {
      ++it;
    }
  }

  last_posed_frames_ = posed;

  if (!filter_error_.empty()) {
    setStatusWarn(filter_error_);
  } else if (stats.empty()) {
    setStatusWarn("No frames in TF buffer");
  } else if (names.empty()) {
    setStatusWarn("No frames match filters");
  } else if (posed == 0) {
    setStatusWarn("No transforms to Fixed Frame '" + context_->fixed_frame +
                  "'");
  } else {
    setOkStatus();
  }
}

void TfDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (context_ == nullptr || context_->tf_buffer == nullptr) {
    last_drew_frames_ = 0;
    return;
  }
  if (!props_.show_axes && !props_.show_arrows && !props_.show_names) {
    last_drew_frames_ = 0;
    return;
  }
  const float scale = props_.marker_scale;
  const double timeout = props_.frame_timeout;
  const float axis_len = kTfDefaultAxisLength * scale;
  const float axis_shaft_diameter = std::clamp(0.022f * scale, 0.008f, 0.05f);
  const float axis_head_diameter = std::clamp(axis_shaft_diameter * 1.8f,
                                              0.016f, 0.09f);
  const float arrow_width = std::clamp(0.022f * scale, 0.008f, 0.06f);
  const int64_t now_ns = WallNs();

  int drew = 0;
  for (auto& entry : frames_) {
    FrameInfo& info = entry.second;
    if (!info.enabled || !info.have_fixed_pose) {
      continue;
    }

    const double age =
        info.last_update_ns > 0
            ? std::max(0.0, (static_cast<double>(now_ns) -
                             static_cast<double>(info.last_update_ns)) *
                                1e-9)
            : 0.0;
    const double age_clamped =
        timeout > 0.0 ? std::min(age, timeout * 0.99) : age;
    const TfAgeVisual age_r =
        TfAgeVisualForTimeout(age_clamped, timeout, QColor(220, 60, 60));
    const TfAgeVisual age_g =
        TfAgeVisualForTimeout(age_clamped, timeout, QColor(60, 220, 60));
    const TfAgeVisual age_b =
        TfAgeVisualForTimeout(age_clamped, timeout, QColor(60, 120, 220));
    const TfAgeVisual age_arrow =
        TfAgeVisualForTimeout(age_clamped, timeout, QColor(255, 255, 85));
    const TfAgeVisual age_name =
        TfAgeVisualForTimeout(age_clamped, timeout, QColor(230, 230, 230));

    const QVector3D& origin = info.position;
    const QQuaternion& ori = info.orientation;
    // TF overlay is not click-selectable (design); skip pick sample spam.
    constexpr bool kPick = false;

    if (props_.show_axes) {
      if (age_r.visible || age_g.visible || age_b.visible) {
        QMatrix4x4 hub;
        hub.setToIdentity();
        hub.translate(origin);
        const float hub_radius =
            std::max(axis_shaft_diameter * 1.2f, axis_len * 0.06f);
        hub.scale(hub_radius * 2.f, hub_radius * 2.f, hub_radius * 2.f);
        scene.addTriangleMeshSolid(UnitTfHubMesh(), hub, QColor(185, 185, 185));
      }
      if (age_r.visible) {
        DrawSolidTfAxis(
            scene, origin,
            origin + ori.rotatedVector(QVector3D(axis_len, 0.f, 0.f)),
            age_r.color, axis_shaft_diameter, axis_head_diameter);
      }
      if (age_g.visible) {
        DrawSolidTfAxis(
            scene, origin,
            origin + ori.rotatedVector(QVector3D(0.f, axis_len, 0.f)),
            age_g.color, axis_shaft_diameter, axis_head_diameter);
      }
      if (age_b.visible) {
        DrawSolidTfAxis(
            scene, origin,
            origin + ori.rotatedVector(QVector3D(0.f, 0.f, axis_len)),
            age_b.color, axis_shaft_diameter, axis_head_diameter);
      }
    }

    if (props_.show_arrows && !info.parent.empty()) {
      const auto parent_it = frames_.find(info.parent);
      if (age_arrow.visible && parent_it != frames_.end() &&
          parent_it->second.have_fixed_pose) {
        const QVector3D& parent_origin = parent_it->second.position;
        const QVector3D delta = parent_origin - origin;
        const float len = delta.length();
        if (len > 1e-4f) {
          DrawTfSegment(scene, origin, parent_origin, age_arrow.color,
                        arrow_width, kPick);
          const QVector3D dir = delta / len;
          QVector3D side = QVector3D::crossProduct(dir, QVector3D(0.f, 0.f, 1.f));
          if (side.lengthSquared() < 1e-6f) {
            side = QVector3D::crossProduct(dir, QVector3D(0.f, 1.f, 0.f));
          }
          side.normalize();
          const float head = std::min(0.15f * len, 0.12f);
          const QVector3D base = parent_origin - dir * head;
          DrawTfSegment(scene, parent_origin, base + side * (head * 0.35f),
                        age_arrow.color, arrow_width, kPick);
          DrawTfSegment(scene, parent_origin, base - side * (head * 0.35f),
                        age_arrow.color, arrow_width, kPick);
        }
      }
    }

    if (props_.show_names) {
      const float char_height = 0.1f * scale;
      const int pixel_height =
          static_cast<int>(std::clamp(char_height * 120.f, 16.f, 128.f));
      const QRgb rgba = age_name.color.rgba();
      if (info.name_label.isNull() || info.name_label_rgba != rgba ||
          info.name_label_pixel_height != pixel_height) {
        info.name_label = rendering::RasterizeTextLabel(
            QString::fromStdString(info.name), age_name.color, pixel_height);
        info.name_label_rgba = rgba;
        info.name_label_pixel_height = pixel_height;
      }
      if (!info.name_label.isNull()) {
        const float aspect =
            static_cast<float>(info.name_label.width()) /
            static_cast<float>(std::max(1, info.name_label.height()));
        const float half_height = char_height * 0.5f;
        scene.addViewFacingTexturedQuad(
            origin + QVector3D(0.f, 0.f, axis_len * 0.35f),
            half_height * aspect, half_height, info.name_label);
      }
    }

    if ((props_.show_axes && (age_r.visible || age_g.visible || age_b.visible)) ||
        (props_.show_arrows && age_arrow.visible) ||
        (props_.show_names && age_name.visible)) {
      ++drew;
    }
  }
  last_drew_frames_ = drew;
}

}  // namespace display
}  // namespace autoviz
