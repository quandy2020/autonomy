/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/pose/tf_trajectory_display.hpp"

#include <algorithm>
#include <cmath>

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/plugins/displays/common/display_tf_utils.hpp"
#include "autoviz/plugins/displays/common/ogre_overlay_draw.hpp"

namespace autoviz {
namespace display {
namespace {

float SampleAgeSec(const std::chrono::steady_clock::time_point& stamp,
                   const std::chrono::steady_clock::time_point& now) {
  return std::chrono::duration<float>(now - stamp).count();
}

float FadeByAge(float age_sec, float duration_sec) {
  if (duration_sec <= 1e-3f) {
    return 1.f;
  }
  const float t = std::clamp(1.f - age_sec / duration_sec, 0.f, 1.f);
  return t * t * (3.f - 2.f * t);
}

QColor MixHotTrail(const QColor& base, float fade, float alpha_scale) {
  const float hot = fade * fade;
  QColor color(
      static_cast<int>(std::clamp(base.red() + (255 - base.red()) * hot * 0.75f,
                                  0.f, 255.f)),
      static_cast<int>(std::clamp(
          base.green() + (255 - base.green()) * hot * 0.55f, 0.f, 255.f)),
      static_cast<int>(std::clamp(base.blue() + (255 - base.blue()) * hot * 0.35f,
                                  0.f, 255.f)));
  // Keep a visible floor so the wake never fully disappears mid-strip.
  color.setAlphaF(std::clamp(alpha_scale * (0.08f + 0.92f * fade), 0.f, 1.f));
  return color;
}

}  // namespace

TFTrajectoryDisplay::TFTrajectoryDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::nav_msgs::Odometry>(
          "TFTrajectory", std::move(channel),
          "automsgs.msgs.nav_msgs.Odometry") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
TFTrajectoryDisplay::propertySpecs() const {
  return {{"frame", "Frame", "base_link"},
          {"duration", "Duration (s)", "12.0"},
          {"min_distance", "Min Distance (m)", "0.025"},
          {"line_width", "Core Width (m)", "0.06"},
          {"glow_width", "Glow Width (m)", "0.20"},
          {"glow_layers", "Glow Layers", "2", {"0", "1", "2"}},
          {"line_color", "Line Color", "255;64;220", {},
           common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "1.0"},
          {"z_lift", "Z Lift (m)", "0.10"}};
}

void TFTrajectoryDisplay::reset() {
  ChannelDisplay::reset();
  trail_.clear();
  layers_drawn_ = false;
}

void TFTrajectoryDisplay::clearReceivedData() {
  // Keep historical trail on channel reset; only Time-panel Reset clears it.
}

void TFTrajectoryDisplay::appendSample(
    const QVector3D& position, std::chrono::steady_clock::time_point now) {
  const float min_distance =
      common::ParseFloatProperty(propertyValue("min_distance", "0.025"), 0.025f);
  if (!trail_.empty()) {
    if ((position - trail_.back().position).length() <
        std::max(0.008f, min_distance)) {
      trail_.back().stamp = now;
      trail_.back().position = position;
      return;
    }
  }
  trail_.push_back({position, now});
}

void TFTrajectoryDisplay::pruneExpired(
    std::chrono::steady_clock::time_point now) {
  const float duration =
      common::ParseFloatProperty(propertyValue("duration", "12.0"), 12.f);
  const auto max_age =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(
              std::max(0.5, static_cast<double>(duration))));
  while (!trail_.empty() && now - trail_.front().stamp > max_age) {
    trail_.pop_front();
  }
  constexpr std::size_t kMaxSamples = 3000;
  while (trail_.size() > kMaxSamples) {
    trail_.pop_front();
  }
}

bool TFTrajectoryDisplay::tryAppendFromTf(
    std::chrono::steady_clock::time_point now) {
  if (context_ == nullptr || context_->tf_buffer == nullptr) {
    return false;
  }
  const std::string preferred = propertyValue("frame", "base_link");
  const std::string candidates[] = {preferred, "base_link", "base_footprint",
                                    "base", "chassis"};
  for (const std::string& frame : candidates) {
    if (frame.empty()) {
      continue;
    }
    automsgs::msgs::geometry_msgs::TransformStamped tf;
    if (!viz::LookupFixedFrameTransform(context_, frame, &tf)) {
      continue;
    }
    const float z_lift =
        common::ParseFloatProperty(propertyValue("z_lift", "0.10"), 0.10f);
    const QVector3D origin(
        static_cast<float>(tf.transform().translation().x()),
        static_cast<float>(tf.transform().translation().y()),
        static_cast<float>(tf.transform().translation().z()) + z_lift);
    appendSample(origin, now);
    return true;
  }
  return false;
}

void TFTrajectoryDisplay::processMessage(
    const automsgs::msgs::nav_msgs::Odometry& message) {
  if (context_ == nullptr) {
    return;
  }
  const float z_lift =
      common::ParseFloatProperty(propertyValue("z_lift", "0.10"), 0.10f);
  QVector3D position;
  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string target_frame = message.child_frame_id().empty()
                                       ? message.header().frame_id()
                                       : message.child_frame_id();
  try {
    const auto tf = context_->tf_buffer->lookupTransform(
        context_->fixed_frame, target_frame, zero_time);
    position = QVector3D(static_cast<float>(tf.transform().translation().x()),
                         static_cast<float>(tf.transform().translation().y()),
                         static_cast<float>(tf.transform().translation().z()) +
                             z_lift);
  } catch (...) {
    const auto& p = message.pose().pose().pose().position();
    // Odom frame pose: also try map<-odom.
    try {
      const auto tf = context_->tf_buffer->lookupTransform(
          context_->fixed_frame, message.header().frame_id(), zero_time);
      const QVector3D local(static_cast<float>(p.x()),
                            static_cast<float>(p.y()),
                            static_cast<float>(p.z()) + z_lift);
      position = viz::TransformLocalPoint(tf, true, local);
    } catch (...) {
      position = QVector3D(static_cast<float>(p.x()), static_cast<float>(p.y()),
                           static_cast<float>(p.z()) + z_lift);
    }
  }
  appendSample(position, std::chrono::steady_clock::now());
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void TFTrajectoryDisplay::clearLayers(rendering::SceneOverlay& scene) {
  static const std::vector<QVector3D> kEmptyPts;
  static const std::vector<QColor> kEmptyCols;
  static const std::vector<float> kEmptyWidths;
  const std::string prefix = name() + "/comet";
  drawColoredBillboardStripOgreOrGl(context_, scene, prefix + "/glow2",
                                    kEmptyPts, kEmptyCols, kEmptyWidths);
  drawColoredBillboardStripOgreOrGl(context_, scene, prefix + "/glow1",
                                    kEmptyPts, kEmptyCols, kEmptyWidths);
  drawColoredBillboardStripOgreOrGl(context_, scene, prefix + "/core",
                                    kEmptyPts, kEmptyCols, kEmptyWidths);
  layers_drawn_ = false;
}

void TFTrajectoryDisplay::onUpdate() {
  ChannelDisplay::onUpdate();
  const auto now = std::chrono::steady_clock::now();
  const bool got_tf = tryAppendFromTf(now);
  pruneExpired(now);
  if (got_tf || !trail_.empty()) {
    setStatusOk(std::to_string(trail_.size()) + " trail samples");
  } else {
    setStatusWarn("Waiting for /odom or TF (base_link)");
  }
  if (!trail_.empty() && context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void TFTrajectoryDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (trail_.size() < 2) {
    if (layers_drawn_) {
      clearLayers(scene);
    }
    return;
  }

  const QColor base = common::ParseColorProperty(
      propertyValue("line_color", "255;64;220"), QColor(255, 64, 220));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  const float duration =
      common::ParseFloatProperty(propertyValue("duration", "12.0"), 12.f);
  const float core_width =
      common::ParseFloatProperty(propertyValue("line_width", "0.06"), 0.06f);
  const float glow_width =
      common::ParseFloatProperty(propertyValue("glow_width", "0.20"), 0.20f);
  const int glow_layers =
      common::ParseIntProperty(propertyValue("glow_layers", "2"), 2);

  const auto now = std::chrono::steady_clock::now();
  const std::size_t n = trail_.size();
  std::vector<QVector3D> points;
  std::vector<QColor> core_colors;
  std::vector<QColor> glow1_colors;
  std::vector<QColor> glow2_colors;
  std::vector<float> core_widths;
  std::vector<float> glow1_widths;
  std::vector<float> glow2_widths;
  points.reserve(n);
  core_colors.reserve(n);
  glow1_colors.reserve(n);
  glow2_colors.reserve(n);
  core_widths.reserve(n);
  glow1_widths.reserve(n);
  glow2_widths.reserve(n);

  for (std::size_t i = 0; i < n; ++i) {
    const float fade =
        FadeByAge(SampleAgeSec(trail_[i].stamp, now), duration);
    const float head =
        static_cast<float>(i) / static_cast<float>(std::max<std::size_t>(1, n - 1));
    const float f = std::clamp(std::max(fade, head * 0.35f), 0.f, 1.f);

    points.push_back(trail_[i].position);
    core_colors.push_back(MixHotTrail(base, f, alpha * (0.45f + 0.55f * f)));
    glow1_colors.push_back(MixHotTrail(base, f, alpha * 0.28f));
    glow2_colors.push_back(MixHotTrail(base, f, alpha * 0.12f));
    core_widths.push_back(std::max(0.012f, core_width * (0.2f + 0.8f * f)));
    glow1_widths.push_back(std::max(0.03f, glow_width * (0.25f + 0.75f * f)));
    glow2_widths.push_back(
        std::max(0.05f, glow_width * 1.8f * (0.3f + 0.7f * f)));
  }

  const std::string prefix = name() + "/comet";
  if (glow_layers >= 2 && glow_width > core_width) {
    drawColoredBillboardStripOgreOrGl(context_, scene, prefix + "/glow2", points,
                                      glow2_colors, glow2_widths);
  } else {
    drawColoredBillboardStripOgreOrGl(context_, scene, prefix + "/glow2", {},
                                      {}, {});
  }
  if (glow_layers >= 1 && glow_width > core_width) {
    drawColoredBillboardStripOgreOrGl(context_, scene, prefix + "/glow1", points,
                                      glow1_colors, glow1_widths);
  } else {
    drawColoredBillboardStripOgreOrGl(context_, scene, prefix + "/glow1", {},
                                      {}, {});
  }
  drawColoredBillboardStripOgreOrGl(context_, scene, prefix + "/core", points,
                                    core_colors, core_widths);
  layers_drawn_ = true;
}

}  // namespace display
}  // namespace autoviz
