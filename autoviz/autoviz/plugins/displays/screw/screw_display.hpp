/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins ScrewDisplay
 * (Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file screw_display.hpp
 * @brief Shared base for TwistStamped / AccelStamped screw visuals (RViz).
 *
 * RViz @c ScrewDisplay&lt;MessageType&gt; owns ScrewVisual history, colors,
 * scales, alpha, and @c processMessagePrivate. Autoviz keeps the same
 * property model and history buffer, drawing via @ref drawScrewOgreOrGl
 * (no ROS / MessageFilter).
 *
 * @see TwistStampedDisplay
 * @see AccelStampedDisplay
 * @see drawScrewOgreOrGl()
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <string>
#include <utility>

#include <QColor>
#include <QVector3D>

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"
#include "autoviz/plugins/displays/common/ogre_overlay_draw.hpp"
#include "autoviz/plugins/displays/common/transform_utils.hpp"
#include "autoviz/rendering/ogre_scene_host.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {

/**
 * @class ScrewDisplay
 * @brief Channel display base that draws linear + angular as a screw visual.
 *
 * @tparam MessageT Protobuf message type (TwistStamped or AccelStamped).
 *
 * ## Properties (RViz ScrewDisplay defaults)
 *
 * | Key                | Label              | Default   |
 * |--------------------|--------------------|-----------|
 * | linear_color       | Linear Color       | 204;51;51 |
 * | angular_color      | Angular Color      | 204;204;51|
 * | alpha              | Alpha              | 1.0       |
 * | linear_scale       | Linear Arrow Scale | 2.0       |
 * | angular_scale      | Angular Arrow Scale| 2.0       |
 * | arrow_width        | Arrow Width        | 0.5       |
 * | history_length     | History Length     | 1         |
 * | hide_small_values  | Hide Small Values  | true      |
 */
template <typename MessageT>
class ScrewDisplay : public ChannelDisplay<MessageT> {
 public:
  /**
   * @brief Constructs the screw display with channel metadata.
   *
   * @param type_id Catalog type id (e.g. @c "TwistStamped").
   * @param channel Autolink channel name.
   * @param message_type Wire / schema type string.
   */
  ScrewDisplay(std::string type_id, std::string channel,
               std::string message_type)
      : ChannelDisplay<MessageT>(std::move(type_id), std::move(channel),
                                 std::move(message_type)) {
    this->setProperties({});
  }

  /**
   * @brief Declares RViz-aligned screw visual properties.
   * @return Specs for the Displays property editor.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override {
    return {{"linear_color", "Linear Color", "204;51;51"},
            {"angular_color", "Angular Color", "204;204;51"},
            {"alpha", "Alpha", "1.0"},
            {"linear_scale", "Linear Arrow Scale", "2.0"},
            {"angular_scale", "Angular Arrow Scale", "2.0"},
            {"arrow_width", "Arrow Width", "0.5"},
            {"history_length", "History Length", "1"},
            {"hide_small_values", "Hide Small Values", "true"}};
  }

 protected:
  /**
   * @struct Sample
   * @brief One history entry: frame origin + linear/angular in fixed frame.
   */
  struct Sample {
    QVector3D origin;  /**< Message frame origin (fixed frame). */
    QVector3D linear;  /**< Linear vector (fixed frame). */
    QVector3D angular; /**< Angular vector (fixed frame). */
  };

  /**
   * @brief RViz @c ScrewDisplay::processMessagePrivate — TF + history push.
   *
   * @param frame_id Message header frame (empty → fixed frame).
   * @param local_linear Linear vector in the message frame.
   * @param local_angular Angular vector in the message frame.
   */
  void processMessagePrivate(const std::string& frame_id,
                             const QVector3D& local_linear,
                             const QVector3D& local_angular) {
    if (this->context_ == nullptr) {
      return;
    }
    if (!std::isfinite(local_linear.x()) || !std::isfinite(local_linear.y()) ||
        !std::isfinite(local_linear.z()) || !std::isfinite(local_angular.x()) ||
        !std::isfinite(local_angular.y()) ||
        !std::isfinite(local_angular.z())) {
      this->setStatusError(
          "Message contained invalid floating point values (nans or infs)");
      return;
    }

    Sample sample;
    const auto zero_time = autoviz::commsgs::ZeroTime();
    const std::string frame =
        frame_id.empty() ? this->context_->fixed_frame : frame_id;
    try {
      const auto tf = this->context_->tf_buffer->lookupTransform(
          this->context_->fixed_frame, frame, zero_time);
      sample.origin = transformPoint(tf, QVector3D(0.f, 0.f, 0.f));
      sample.linear = rotateVector(tf.transform().rotation(), local_linear);
      sample.angular = rotateVector(tf.transform().rotation(), local_angular);
    } catch (...) {
      this->setStatusError("Error transforming from frame '" + frame +
                           "' to frame '" + this->context_->fixed_frame + "'");
      return;
    }

    history_.push_back(sample);
    trimHistory();
    if (this->context_->request_redraw) {
      this->context_->request_redraw();
    }
  }

  /**
   * @brief Clears history and removes Ogre screw visuals for this display.
   */
  void clearReceivedData() override {
    history_.clear();
    clearOgreHistory();
  }

  /**
   * @brief Draws every history sample via @ref drawScrewOgreOrGl.
   *
   * @param scene Scene overlay / Ogre host for the current frame.
   */
  void onDraw(rendering::SceneOverlay& scene) override {
    if (history_.empty()) {
      return;
    }
    trimHistory();

    QColor linear_color = common::ParseColorProperty(
        this->propertyValue("linear_color", "204;51;51"));
    QColor angular_color = common::ParseColorProperty(
        this->propertyValue("angular_color", "204;204;51"));
    const float alpha =
        common::ParseFloatProperty(this->propertyValue("alpha", "1.0"), 1.f);
    linear_color.setAlphaF(std::clamp(alpha, 0.f, 1.f));
    angular_color.setAlphaF(std::clamp(alpha, 0.f, 1.f));

    const float linear_scale = common::ParseFloatProperty(
        this->propertyValue("linear_scale", "2.0"), 2.f);
    const float angular_scale = common::ParseFloatProperty(
        this->propertyValue("angular_scale", "2.0"), 2.f);
    const float arrow_width = common::ParseFloatProperty(
        this->propertyValue("arrow_width", "0.5"), 0.5f);
    const bool hide_small_values = common::ParseBoolProperty(
        this->propertyValue("hide_small_values", "true"), true);

    const std::string prefix = this->name() + "/screw/";
    for (size_t i = 0; i < history_.size(); ++i) {
      const Sample& sample = history_[i];
      drawScrewOgreOrGl(this->context_, scene, prefix + std::to_string(i),
                        sample.origin, sample.linear, sample.angular,
                        linear_color, angular_color, linear_scale,
                        angular_scale, arrow_width, hide_small_values);
    }
    // Drop stale Ogre slots when history shrank (RViz pops the deque).
    if (this->context_ != nullptr &&
        this->context_->ogre_scene_host != nullptr) {
      for (size_t i = history_.size(); i < drawn_slots_; ++i) {
        this->context_->ogre_scene_host->removeDisplay(prefix +
                                                       std::to_string(i));
      }
    }
    drawn_slots_ = history_.size();
  }

 private:
  /**
   * @brief Caps @c history_ to the History Length property (min 1).
   */
  void trimHistory() {
    int length = static_cast<int>(common::ParseFloatProperty(
        this->propertyValue("history_length", "1"), 1.f));
    length = std::max(1, length);
    while (static_cast<int>(history_.size()) > length) {
      history_.pop_front();
    }
  }

  /** Removes all Ogre screw entries under this display's name prefix. */
  void clearOgreHistory() {
    if (this->context_ != nullptr &&
        this->context_->ogre_scene_host != nullptr) {
      this->context_->ogre_scene_host->removeDisplaysWithPrefix(
          this->name() + "/screw/");
    }
    drawn_slots_ = 0;
  }

  /** Circular history of screw samples (oldest at front). */
  std::deque<Sample> history_;

  /** Number of Ogre screw slots drawn last frame (for stale cleanup). */
  size_t drawn_slots_ = 0;
};

}  // namespace display
}  // namespace autoviz
