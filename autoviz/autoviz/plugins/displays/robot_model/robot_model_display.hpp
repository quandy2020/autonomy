/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins RobotModelDisplay
 * (Willow Garage / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file robot_model_display.hpp
 * @brief RobotModel Display — URDF visuals placed by TF / JointState.
 *
 * Mirrors RViz @c RobotModelDisplay: description from Topic or File, owns a
 * @ref robot::Robot helper, and updates link poses each frame via
 * @ref robot::TFLinkUpdater.
 *
 * Autoviz extras vs stock RViz: JointState channel (FK fallback), PBR/textured
 * mesh path, and Autolink @c ChannelReaderRegistry subscriptions.
 *
 * @see robot::Robot
 * @see UrdfModel
 */

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>

#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/display.hpp"
#include "autoviz/plugins/displays/common/proto_payload_utils.hpp"
#include "autoviz/plugins/robot/robot.hpp"
#include "autoviz/plugins/transformation/tf_frame_transformer.hpp"
#include "autoviz/plugins/transformation/transformer_guard.hpp"
#include "autoviz/common/interaction/channel_reader_registry.hpp"
#include "autoviz/common/interaction/message_queue.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {

/**
 * @class RobotModelDisplay
 * @brief RViz RobotModel display adapted for Autoviz / Autolink.
 */
class RobotModelDisplay : public Display {
 public:
  explicit RobotModelDisplay(std::string joint_channel);

  std::string typeId() const override { return "RobotModel"; }
  std::string channel() const override { return joint_channel_; }
  void setChannel(const std::string& channel) override;
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

  /** Clears robot meshes and cached description (RViz @c clear). */
  void clear();

 protected:
  void onEnable() override;
  void onDisable() override;
  void reset() override;
  void onUpdate() override;
  void onPropertyChanged(const std::string& key) override;
  void onDraw(rendering::SceneOverlay& scene) override;

  /**
   * @brief Loads URDF from the active Description Source (Topic file path or
   *        File property) — RViz @c load_urdf.
   */
  virtual void load_urdf();
  virtual void load_urdf_from_file(const std::string& filepath);
  virtual void load_urdf_from_string(const std::string& robot_description);
  void display_urdf_content();
  void updateRobot();

 private:
  void subscribeChannels();
  void updateVisualVisible();
  void updateCollisionVisible();
  void updateAlpha();
  void updateMassVisible();
  void updateInertiaVisible();
  void processJointState(const proto_wire::ParsedJointState& message);
  void processDescription(const std::string& urdf_text);

  std::string joint_channel_;
  std::string description_channel_;
  std::string robot_description_;
  std::unique_ptr<robot::Robot> robot_;

  bool has_new_transforms_ = true;
  std::chrono::nanoseconds time_since_last_transform_{0};

  integration::MessageQueue joint_queue_;
  integration::MessageQueue description_queue_;
  integration::ChannelReaderRegistry::SubscriptionId joint_subscription_ = 0;
  integration::ChannelReaderRegistry::SubscriptionId description_subscription_ =
      0;

  std::unique_ptr<transformation::TransformerGuard<
      transformation::TFFrameTransformer>>
      transformer_guard_;
  bool transformer_guard_initialized_ = false;
};

}  // namespace display
}  // namespace autoviz
