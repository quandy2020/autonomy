/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <unordered_map>

#include "autoviz/common/display_property.hpp"
#include "autoviz/display/display.hpp"
#include "autoviz/display/proto_payload_utils.hpp"
#include "autoviz/display/urdf_model.hpp"
#include "autoviz/integration/message_queue.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {

/** Visualizes JointState effort/torque on URDF joint axes (RViz Effort). */
class EffortDisplay : public Display {
 public:
  explicit EffortDisplay(std::string joint_channel);

  std::string typeId() const override { return "Effort"; }
  std::string channel() const override { return joint_channel_; }
  void setChannel(const std::string& channel) override;

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void onEnable() override;
  void onDisable() override;
  void onUpdate() override;
  void onPropertyChanged(const std::string& key) override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  void reloadUrdf();
  void processJointState(const proto_wire::ParsedJointState& message);
  void drawEffortArrow(rendering::SceneOverlay& scene, const QVector3D& origin,
                       const QVector3D& axis, double effort,
                       const QColor& positive_color,
                       const QColor& negative_color, float min_length,
                       float scale) const;

  std::string joint_channel_;
  UrdfModel model_;
  std::unordered_map<std::string, double> joint_positions_;
  std::unordered_map<std::string, double> joint_efforts_;
  integration::MessageQueue joint_queue_;
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>>
      joint_reader_;
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>>
      description_reader_;
};

}  // namespace display
}  // namespace autoviz
