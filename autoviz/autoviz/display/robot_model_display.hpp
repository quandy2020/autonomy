/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <QImage>

#include "autoviz/common/display_property.hpp"
#include "autoviz/display/display.hpp"
#include "autoviz/display/obj_mesh.hpp"
#include "autoviz/display/proto_payload_utils.hpp"
#include "autoviz/display/urdf_model.hpp"
#include "autoviz/integration/message_queue.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {

class RobotModelDisplay : public Display {
 public:
  explicit RobotModelDisplay(std::string joint_channel);

  std::string typeId() const override { return "RobotModel"; }
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
  void rebuildMeshCache();
  void cacheGeometryMesh(const UrdfGeometry& geometry, const std::string& link_name,
                         std::unordered_map<std::string, ObjMesh>* cache);
  void drawLinkGeometry(rendering::SceneOverlay& scene, const UrdfGeometry& geometry,
                        const ObjMesh* mesh, const QMatrix4x4& link_transform,
                        const QColor& color, bool solid_visual, bool use_pbr) const;
  QColor linkColor(const UrdfGeometry& geometry, const QColor& fallback,
                   float alpha, bool use_urdf_materials) const;
  QImage loadMaterialTexture(const UrdfMaterial& material) const;
  void processJointState(const proto_wire::ParsedJointState& message);
  void processDescription(const std::string& urdf_text);

  std::string joint_channel_;
  std::string description_channel_;
  UrdfModel model_;
  std::unordered_map<std::string, ObjMesh> visual_meshes_;
  std::unordered_map<std::string, ObjMesh> collision_meshes_;
  mutable std::unordered_map<std::string, QImage> texture_cache_;
  std::unordered_map<std::string, double> joint_positions_;
  integration::MessageQueue joint_queue_;
  integration::MessageQueue description_queue_;
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>>
      joint_reader_;
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>>
      description_reader_;
};

}  // namespace display
}  // namespace autoviz
