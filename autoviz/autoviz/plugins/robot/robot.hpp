/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins robot::Robot
 * (Willow Garage / Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file robot.hpp
 * @brief URDF robot visual helper (RViz @c robot::Robot layout).
 *
 * Owns @ref RobotLink / @ref RobotJoint maps, applies @ref LinkUpdater poses,
 * and draws via Autoviz @ref SceneOverlay / Ogre helpers.
 *
 * @see RobotModelDisplay
 * @see TFLinkUpdater
 * @see UrdfModel
 */

#pragma once

#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include <QColor>
#include <QImage>
#include <QMatrix4x4>

#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/common/obj_mesh.hpp"
#include "autoviz/plugins/displays/common/proto_payload_utils.hpp"
#include "autoviz/plugins/displays/common/urdf_model.hpp"
#include "autoviz/plugins/robot/link_updater.hpp"
#include "autoviz/plugins/robot/robot_joint.hpp"
#include "autoviz/plugins/robot/robot_link.hpp"

namespace autoviz {

namespace common {
class DisplayContext;
}

namespace rendering {
class SceneOverlay;
}

namespace display {
namespace robot {

/**
 * @class Robot
 * @brief Loads a URDF and draws visual / collision geometries (RViz Robot).
 */
class Robot {
 public:
  using LinkMap = std::map<std::string, std::unique_ptr<RobotLink>>;
  using JointMap = std::map<std::string, std::unique_ptr<RobotJoint>>;

  Robot() = default;
  ~Robot();

  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;

  /** Clears meshes, links, joints, and the parsed model (RViz @c clear). */
  void clear();

  /**
   * @brief Loads URDF XML text into @ref UrdfModel and rebuilds link/joint maps.
   * @return @c true when parse + mesh rebuild succeeded.
   */
  bool loadFromString(const std::string& urdf_text);

  /** Loads a URDF file path. */
  bool loadFromFile(const std::string& filepath);

  bool empty() const { return model_.empty(); }
  const UrdfModel& model() const { return model_; }
  UrdfModel& model() { return model_; }

  const LinkMap& getLinks() const { return links_; }
  const JointMap& getJoints() const { return joints_; }
  RobotLink* getLink(const std::string& name);
  RobotJoint* getJoint(const std::string& name);

  void setVisualVisible(bool visible);
  void setCollisionVisible(bool visible);
  void setMassVisible(bool visible) { mass_visible_ = visible; }
  void setInertiaVisible(bool visible) { inertia_visible_ = visible; }
  void setAlpha(float alpha);
  void setVisible(bool visible) { visible_ = visible; }

  bool visualVisible() const { return visual_visible_; }
  bool collisionVisible() const { return collision_visible_; }
  float alpha() const { return alpha_; }
  bool visible() const { return visible_; }

  /** Applies joint positions used for FK when TF is incomplete. */
  void setJointPositions(
      const std::unordered_map<std::string, double>& positions);

  const std::unordered_map<std::string, double>& jointPositions() const {
    return joint_positions_;
  }

  /**
   * @brief Updates every link pose from @p updater (RViz @c Robot::update).
   *
   * Links that fail lookup keep @c have_transform false so @ref draw can fall
   * back to JointState FK × root TF.
   */
  void update(const LinkUpdater& updater);

  /**
   * @brief Draws the robot for the current link / joint / TF state.
   */
  void draw(rendering::SceneOverlay& scene, common::DisplayContext* context,
            const common::DisplayPropertyMap& properties,
            const std::string& display_name);

  /** Geometry-load warnings accumulated during the last @ref load*. */
  const std::string& geometryErrors() const { return geometry_errors_; }

 private:
  void rebuildLinkJointMaps();
  void rebuildMeshCache();
  void cacheGeometryMesh(const UrdfGeometry& geometry,
                         const std::string& link_name,
                         std::unordered_map<std::string, ObjMesh>* cache);
  QColor linkColor(const UrdfGeometry& geometry, const QColor& fallback,
                   float alpha, bool use_urdf_materials) const;
  QImage loadMaterialTexture(const UrdfMaterial& material) const;
  void drawLinkGeometry(rendering::SceneOverlay& scene,
                        const UrdfGeometry& geometry, const ObjMesh* mesh,
                        const QMatrix4x4& link_transform, const QColor& color,
                        bool solid_visual, bool use_pbr) const;

  UrdfModel model_;
  LinkMap links_;
  JointMap joints_;
  std::unordered_map<std::string, ObjMesh> visual_meshes_;
  std::unordered_map<std::string, ObjMesh> collision_meshes_;
  mutable std::unordered_map<std::string, QImage> texture_cache_;
  std::unordered_map<std::string, double> joint_positions_;
  std::string geometry_errors_;

  bool visible_ = true;
  bool visual_visible_ = true;
  bool collision_visible_ = false;
  bool mass_visible_ = false;
  bool inertia_visible_ = false;
  float alpha_ = 1.f;
};

}  // namespace robot
}  // namespace display
}  // namespace autoviz
