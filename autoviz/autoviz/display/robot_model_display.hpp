/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file robot_model_display.hpp
 * @brief Display for URDF robot visuals driven by JointState (+ description).
 *
 * Loads a URDF from topic (@c std_msgs/String, default @c /robot_description)
 * or file. Each link is placed with TF from the fixed frame, the same way
 * RViz2 RobotModel uses @c /tf. Joint positions are the fallback when a link
 * has no transform yet.
 *
 * ## Properties
 *
 * - @c description_source — @c Topic or @c File
 * - @c description_channel / @c urdf_path — description inputs
 * - @c tf_prefix / @c root_link / @c update_interval
 * - @c visual_enabled / @c collision_enabled / @c visual_style
 * - @c alpha / @c color / @c use_urdf_materials / @c show_axes
 *
 * @note Unlike most channel displays, the primary @ref channel() is the
 *       **joint** topic; description uses a separate property channel.
 *       Both topics are subscribed through ChannelReaderRegistry so a
 *       channels-panel probe on the same name still delivers the payload.
 *
 * @see UrdfModel
 * @see proto_wire::ParsedJointState
 * @see ogre_pbr_mesh_draw.hpp
 * @see Display
 */

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
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/integration/message_queue.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {

/**
 * @class RobotModelDisplay
 * @brief Renders a URDF robot model with live joint state.
 *
 * ## Data flow
 *
 * - **Description:** @ref processDescription() → @ref UrdfModel::loadFromString
 *   → @ref rebuildMeshCache().
 * - **Joints:** @ref processJointState() updates @c joint_positions_.
 * - **Draw:** @ref onDraw() computes link transforms and emits meshes via
 *   PBR / entity / flat mesh helpers.
 *
 * @note Joint and description subscriptions go through ChannelReaderRegistry.
 *       Lifetimes are tied to @ref onEnable() / @ref onDisable().
 */
class RobotModelDisplay : public Display {
 public:
  /**
   * @brief Constructs a RobotModel display with an initial joint channel.
   *
   * @param joint_channel Autolink / topic name for JointState.
   */
  explicit RobotModelDisplay(std::string joint_channel);

  /**
   * @brief Catalog type id used by the display registry.
   * @return Always @c "RobotModel".
   */
  std::string typeId() const override { return "RobotModel"; }

  /**
   * @brief Returns the joint-state channel name.
   * @return Current @c joint_channel_.
   */
  std::string channel() const override { return joint_channel_; }

  /**
   * @brief Rebinds the joint-state channel (re-subscribes if enabled).
   *
   * @param channel New JointState topic / channel name.
   */
  void setChannel(const std::string& channel) override;

  /**
   * @brief Declares description source, visual, and TF-prefix properties.
   * @return Property specification list for the Displays tree.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Subscribes to joint and description channels.
   */
  void onEnable() override;

  /**
   * @brief Joins the shared channel readers for joints and description.
   */
  void subscribeChannels();

  /**
   * @brief Unsubscribes readers and clears queues.
   */
  void onDisable() override;

  /**
   * @brief Clears model caches and joint positions (Display Reset).
   */
  void reset() override;

  /**
   * @brief Drains message queues and refreshes joint / description state.
   */
  void onUpdate() override;

  /**
   * @brief Reacts to property edits (reload URDF, rebuild meshes, etc.).
   *
   * @param key Changed property key.
   */
  void onPropertyChanged(const std::string& key) override;

  /**
   * @brief Draws visual/collision link meshes for the current joint state.
   *
   * @param scene Scene overlay / Ogre host for the current frame.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  /**
   * @brief Reloads URDF from file or last description topic payload.
   */
  void reloadUrdf();

  /**
   * @brief Rebuilds @c visual_meshes_ / @c collision_meshes_ from @c model_.
   */
  void rebuildMeshCache();

  /**
   * @brief Generates or loads mesh data for one URDF geometry into @p cache.
   *
   * @param geometry Visual or collision geometry descriptor.
   * @param link_name Owning link (for cache keys / logging).
   * @param cache Destination mesh map.
   */
  void cacheGeometryMesh(const UrdfGeometry& geometry, const std::string& link_name,
                         std::unordered_map<std::string, ObjMesh>* cache);

  /**
   * @brief Draws one link geometry (solid / wireframe / PBR).
   *
   * @param scene Target overlay.
   * @param geometry URDF geometry (type, material, mesh path).
   * @param mesh Cached mesh pointer; may be @c nullptr for primitives rebuilt
   *        on the fly.
   * @param link_transform Fixed-frame link pose.
   * @param color Resolved draw color.
   * @param solid_visual When @c false, wireframe style.
   * @param use_pbr Prefer PBR draw path when materials allow.
   */
  void drawLinkGeometry(rendering::SceneOverlay& scene, const UrdfGeometry& geometry,
                        const ObjMesh* mesh, const QMatrix4x4& link_transform,
                        const QColor& color, bool solid_visual, bool use_pbr) const;

  /**
   * @brief Resolves link color from URDF material or fallback / alpha.
   *
   * @param geometry Geometry carrying optional material.
   * @param fallback Display Color property.
   * @param alpha Global alpha multiplier.
   * @param use_urdf_materials Whether to prefer URDF rgba.
   * @return Final QColor.
   */
  QColor linkColor(const UrdfGeometry& geometry, const QColor& fallback,
                   float alpha, bool use_urdf_materials) const;

  /**
   * @brief Loads and caches a material diffuse texture image.
   *
   * @param material URDF material with optional @c texture_filename.
   * @return Texture image (may be null/empty on failure).
   */
  QImage loadMaterialTexture(const UrdfMaterial& material) const;

  /**
   * @brief Applies a decoded JointState to @c joint_positions_.
   *
   * @param message Parsed joint arrays.
   */
  void processJointState(const proto_wire::ParsedJointState& message);

  /**
   * @brief Parses URDF XML text and rebuilds the mesh cache.
   *
   * @param urdf_text Robot description XML.
   */
  void processDescription(const std::string& urdf_text);

  std::string joint_channel_;       /**< JointState topic / channel. */
  std::string description_channel_; /**< Description topic (when source=Topic). */
  std::string description_text_;    /**< Last URDF text, so a repeat is not reparsed. */
  UrdfModel model_;                 /**< Parsed URDF model. */
  std::unordered_map<std::string, ObjMesh> visual_meshes_;    /**< Visual cache. */
  std::unordered_map<std::string, ObjMesh> collision_meshes_; /**< Collision cache. */
  mutable std::unordered_map<std::string, QImage> texture_cache_; /**< Texture cache. */
  std::unordered_map<std::string, double> joint_positions_; /**< Latest joint values. */
  integration::MessageQueue joint_queue_;        /**< Incoming JointState queue. */
  integration::MessageQueue description_queue_;  /**< Incoming description queue. */
  integration::ChannelReaderRegistry::SubscriptionId joint_subscription_ =
      0; /**< Shared reader for joints. */
  integration::ChannelReaderRegistry::SubscriptionId description_subscription_ =
      0; /**< Shared reader for the URDF topic. */
};

}  // namespace display
}  // namespace autoviz
