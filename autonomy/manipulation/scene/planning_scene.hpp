/*
 * Copyright 2026 The Openbot Authors
 *
 * Planning scene (MoveIt planning_scene analogue).
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "autonomy/manipulation/core/link_fk.hpp"
#include "autonomy/manipulation/core/robot_model.hpp"

namespace autonomy {
namespace manipulation {

namespace collision {
class CollisionDetector;
}  // namespace collision

namespace scene {

/** @brief Primitive / mesh kind used by in-memory collision proxies. */
enum class ShapeType { kBox, kSphere, kCylinder, kMesh, kUnknown };

/** @brief Single mesh vertex in object-local coordinates. */
struct MeshVertex {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/**
 * @brief Compact in-memory collision proxy; wire format is moveit_msgs.CollisionObject.
 *
 * Pose is the object origin in the world (or link) frame; size_* interpret
 * box extents, sphere diameter (size_x), or cylinder radius/height.
 */
struct CollisionObject {
  std::string id;
  ShapeType type = ShapeType::kUnknown;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double size_x = 0.0;
  double size_y = 0.0;
  double size_z = 0.0;
  std::vector<MeshVertex> mesh_vertices;
};

/** @brief Collision object attached to a robot link with optional touch links. */
struct AttachedCollisionObject {
  std::string link_name;
  CollisionObject object;
  std::vector<std::string> touch_links;
};

/** @brief Occupied voxel / point sample used for coarse occupancy checks. */
struct OccupiedPoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/**
 * @brief Pairwise allowed-collision entries (symmetric lookup by caller).
 *
 * Key is a body id; value is the set of bodies it may collide with.
 */
using AllowedCollisionMatrix =
    std::unordered_map<std::string, std::unordered_set<std::string>>;

/**
 * @brief Planning-scene interface: robot state, world objects, ACM, occupancy.
 *
 * Implementations own thread-safety policy; callers typically share one scene
 * via SceneMonitor / planners / servo.
 */
class PlanningScene {
 public:
  virtual ~PlanningScene() = default;

  /** @brief Replace the current robot joint state snapshot. */
  virtual void SetCurrentState(const core::JointState& state) = 0;

  /** @brief @return Copy of the last set robot joint state. */
  virtual core::JointState GetCurrentState() const = 0;

  /** @brief Insert or replace a world collision object by @p object.id. */
  virtual void AddCollisionObject(const CollisionObject& object) = 0;

  /** @brief Remove a world object; no-op if @p id is unknown. */
  virtual void RemoveCollisionObject(const std::string& id) = 0;

  /** @brief Attach @p attached.object to @p attached.link_name (removes from world if present). */
  virtual void AttachObject(const AttachedCollisionObject& attached) = 0;

  /** @brief Detach object @p object_id from the robot (does not re-add to world). */
  virtual void DetachObject(const std::string& object_id) = 0;

  /** @brief @return Snapshot of all currently attached objects. */
  virtual std::vector<AttachedCollisionObject> GetAttachedObjects() const = 0;

  /**
   * @brief Whether @p state is considered valid for planning (no collision / occupancy).
   * @return true if valid.
   */
  virtual bool IsStateValid(const core::JointState& state) const = 0;

  /**
   * @brief Collision query at @p state (robot–world and/or self per detector).
   * @return true if a collision is detected.
   */
  virtual bool CheckCollision(const core::JointState& state) const = 0;

  /** @brief Collision query with optional contact body names. */
  struct CollisionInfo {
    bool collision = false;
    std::string contact_body_a;
    std::string contact_body_b;
  };

  /**
   * @brief Detailed collision query (contacts when the backend provides them).
   * @param[in] state Robot configuration to test.
   * @return Collision flag and optional contact pair.
   */
  virtual CollisionInfo CheckCollisionDetailed(
      const core::JointState& state) const {
    CollisionInfo info;
    info.collision = CheckCollision(state);
    return info;
  }

  /**
   * @brief Whether every waypoint of @p trajectory is collision-free / valid.
   * @return true if the path is valid.
   */
  virtual bool IsPathValid(const core::RobotTrajectory& trajectory) const = 0;

  /** @brief @return Snapshot of world (non-attached) collision objects. */
  virtual std::vector<CollisionObject> GetCollisionObjects() const = 0;

  /**
   * @brief Set whether bodies @p a and @p b may collide without reporting.
   * @param[in] allowed true to allow the pair, false to forbid.
   */
  virtual void SetAllowedCollision(const std::string& a, const std::string& b,
                                   bool allowed) = 0;

  /**
   * @brief Query the allowed-collision matrix for the unordered pair (@p a, @p b).
   * @return true if the pair is allowed.
   */
  virtual bool IsCollisionAllowed(const std::string& a,
                                  const std::string& b) const = 0;

  /** @brief Install the collision backend used by CheckCollision / IsStateValid. */
  virtual void SetCollisionDetector(
      std::shared_ptr<collision::CollisionDetector> detector) = 0;

  /** @brief Optional FK tree for attached-object / link-frame transforms. */
  virtual void SetLinkTree(
      std::shared_ptr<const core::LinkFkTree> tree) = 0;

  /**
   * @brief Replace occupancy samples used for coarse environment checks.
   * @param[in] points Occupied centers in the world frame.
   * @param[in] resolution Voxel / query radius scale (meters).
   */
  virtual void SetOccupiedPoints(std::vector<OccupiedPoint> points,
                                 double resolution) = 0;

  /** @brief Clear occupancy samples. */
  virtual void ClearOccupiedPoints() = 0;

  /** @brief Remove all world (non-attached) collision objects. */
  virtual void ClearWorldObjects() = 0;

  /** @brief Detach and drop all attached collision objects. */
  virtual void ClearAttachedObjects() = 0;

  /** @brief @return Snapshot of occupied points. */
  virtual std::vector<OccupiedPoint> OccupiedPoints() const = 0;

  /** @brief @return Occupancy resolution last set with SetOccupiedPoints. */
  virtual double OccupancyResolution() const = 0;
};

/**
 * @brief Fill size_* AABB of @p object from mesh_vertices (no-op if empty).
 * @param[in,out] object Mesh collision object to update.
 */
void UpdateMeshAabb(CollisionObject* object);

/**
 * @brief Pose @p object from link frame into world using @p link_tf.
 * @return Transformed collision object (id / shape preserved).
 */
CollisionObject TransformAttached(const CollisionObject& object,
                                  const core::Transform& link_tf);

}  // namespace scene
}  // namespace manipulation
}  // namespace autonomy
