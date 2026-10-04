/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file urdf_model.hpp
 * @brief Minimal URDF parser for robot-model visualization.
 *
 * Parses links, joints, materials, and geometry (box / cylinder / sphere /
 * mesh) sufficiently for @ref RobotModelDisplay. Not a full URDF/DOM
 * implementation — unsupported tags are skipped.
 *
 * ## Capabilities
 *
 * - Load from string or file (@ref UrdfModel::loadFromString /
 *   @ref UrdfModel::loadFromFile)
 * - Resolve mesh / texture paths relative to the URDF base directory
 * - Compute link world transforms from joint positions
 *   (@ref UrdfModel::computeLinkTransforms)
 *
 * @see RobotModelDisplay
 * @see UrdfLink
 * @see UrdfJoint
 * @see ObjMesh
 */

#pragma once

#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector3D>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoviz {
namespace display {

/**
 * @struct UrdfMaterial
 * @brief Visual material (RGBA, optional PBR hints, optional texture path).
 */
struct UrdfMaterial {
  float r = 0.7f;  /**< Red in [0, 1]. */
  float g = 0.7f;  /**< Green in [0, 1]. */
  float b = 0.7f;  /**< Blue in [0, 1]. */
  float a = 1.f;   /**< Alpha in [0, 1]. */
  float metallic = 0.08f;  /**< Optional metallic factor. */
  float roughness = 0.52f; /**< Optional roughness factor. */
  std::string texture_filename; /**< Relative or package texture path. */
  bool valid = false;      /**< Whether color/material was parsed. */
  bool has_texture = false; /**< Whether @c texture_filename is set. */
};

/**
 * @struct UrdfGeometry
 * @brief One visual or collision geometry attached to a link.
 */
struct UrdfGeometry {
  /**
   * @enum Type
   * @brief Supported URDF geometry primitives / mesh.
   */
  enum class Type {
    kBox,      /**< Box with @c size extents. */
    kCylinder, /**< Cylinder (radius=size.x, length=size.z convention). */
    kSphere,   /**< Sphere (radius=size.x). */
    kMesh,     /**< External mesh via @c mesh_filename. */
    kUnknown,  /**< Unrecognized / empty geometry. */
  };

  Type type = Type::kUnknown;          /**< Geometry kind. */
  QVector3D size{0.1f, 0.1f, 0.1f};    /**< Box extents or primitive params. */
  QVector3D origin;                    /**< Origin translation relative to link. */
  QQuaternion rotation;                /**< Origin rotation relative to link. */
  std::string mesh_filename;           /**< Mesh path when @c type == kMesh. */
  QVector3D mesh_scale{1.f, 1.f, 1.f}; /**< Per-axis mesh scale. */
  UrdfMaterial material;               /**< Optional material override. */
};

/**
 * @struct UrdfLink
 * @brief One robot link with optional visual and collision geometry.
 */
struct UrdfLink {
  std::string name;          /**< Link name (unique in the model). */
  UrdfGeometry visual;       /**< Visual geometry (if @c has_visual). */
  UrdfGeometry collision;    /**< Collision geometry (if @c has_collision). */
  bool has_visual = false;   /**< Whether @c visual was parsed. */
  bool has_collision = false; /**< Whether @c collision was parsed. */
};

/**
 * @enum UrdfJointType
 * @brief Supported joint types for forward kinematics in the viewer.
 */
enum class UrdfJointType {
  kFixed,      /**< Fixed joint (no DOF). */
  kRevolute,   /**< Revolute about @c axis with limits (limits unused here). */
  kContinuous, /**< Continuous revolute. */
  kPrismatic,  /**< Prismatic along @c axis. */
  kUnknown,    /**< Unsupported joint type (treated as fixed). */
};

/**
 * @struct UrdfJoint
 * @brief One joint connecting @c parent → @c child.
 */
struct UrdfJoint {
  std::string name;   /**< Joint name (matches JointState). */
  std::string parent; /**< Parent link name. */
  std::string child;  /**< Child link name. */
  UrdfJointType type = UrdfJointType::kUnknown; /**< Joint type. */
  QVector3D origin;   /**< Joint origin translation in parent. */
  QQuaternion rotation; /**< Joint origin rotation in parent. */
  QVector3D axis{1.f, 0.f, 0.f}; /**< Motion axis in joint frame. */
};

/**
 * @class UrdfModel
 * @brief In-memory URDF graph used by @ref RobotModelDisplay.
 *
 * ## Usage
 *
 * 1. @ref loadFromString / @ref loadFromFile
 * 2. Optionally @ref resolveMeshPath for each mesh filename
 * 3. @ref computeLinkTransforms with current joint positions
 * 4. Draw each link's visual/collision with the resulting matrices
 *
 * @note @c base_directory_ is set from the file path on @ref loadFromFile,
 *       or left empty for topic-loaded XML (callers may set paths differently).
 */
class UrdfModel {
 public:
  /**
   * @brief Parses URDF XML from an in-memory string.
   *
   * @param xml Full robot description XML.
   * @return @c true on successful parse with at least one link.
   */
  bool loadFromString(const std::string& xml);

  /**
   * @brief Loads and parses a URDF file from disk.
   *
   * Sets @c base_directory_ to the file's parent directory for mesh resolution.
   *
   * @param path Filesystem path to a `.urdf` / `.xacro`-expanded XML file.
   * @return @c true on successful read and parse.
   */
  bool loadFromFile(const std::string& path);

  /**
   * @brief Returns parsed links.
   * @return Const reference to the link list.
   */
  const std::vector<UrdfLink>& links() const { return links_; }

  /**
   * @brief Returns parsed joints.
   * @return Const reference to the joint list.
   */
  const std::vector<UrdfJoint>& joints() const { return joints_; }

  /**
   * @brief Returns the inferred root link name.
   * @return Root link id (empty if unknown).
   */
  const std::string& rootLink() const { return root_link_; }

  /**
   * @brief Directory used to resolve relative mesh/texture paths.
   * @return Base directory string (may be empty for topic loads).
   */
  const std::string& baseDirectory() const { return base_directory_; }

  /**
   * @brief Whether the model has no links.
   * @return @c true if @c links_ is empty.
   */
  bool empty() const { return links_.empty(); }

  /**
   * @brief Resolves a mesh filename against @p base_directory (and package://).
   *
   * @param base_directory URDF base directory.
   * @param filename Mesh path from the URDF.
   * @return Absolute or usable filesystem path (may be unchanged on failure).
   */
  static std::string resolveMeshPath(const std::string& base_directory,
                                     const std::string& filename);

  /**
   * @brief Resolves a texture filename (same rules as meshes).
   *
   * @param base_directory URDF base directory.
   * @param filename Texture path from the material.
   * @return Resolved path via @ref resolveMeshPath.
   */
  static std::string resolveTexturePath(const std::string& base_directory,
                                        const std::string& filename) {
    return resolveMeshPath(base_directory, filename);
  }

  /**
   * @brief Forward-kinematics: link name → fixed/root-relative transform.
   *
   * Missing joint positions default to 0. Unsupported joint types are treated
   * as fixed.
   *
   * @param joint_positions Map of joint name → position (rad or meters).
   * @return Transform map for every reachable link.
   */
  std::unordered_map<std::string, QMatrix4x4> computeLinkTransforms(
      const std::unordered_map<std::string, double>& joint_positions) const;

 private:
  /**
   * @brief Internal XML parse implementation shared by loaders.
   *
   * @param xml Robot description XML.
   * @return @c true on success.
   */
  bool parseXml(const std::string& xml);

  std::vector<UrdfLink> links_;   /**< Parsed links. */
  std::vector<UrdfJoint> joints_; /**< Parsed joints. */
  std::string root_link_;         /**< Root link name. */
  std::string base_directory_;    /**< Mesh/texture resolution base. */
  std::unordered_map<std::string, UrdfMaterial> material_library_; /**< Named materials. */
};

}  // namespace display
}  // namespace autoviz
