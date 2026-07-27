/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector3D>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoviz {
namespace display {

struct UrdfMaterial {
  float r = 0.7f;
  float g = 0.7f;
  float b = 0.7f;
  float a = 1.f;
  float metallic = 0.08f;
  float roughness = 0.52f;
  std::string texture_filename;
  bool valid = false;
  bool has_texture = false;
};

struct UrdfGeometry {
  enum class Type { kBox, kCylinder, kSphere, kMesh, kUnknown };
  Type type = Type::kUnknown;
  QVector3D size{0.1f, 0.1f, 0.1f};
  QVector3D origin;
  QQuaternion rotation;
  std::string mesh_filename;
  QVector3D mesh_scale{1.f, 1.f, 1.f};
  UrdfMaterial material;
};

struct UrdfLink {
  std::string name;
  UrdfGeometry visual;
  UrdfGeometry collision;
  bool has_visual = false;
  bool has_collision = false;
};

enum class UrdfJointType {
  kFixed,
  kRevolute,
  kContinuous,
  kPrismatic,
  kUnknown
};

struct UrdfJoint {
  std::string name;
  std::string parent;
  std::string child;
  UrdfJointType type = UrdfJointType::kUnknown;
  QVector3D origin;
  QQuaternion rotation;
  QVector3D axis{1.f, 0.f, 0.f};
};

class UrdfModel {
 public:
  bool loadFromString(const std::string& xml);
  bool loadFromFile(const std::string& path);

  const std::vector<UrdfLink>& links() const { return links_; }
  const std::vector<UrdfJoint>& joints() const { return joints_; }
  const std::string& rootLink() const { return root_link_; }
  const std::string& baseDirectory() const { return base_directory_; }
  bool empty() const { return links_.empty(); }

  static std::string resolveMeshPath(const std::string& base_directory,
                                     const std::string& filename);
  static std::string resolveTexturePath(const std::string& base_directory,
                                        const std::string& filename) {
    return resolveMeshPath(base_directory, filename);
  }

  std::unordered_map<std::string, QMatrix4x4> computeLinkTransforms(
      const std::unordered_map<std::string, double>& joint_positions) const;

 private:
  bool parseXml(const std::string& xml);

  std::vector<UrdfLink> links_;
  std::vector<UrdfJoint> joints_;
  std::string root_link_;
  std::string base_directory_;
  std::unordered_map<std::string, UrdfMaterial> material_library_;
};

}  // namespace display
}  // namespace autoviz
