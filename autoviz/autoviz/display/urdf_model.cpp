/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/urdf_model.hpp"

#include <QDir>
#include <QDomDocument>
#include <QFile>
#include <QFileInfo>
#include <algorithm>
#include <cmath>

#include "autoviz/common/path_env_utils.hpp"

namespace autoviz {
namespace display {
namespace {

QVector3D ParseXyz(const QString& text) {
  const QStringList parts = text.simplified().split(' ', Qt::SkipEmptyParts);
  if (parts.size() < 3) {
    return {};
  }
  return QVector3D(parts[0].toFloat(), parts[1].toFloat(), parts[2].toFloat());
}

QQuaternion ParseRpy(const QString& text) {
  const QVector3D rpy = ParseXyz(text);
  return QQuaternion::fromEulerAngles(rpy.x(), rpy.y(), rpy.z());
}

QMatrix4x4 JointOriginMatrix(const UrdfJoint& joint) {
  QMatrix4x4 matrix;
  matrix.setToIdentity();
  matrix.translate(joint.origin);
  matrix.rotate(joint.rotation());
  return matrix;
}

QMatrix4x4 JointMotionMatrix(const UrdfJoint& joint, double position) {
  QMatrix4x4 matrix;
  matrix.setToIdentity();
  if (joint.type == UrdfJointType::kPrismatic) {
    matrix.translate(joint.axis * static_cast<float>(position));
  } else if (joint.type == UrdfJointType::kRevolute ||
             joint.type == UrdfJointType::kContinuous) {
    matrix.rotate(static_cast<float>(qRadiansToDegrees(position)), joint.axis);
  }
  return matrix;
}

UrdfJointType ParseJointType(const QString& type) {
  if (type == QLatin1String("fixed")) {
    return UrdfJointType::kFixed;
  }
  if (type == QLatin1String("revolute")) {
    return UrdfJointType::kRevolute;
  }
  if (type == QLatin1String("continuous")) {
    return UrdfJointType::kContinuous;
  }
  if (type == QLatin1String("prismatic")) {
    return UrdfJointType::kPrismatic;
  }
  return UrdfJointType::kUnknown;
}

UrdfGeometry ParseGeometry(const QDomElement& geometry_node) {
  UrdfGeometry geometry;
  const QDomElement box = geometry_node.firstChildElement(QStringLiteral("box"));
  if (!box.isNull()) {
    geometry.type = UrdfGeometry::Type::kBox;
    geometry.size = ParseXyz(box.attribute(QStringLiteral("size")));
    return geometry;
  }
  const QDomElement cylinder =
      geometry_node.firstChildElement(QStringLiteral("cylinder"));
  if (!cylinder.isNull()) {
    geometry.type = UrdfGeometry::Type::kCylinder;
    const float radius = cylinder.attribute(QStringLiteral("radius")).toFloat();
    const float length = cylinder.attribute(QStringLiteral("length")).toFloat();
    geometry.size = QVector3D(radius, length, radius);
    return geometry;
  }
  const QDomElement sphere =
      geometry_node.firstChildElement(QStringLiteral("sphere"));
  if (!sphere.isNull()) {
    geometry.type = UrdfGeometry::Type::kSphere;
    const float radius = sphere.attribute(QStringLiteral("radius")).toFloat();
    geometry.size = QVector3D(radius, radius, radius);
    return geometry;
  }
  const QDomElement mesh = geometry_node.firstChildElement(QStringLiteral("mesh"));
  if (!mesh.isNull()) {
    geometry.type = UrdfGeometry::Type::kMesh;
    geometry.mesh_filename =
        mesh.attribute(QStringLiteral("filename")).toStdString();
    const QString scale_attr = mesh.attribute(QStringLiteral("scale"));
    if (!scale_attr.isEmpty()) {
      geometry.mesh_scale = ParseXyz(scale_attr);
    }
    return geometry;
  }
  return geometry;
}

UrdfMaterial ParseMaterialDefinition(const QDomElement& material_node) {
  UrdfMaterial material;
  if (material_node.isNull()) {
    return material;
  }
  const QDomElement color =
      material_node.firstChildElement(QStringLiteral("color"));
  if (!color.isNull()) {
    const QStringList rgba =
        color.attribute(QStringLiteral("rgba")).simplified().split(
            ' ', Qt::SkipEmptyParts);
    if (rgba.size() >= 4) {
      material.r = rgba[0].toFloat();
      material.g = rgba[1].toFloat();
      material.b = rgba[2].toFloat();
      material.a = rgba[3].toFloat();
      material.valid = true;
      const float lum = 0.2126f * material.r + 0.7152f * material.g +
                        0.0722f * material.b;
      material.metallic = std::clamp(lum * 0.2f, 0.02f, 0.85f);
      material.roughness = std::clamp(0.78f - lum * 0.4f, 0.15f, 0.95f);
    }
  }
  const QDomElement texture =
      material_node.firstChildElement(QStringLiteral("texture"));
  if (!texture.isNull()) {
    material.texture_filename =
        texture.attribute(QStringLiteral("filename")).toStdString();
    material.has_texture = !material.texture_filename.empty();
    if (material.has_texture) {
      material.valid = true;
    }
  }
  return material;
}

UrdfMaterial ParseMaterial(
    const QDomElement& parent,
    const std::unordered_map<std::string, UrdfMaterial>& material_library) {
  UrdfMaterial material;
  const QDomElement material_node =
      parent.firstChildElement(QStringLiteral("material"));
  if (material_node.isNull()) {
    return material;
  }
  const QString material_name = material_node.attribute(QStringLiteral("name"));
  const bool has_inline_color =
      !material_node.firstChildElement(QStringLiteral("color")).isNull();
  const bool has_inline_texture =
      !material_node.firstChildElement(QStringLiteral("texture")).isNull();
  if (!material_name.isEmpty() && !has_inline_color && !has_inline_texture) {
    const auto found = material_library.find(material_name.toStdString());
    if (found != material_library.end()) {
      return found->second;
    }
  }
  return ParseMaterialDefinition(material_node);
}

UrdfGeometry ParseLinkGeometry(
    const QDomElement& element,
    const std::unordered_map<std::string, UrdfMaterial>& material_library) {
  UrdfGeometry geometry;
  if (element.isNull()) {
    return geometry;
  }
  const QDomElement origin = element.firstChildElement(QStringLiteral("origin"));
  if (!origin.isNull()) {
    *geometry.mutable_origin() = ParseXyz(origin.attribute(QStringLiteral("xyz")));
    *geometry.mutable_rotation() = ParseRpy(origin.attribute(QStringLiteral("rpy")));
  }
  const QDomElement geometry_node =
      element.firstChildElement(QStringLiteral("geometry"));
  if (!geometry_node.isNull()) {
    geometry = ParseGeometry(geometry_node);
    *geometry.mutable_origin() = geometry.origin;
    *geometry.mutable_rotation() = geometry.rotation();
  }
  geometry.material = ParseMaterial(element, material_library);
  return geometry;
}

}  // namespace

QStringList PackageSearchPrefixes(const std::string& base_directory) {
  return common::resourceSearchPaths(base_directory);
}

std::string UrdfModel::resolveMeshPath(const std::string& base_directory,
                                       const std::string& filename) {
  if (filename.empty()) {
    return {};
  }
  QString path = QString::fromStdString(filename);
  if (path.startsWith(QLatin1String("file://"))) {
    return path.mid(7).toStdString();
  }
  if (path.startsWith(QLatin1String("package://"))) {
    const QString rest = path.mid(10);
    const int slash = rest.indexOf(QLatin1Char('/'));
    const QString package_name = slash > 0 ? rest.left(slash) : rest;
    const QString relative_path = slash > 0 ? rest.mid(slash + 1) : QString();

    const QStringList prefixes = PackageSearchPrefixes(base_directory);
    for (const QString& prefix : prefixes) {
      const QString candidate =
          QDir(prefix).filePath(QStringLiteral("share/%1/%2")
                                    .arg(package_name, relative_path));
      if (QFileInfo(candidate).exists()) {
        return candidate.toStdString();
      }
    }

    if (!relative_path.isEmpty()) {
      path = relative_path;
    }
  }
  const QFileInfo info(path);
  if (info.isAbsolute()) {
    return info.absoluteFilePath().toStdString();
  }
  return QDir(QString::fromStdString(base_directory))
      .filePath(path)
      .toStdString();
}

bool UrdfModel::loadFromFile(const std::string& path) {
  QFile file(QString::fromStdString(path));
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return false;
  }
  base_directory_ = QFileInfo(file).absolutePath().toStdString();
  return parseXml(file.readAll().toStdString());
}

bool UrdfModel::loadFromString(const std::string& xml) {
  return parseXml(xml);
}

bool UrdfModel::parseXml(const std::string& xml) {
  links_.clear();
  joints_.clear();
  root_link_.clear();
  material_library_.clear();
  if (base_directory_.empty()) {
    base_directory_ = QDir::currentPath().toStdString();
  }

  QDomDocument document;
  if (!document.setContent(QString::fromStdString(xml))) {
    return false;
  }

  const QDomElement robot = document.documentElement();
  if (robot.isNull() || robot.tagName() != QLatin1String("robot")) {
    return false;
  }

  for (auto material_node = robot.firstChildElement(QStringLiteral("material"));
       !material_node.isNull();
       material_node = material_node.nextSiblingElement(QStringLiteral("material"))) {
    const std::string name =
        material_node.attribute(QStringLiteral("name")).toStdString();
    if (name.empty()) {
      continue;
    }
    material_library_[name] = ParseMaterialDefinition(material_node);
  }

  std::unordered_map<std::string, bool> child_links;
  for (auto link_node = robot.firstChildElement(QStringLiteral("link"));
       !link_node.isNull();
       link_node = link_node.nextSiblingElement(QStringLiteral("link"))) {
    UrdfLink link;
    link.name = link_node.attribute(QStringLiteral("name")).toStdString();
    const QDomElement visual =
        link_node.firstChildElement(QStringLiteral("visual"));
    if (!visual.isNull()) {
      link.visual = ParseLinkGeometry(visual, material_library_);
      link.has_visual = link.visual.type != UrdfGeometry::Type::kUnknown;
    }
    const QDomElement collision =
        link_node.firstChildElement(QStringLiteral("collision"));
    if (!collision.isNull()) {
      link.collision = ParseLinkGeometry(collision, material_library_);
      link.has_collision = link.collision.type != UrdfGeometry::Type::kUnknown;
    }
    links_.push_back(std::move(link));
  }

  for (auto joint_node = robot.firstChildElement(QStringLiteral("joint"));
       !joint_node.isNull();
       joint_node = joint_node.nextSiblingElement(QStringLiteral("joint"))) {
    UrdfJoint joint;
    joint.name = joint_node.attribute(QStringLiteral("name")).toStdString();
    joint.type = ParseJointType(joint_node.attribute(QStringLiteral("type")));
    joint.parent = joint_node.firstChildElement(QStringLiteral("parent"))
                       .attribute(QStringLiteral("link"))
                       .toStdString();
    joint.child = joint_node.firstChildElement(QStringLiteral("child"))
                      .attribute(QStringLiteral("link"))
                      .toStdString();
    child_links[joint.child] = true;
    const QDomElement origin =
        joint_node.firstChildElement(QStringLiteral("origin"));
    if (!origin.isNull()) {
      *joint.mutable_origin() = ParseXyz(origin.attribute(QStringLiteral("xyz")));
      *joint.mutable_rotation() = ParseRpy(origin.attribute(QStringLiteral("rpy")));
    }
    const QDomElement axis =
        joint_node.firstChildElement(QStringLiteral("axis"));
    if (!axis.isNull()) {
      joint.axis = ParseXyz(axis.attribute(QStringLiteral("xyz")));
      if (joint.axis.lengthSquared() < 1e-6f) {
        joint.axis = QVector3D(1.f, 0.f, 0.f);
      } else {
        joint.axis.normalize();
      }
    }
    joints_.push_back(std::move(joint));
  }

  for (const auto& link : links_) {
    if (!child_links.count(link.name())) {
      root_link_ = link.name();
      break;
    }
  }
  if (root_link_.empty() && !links_.empty()) {
    root_link_ = links_.front().name;
  }
  return !links_.empty();
}

std::unordered_map<std::string, QMatrix4x4> UrdfModel::computeLinkTransforms(
    const std::unordered_map<std::string, double>& joint_positions) const {
  std::unordered_map<std::string, QMatrix4x4> transforms;
  if (root_link_.empty()) {
    return transforms;
  }

  transforms[root_link_].setToIdentity();

  bool progress = true;
  while (progress) {
    progress = false;
    for (const auto& joint : joints_) {
      if (transforms.count(joint.parent) == 0 ||
          transforms.count(joint.child) != 0) {
        continue;
      }
      double position = 0.0;
      const auto it = joint_positions.find(joint.name());
      if (it != joint_positions.end()) {
        position = it->second;
      }
      transforms[joint.child] =
          transforms[joint.parent] * JointOriginMatrix(joint) *
          JointMotionMatrix(joint, position);
      progress = true;
    }
  }
  return transforms;
}

}  // namespace display
}  // namespace autoviz
