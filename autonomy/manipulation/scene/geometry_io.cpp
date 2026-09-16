/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/scene/geometry_io.hpp"

#include <fstream>
#include <sstream>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace manipulation {
namespace scene {
namespace {

const char* ShapeName(ShapeType t) {
  switch (t) {
    case ShapeType::kBox:
      return "box";
    case ShapeType::kSphere:
      return "sphere";
    case ShapeType::kCylinder:
      return "cylinder";
    case ShapeType::kMesh:
      return "mesh";
    default:
      return "unknown";
  }
}

ShapeType ParseShape(const std::string& s) {
  if (s == "box") {
    return ShapeType::kBox;
  }
  if (s == "sphere") {
    return ShapeType::kSphere;
  }
  if (s == "cylinder") {
    return ShapeType::kCylinder;
  }
  if (s == "mesh") {
    return ShapeType::kMesh;
  }
  return ShapeType::kUnknown;
}

}  // namespace

bool SaveGeometryToFile(const PlanningScene& scene, const std::string& path) {
  std::ofstream out(path);
  if (!out) {
    AERROR << "SaveGeometryToFile: cannot open " << path;
    return false;
  }
  out << "# autonomy manipulation scene v1\n";
  const auto objects = scene.GetCollisionObjects();
  out << "objects " << objects.size() << "\n";
  for (const auto& obj : objects) {
    out << "object " << obj.id << " " << ShapeName(obj.type) << " " << obj.x
        << " " << obj.y << " " << obj.z << " " << obj.size_x << " "
        << obj.size_y << " " << obj.size_z << "\n";
  }
  const auto occupied = scene.OccupiedPoints();
  out << "occupied " << scene.OccupancyResolution() << " " << occupied.size()
      << "\n";
  for (const auto& p : occupied) {
    out << p.x << " " << p.y << " " << p.z << "\n";
  }
  return true;
}

bool LoadGeometryFromFile(PlanningScene* scene, const std::string& path) {
  if (!scene) {
    return false;
  }
  std::ifstream in(path);
  if (!in) {
    AERROR << "LoadGeometryFromFile: cannot open " << path;
    return false;
  }
  scene->ClearWorldObjects();
  scene->ClearOccupiedPoints();

  std::string line;
  std::vector<OccupiedPoint> occupied;
  double resolution = 0.05;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ss(line);
    std::string tag;
    ss >> tag;
    if (tag == "objects") {
      continue;
    }
    if (tag == "object") {
      CollisionObject obj;
      std::string type;
      ss >> obj.id >> type >> obj.x >> obj.y >> obj.z >> obj.size_x >>
          obj.size_y >> obj.size_z;
      obj.type = ParseShape(type);
      scene->AddCollisionObject(obj);
      continue;
    }
    if (tag == "occupied") {
      std::size_t n = 0;
      ss >> resolution >> n;
      occupied.clear();
      occupied.reserve(n);
      for (std::size_t i = 0; i < n; ++i) {
        if (!std::getline(in, line)) {
          break;
        }
        std::istringstream ps(line);
        OccupiedPoint p;
        ps >> p.x >> p.y >> p.z;
        occupied.push_back(p);
      }
      scene->SetOccupiedPoints(std::move(occupied), resolution);
      continue;
    }
  }
  return true;
}

}  // namespace scene
}  // namespace manipulation
}  // namespace autonomy
