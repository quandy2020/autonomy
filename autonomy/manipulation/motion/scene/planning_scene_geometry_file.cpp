/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/scene/planning_scene_geometry_file.hpp"

#include <fstream>
#include <sstream>

#include <automsgs/msgs/shape_msgs/solid_primitive.pb.h>

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/motion/scene/collision_object_helpers.hpp"

namespace autonomy {
namespace manipulation {
namespace scene {
namespace {

const char* ShapeName(automsgs::msgs::shape_msgs::SolidPrimitive::Type t) {
  using SP = automsgs::msgs::shape_msgs::SolidPrimitive;
  switch (t) {
    case SP::BOX:
      return "box";
    case SP::SPHERE:
      return "sphere";
    case SP::CYLINDER:
      return "cylinder";
    default:
      return "unknown";
  }
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
    const automsgs::msgs::geometry_msgs::Pose pose = GetObjectPose(obj);
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;
    GetPrimitiveSizes(obj, &sx, &sy, &sz);
    const char* shape = HasMesh(obj) && !HasPrimitive(obj)
                            ? "mesh"
                            : ShapeName(GetPrimitiveType(obj));
    out << "object " << obj.id() << " " << shape << " " << pose.position().x()
        << " " << pose.position().y() << " " << pose.position().z() << " " << sx
        << " " << sy << " " << sz << "\n";
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
      std::string id;
      std::string type;
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      double sx = 0.0;
      double sy = 0.0;
      double sz = 0.0;
      ss >> id >> type >> x >> y >> z >> sx >> sy >> sz;
      automsgs::msgs::moveit_msgs::CollisionObject obj;
      if (type == "sphere") {
        obj = MakeSphereObject(id, x, y, z, sx);
      } else if (type == "cylinder") {
        obj = MakeCylinderObject(id, x, y, z, sx, sz);
      } else {
        obj = MakeBoxObject(id, x, y, z, sx, sy, sz);
      }
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
