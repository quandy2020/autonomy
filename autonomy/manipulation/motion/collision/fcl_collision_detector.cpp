/*
 * Copyright 2026 The Openbot Authors
 *
 * FCL full-chain robot–world / self collision (LinkFk + ACM + URDF convex).
 */

#include "autonomy/manipulation/motion/collision/fcl_collision_detector.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fcl/fcl.h>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/motion/collision/link_collision_geometry.hpp"
#include "autonomy/manipulation/motion/scene/collision_object_util.hpp"

namespace autonomy {
namespace manipulation {
namespace collision {
namespace {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

Vec3 EstimateEe(const core::JointState& state, double link_length) {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  for (double q : state.position()) {
    yaw += q;
    x += link_length * std::cos(yaw);
    y += link_length * std::sin(yaw);
  }
  return {x, y, 0.0};
}

fcl::Transform3d ToFcl(const core::Transform& t) {
  fcl::Transform3d tf = fcl::Transform3d::Identity();
  tf.translation() = fcl::Vector3d(t.x, t.y, t.z);
  tf.linear() = fcl::Quaterniond(t.qw, t.qx, t.qy, t.qz).toRotationMatrix();
  return tf;
}

core::Transform ComposeLocal(const core::Transform& link,
                             const core::Transform& local) {
  return core::Compose(link, local);
}

std::shared_ptr<fcl::CollisionGeometryd> MakeConvexGeom(
    const std::vector<MeshVertex>& verts,
    const std::vector<int>& faces) {
  // @p faces are triangle index triplets; FCL encodes each face as
  // [n, i0, ..., i_{n-1}].
  if (verts.size() < 4 || faces.size() < 9 || faces.size() % 3 != 0) {
    return nullptr;
  }
  auto pts = std::make_shared<std::vector<fcl::Vector3d>>();
  pts->reserve(verts.size());
  for (const auto& v : verts) {
    pts->emplace_back(v.x, v.y, v.z);
  }
  const int num_faces = static_cast<int>(faces.size() / 3);
  auto polys = std::make_shared<std::vector<int>>();
  polys->reserve(static_cast<std::size_t>(num_faces) * 4u);
  for (std::size_t i = 0; i + 2 < faces.size(); i += 3) {
    polys->push_back(3);
    polys->push_back(faces[i]);
    polys->push_back(faces[i + 1]);
    polys->push_back(faces[i + 2]);
  }
  return std::make_shared<fcl::Convexd>(pts, num_faces, polys, false);
}

std::shared_ptr<fcl::CollisionGeometryd> MakeBvhGeom(
    const std::vector<MeshVertex>& verts,
    const std::vector<int>& triangles) {
  if (verts.empty() || triangles.size() < 3 || triangles.size() % 3 != 0) {
    return nullptr;
  }
  auto model = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
  std::vector<fcl::Vector3d> points;
  points.reserve(verts.size());
  for (const auto& v : verts) {
    points.emplace_back(v.x, v.y, v.z);
  }
  std::vector<fcl::Triangle> tris;
  tris.reserve(triangles.size() / 3);
  for (std::size_t i = 0; i + 2 < triangles.size(); i += 3) {
    tris.emplace_back(triangles[i], triangles[i + 1], triangles[i + 2]);
  }
  if (model->beginModel() != fcl::BVH_OK) {
    return nullptr;
  }
  if (model->addSubModel(points, tris) != fcl::BVH_OK) {
    return nullptr;
  }
  if (model->endModel() != fcl::BVH_OK) {
    return nullptr;
  }
  return model;
}

fcl::CollisionObjectd MakeWorldObject(const scene::CollisionObject& obj) {
  using SP = automsgs::msgs::shape_msgs::SolidPrimitive;
  std::shared_ptr<fcl::CollisionGeometryd> geom;
  const auto pose = scene::GetObjectPose(obj);
  double sx = 0.0;
  double sy = 0.0;
  double sz = 0.0;
  scene::GetPrimitiveSizes(obj, &sx, &sy, &sz);
  if (scene::GetPrimitiveType(obj) == SP::SPHERE) {
    geom = std::make_shared<fcl::Sphered>(std::max(1e-4, sx));
  } else if (scene::GetPrimitiveType(obj) == SP::CYLINDER) {
    geom = std::make_shared<fcl::Cylinderd>(std::max(1e-4, sx),
                                            std::max(1e-4, sz));
  } else if (scene::HasMesh(obj) && obj.meshes(0).vertices_size() > 0) {
    std::vector<MeshVertex> verts;
    verts.reserve(static_cast<std::size_t>(obj.meshes(0).vertices_size()));
    for (const auto& v : obj.meshes(0).vertices()) {
      verts.push_back({v.x(), v.y(), v.z()});
    }
    std::vector<int> tris;
    for (const auto& t : obj.meshes(0).triangles()) {
      if (t.vertex_indices_size() >= 3) {
        tris.push_back(t.vertex_indices(0));
        tris.push_back(t.vertex_indices(1));
        tris.push_back(t.vertex_indices(2));
      }
    }
    if (!tris.empty()) {
      geom = MakeBvhGeom(verts, tris);
    }
    if (!geom) {
      std::vector<MeshVertex> hv;
      std::vector<int> hf;
      BuildConvexHull(verts, &hv, &hf);
      geom = MakeConvexGeom(hv, hf);
    }
    if (!geom) {
      geom = std::make_shared<fcl::Boxd>(std::max(1e-4, sx), std::max(1e-4, sy),
                                        std::max(1e-4, sz));
    }
  } else {
    geom = std::make_shared<fcl::Boxd>(std::max(1e-4, sx), std::max(1e-4, sy),
                                      std::max(1e-4, sz));
  }
  fcl::Transform3d tf = fcl::Transform3d::Identity();
  tf.translation() =
      fcl::Vector3d(pose.position().x(), pose.position().y(), pose.position().z());
  tf.linear() =
      fcl::Quaterniond(pose.orientation().w(), pose.orientation().x(),
                       pose.orientation().y(), pose.orientation().z())
          .toRotationMatrix();
  return fcl::CollisionObjectd(geom, tf);
}

fcl::CollisionObjectd MakeFromShape(const LinkCollisionShape& shape,
                                    const core::Transform& link_pose) {
  const core::Transform pose = ComposeLocal(link_pose, shape.origin);
  std::shared_ptr<fcl::CollisionGeometryd> geom;
  if (shape.kind == LinkShapeKind::kSphere) {
    geom = std::make_shared<fcl::Sphered>(std::max(1e-4, shape.size_x));
  } else if (shape.kind == LinkShapeKind::kCylinder) {
    geom = std::make_shared<fcl::Cylinderd>(std::max(1e-4, shape.size_x),
                                            std::max(1e-4, shape.size_z));
  } else if (shape.kind == LinkShapeKind::kMesh) {
    geom = MakeBvhGeom(shape.mesh_vertices, shape.mesh_triangles);
    if (!geom) {
      geom = MakeConvexGeom(shape.convex_vertices, shape.convex_faces);
    }
    if (!geom) {
      geom = std::make_shared<fcl::Sphered>(std::max(1e-4, 0.04));
    }
  } else if (shape.kind == LinkShapeKind::kConvex) {
    geom = MakeConvexGeom(shape.convex_vertices, shape.convex_faces);
    if (!geom) {
      geom = std::make_shared<fcl::Sphered>(std::max(1e-4, 0.04));
    }
  } else {
    geom = std::make_shared<fcl::Boxd>(std::max(1e-4, shape.size_x),
                                      std::max(1e-4, shape.size_y),
                                      std::max(1e-4, shape.size_z));
  }
  return fcl::CollisionObjectd(geom, ToFcl(pose));
}

fcl::CollisionObjectd MakeLinkSphere(const core::Transform& pose, double r) {
  auto geom = std::make_shared<fcl::Sphered>(std::max(1e-4, r));
  return fcl::CollisionObjectd(geom, ToFcl(pose));
}

bool PairAllowed(const scene::PlanningScene& scene, const std::string& a,
                 const std::string& b) {
  return scene.IsCollisionAllowed(a, b);
}

bool CollidePair(const fcl::CollisionObjectd& a, const fcl::CollisionObjectd& b,
                 CollisionResult* result, const std::string& name_a,
                 const std::string& name_b, double contact_distance) {
  fcl::CollisionRequestd req;
  req.num_max_contacts = 1;
  if (contact_distance > 1e-9) {
    // Treat near-contacts within padding as collisions (MoveIt padding).
    req.enable_contact = true;
  }
  fcl::CollisionResultd res;
  fcl::collide(&a, &b, req, res);
  if (res.isCollision()) {
    result->collision = true;
    result->contact_body_a = name_a;
    result->contact_body_b = name_b;
    return true;
  }
  if (contact_distance > 1e-9) {
    fcl::DistanceRequestd dreq;
    dreq.enable_nearest_points = false;
    fcl::DistanceResultd dres;
    const double d = fcl::distance(&a, &b, dreq, dres);
    if (d >= 0.0 && d <= contact_distance) {
      result->collision = true;
      result->contact_body_a = name_a;
      result->contact_body_b = name_b;
      return true;
    }
  }
  return false;
}

CollisionResult CheckEeProxyWorld(const core::JointState& state,
                                  const scene::PlanningScene& scene,
                                  double link_length, double ee_radius,
                                  double padding) {
  CollisionResult result;
  const Vec3 ee = EstimateEe(state, link_length);
  auto sphere = std::make_shared<fcl::Sphered>(ee_radius + padding);
  fcl::Transform3d ee_tf = fcl::Transform3d::Identity();
  ee_tf.translation() = fcl::Vector3d(ee.x, ee.y, ee.z);
  fcl::CollisionObjectd ee_obj(sphere, ee_tf);

  for (const auto& obj : scene.GetCollisionObjects()) {
    fcl::CollisionObjectd world = MakeWorldObject(obj);
    if (CollidePair(ee_obj, world, &result, "ee", obj.id(), padding)) {
      return result;
    }
  }
  return result;
}

std::vector<std::pair<std::string, fcl::CollisionObjectd>> BuildRobotBodies(
    const std::unordered_map<std::string, core::Transform>& poses,
    const LinkCollisionModel* shapes, double link_radius) {
  std::vector<std::pair<std::string, fcl::CollisionObjectd>> bodies;
  if (shapes && !shapes->empty()) {
    for (const auto& s : shapes->Shapes()) {
      const auto it = poses.find(s.link_name);
      if (it == poses.end()) {
        continue;
      }
      bodies.emplace_back(s.link_name, MakeFromShape(s, it->second));
    }
    if (!bodies.empty()) {
      return bodies;
    }
  }
  for (const auto& kv : poses) {
    bodies.emplace_back(kv.first, MakeLinkSphere(kv.second, link_radius));
  }
  return bodies;
}

}  // namespace

bool FclCollisionDetector::Init(const std::string& id) {
  id_ = id.empty() ? "fcl" : id;
  AINFO << "FclCollisionDetector ready id=" << id_
        << " full_chain=" << (link_tree_ ? "yes" : "ee_proxy")
        << " urdf_shapes="
        << (link_shapes_ && !link_shapes_->empty() ? "yes" : "no");
  return true;
}

CollisionResult FclCollisionDetector::CheckRobotWorld(
    const core::JointState& state,
    const scene::PlanningScene& scene) const {
  if (!link_tree_) {
    return CheckEeProxyWorld(state, scene, link_length_, ee_radius_, padding_);
  }

  CollisionResult result;
  std::unordered_map<std::string, core::Transform> poses;
  if (!link_tree_->Compute(state, &poses) || poses.empty()) {
    return CheckEeProxyWorld(state, scene, link_length_, ee_radius_, padding_);
  }

  const auto bodies =
      BuildRobotBodies(poses, link_shapes_.get(), link_radius_ + padding_);
  const auto world = scene.GetCollisionObjects();
  for (const auto& body : bodies) {
    for (const auto& obj : world) {
      if (PairAllowed(scene, body.first, obj.id())) {
        continue;
      }
      fcl::CollisionObjectd world_obj = MakeWorldObject(obj);
      if (CollidePair(body.second, world_obj, &result, body.first, obj.id(),
                      padding_)) {
        return result;
      }
    }

    const double res_r = std::max(1e-3, 0.5 * scene.OccupancyResolution());
    for (const auto& p : scene.OccupiedPoints()) {
      auto sph = std::make_shared<fcl::Sphered>(res_r);
      fcl::Transform3d tf = fcl::Transform3d::Identity();
      tf.translation() = fcl::Vector3d(p.x, p.y, p.z);
      fcl::CollisionObjectd occ(sph, tf);
      if (CollidePair(body.second, occ, &result, body.first, "occupancy",
                      padding_)) {
        return result;
      }
    }
  }
  return result;
}

CollisionResult FclCollisionDetector::CheckRobotSelf(
    const core::JointState& state,
    const scene::PlanningScene* scene) const {
  CollisionResult result;
  if (!link_tree_) {
    return result;
  }
  std::unordered_map<std::string, core::Transform> poses;
  if (!link_tree_->Compute(state, &poses) || poses.size() < 2) {
    return result;
  }

  std::unordered_set<std::string> adjacent;
  for (const auto& j : link_tree_->Joints()) {
    adjacent.insert(j.parent + "|" + j.child);
    adjacent.insert(j.child + "|" + j.parent);
  }

  const auto bodies =
      BuildRobotBodies(poses, link_shapes_.get(), link_radius_ + padding_);
  for (std::size_t i = 0; i < bodies.size(); ++i) {
    for (std::size_t j = i + 1; j < bodies.size(); ++j) {
      if (bodies[i].first == bodies[j].first) {
        continue;
      }
      if (adjacent.count(bodies[i].first + "|" + bodies[j].first)) {
        continue;
      }
      if (scene &&
          scene->IsCollisionAllowed(bodies[i].first, bodies[j].first)) {
        continue;
      }
      if (CollidePair(bodies[i].second, bodies[j].second, &result,
                      bodies[i].first, bodies[j].first, padding_)) {
        return result;
      }
    }
  }
  return result;
}

DistanceResult FclCollisionDetector::DistanceRobotWorld(
    const core::JointState& state,
    const scene::PlanningScene& scene) const {
  DistanceResult best;
  best.distance = 1e9;
  std::unordered_map<std::string, core::Transform> poses;
  std::vector<std::pair<std::string, fcl::CollisionObjectd>> bodies;
  if (link_tree_ && link_tree_->Compute(state, &poses) && !poses.empty()) {
    bodies = BuildRobotBodies(poses, link_shapes_.get(),
                              link_radius_ + padding_);
  } else {
    const Vec3 ee = EstimateEe(state, link_length_);
    auto sphere = std::make_shared<fcl::Sphered>(ee_radius_ + padding_);
    fcl::Transform3d ee_tf = fcl::Transform3d::Identity();
    ee_tf.translation() = fcl::Vector3d(ee.x, ee.y, ee.z);
    bodies.emplace_back("ee", fcl::CollisionObjectd(sphere, ee_tf));
  }

  for (const auto& body : bodies) {
    for (const auto& obj : scene.GetCollisionObjects()) {
      if (PairAllowed(scene, body.first, obj.id())) {
        continue;
      }
      fcl::CollisionObjectd world_obj = MakeWorldObject(obj);
      fcl::DistanceRequestd dreq;
      dreq.enable_nearest_points = true;
      fcl::DistanceResultd dres;
      const double d = fcl::distance(&body.second, &world_obj, dreq, dres);
      if (d < best.distance) {
        best.distance = d;
        best.nearest_body_a = body.first;
        best.nearest_body_b = obj.id();
        if (dres.min_distance < 1e9) {
          best.nearest_x = dres.nearest_points[0][0];
          best.nearest_y = dres.nearest_points[0][1];
          best.nearest_z = dres.nearest_points[0][2];
        }
      }
    }

    const double res_r = std::max(1e-3, 0.5 * scene.OccupancyResolution());
    for (const auto& p : scene.OccupiedPoints()) {
      auto sph = std::make_shared<fcl::Sphered>(res_r);
      fcl::Transform3d tf = fcl::Transform3d::Identity();
      tf.translation() = fcl::Vector3d(p.x, p.y, p.z);
      fcl::CollisionObjectd occ(sph, tf);
      fcl::DistanceRequestd dreq;
      dreq.enable_nearest_points = true;
      fcl::DistanceResultd dres;
      const double d = fcl::distance(&body.second, &occ, dreq, dres);
      if (d < best.distance) {
        best.distance = d;
        best.nearest_body_a = body.first;
        best.nearest_body_b = "occupancy";
        if (dres.min_distance < 1e9) {
          best.nearest_x = dres.nearest_points[0][0];
          best.nearest_y = dres.nearest_points[0][1];
          best.nearest_z = dres.nearest_points[0][2];
        }
      }
    }
  }
  best.collision = best.distance <= padding_;
  if (best.collision && best.distance > 0.0) {
    // Within padding band → treat as contact for planning.
    best.distance = 0.0;
  }
  return best;
}

std::shared_ptr<CollisionDetector> CreateFclCollisionDetector() {
  return std::make_shared<FclCollisionDetector>();
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(FclCollisionDetector, CollisionDetector);

}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
