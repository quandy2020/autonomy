/*
 * Copyright 2026 The Openbot Authors
 *
 * Helpers for moveit_msgs::CollisionObject (primitives / meshes / poses).
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/shape_msgs/solid_primitive.pb.h>

#include "autonomy/manipulation/common/msg_types.hpp"
#include "autonomy/manipulation/model/link_fk.hpp"

namespace autonomy {
namespace manipulation {
namespace scene {

/** @brief Alias: moveit_msgs/CollisionObject (canonical scene object). */
using CollisionObject = CollisionObjectMsg;

/** @brief Alias: moveit_msgs/AttachedCollisionObject. */
using AttachedCollisionObject = AttachedCollisionObjectMsg;

/** @brief Alias: moveit_msgs/AllowedCollisionMatrix. */
using AllowedCollisionMatrix = AllowedCollisionMatrixMsg;

namespace detail {

inline void ComposeQuat(double aw, double ax, double ay, double az, double bw,
                        double bx, double by, double bz, double* ow, double* ox,
                        double* oy, double* oz) {
  *ow = aw * bw - ax * bx - ay * by - az * bz;
  *ox = aw * bx + ax * bw + ay * bz - az * by;
  *oy = aw * by - ax * bz + ay * bw + az * bx;
  *oz = aw * bz + ax * by - ay * bx + az * bw;
}

inline void RotateVec(const core::Transform& t, double x, double y, double z,
                      double* ox, double* oy, double* oz) {
  const double qw = t.qw;
  const double qx = t.qx;
  const double qy = t.qy;
  const double qz = t.qz;
  const double ix = qw * x + qy * z - qz * y;
  const double iy = qw * y + qz * x - qx * z;
  const double iz = qw * z + qx * y - qy * x;
  const double iw = -qx * x - qy * y - qz * z;
  *ox = ix * qw + iw * -qx + iy * -qz - iz * -qy;
  *oy = iy * qw + iw * -qy + iz * -qx - ix * -qz;
  *oz = iz * qw + iw * -qz + ix * -qy - iy * -qx;
}

}  // namespace detail

/**
 * @brief Primary pose of a collision object (primitive_poses[0], mesh_poses[0], or pose).
 */
inline Pose GetObjectPose(const CollisionObject& object) {
  if (object.primitive_poses_size() > 0) {
    return object.primitive_poses(0);
  }
  if (object.mesh_poses_size() > 0) {
    return object.mesh_poses(0);
  }
  return object.pose();
}

/** @brief Write @p pose into primitive_poses[0] (creates if needed) and clear top-level pose. */
inline void SetObjectPose(CollisionObject* object, const Pose& pose) {
  if (!object) {
    return;
  }
  if (object->primitive_poses_size() == 0) {
    *object->add_primitive_poses() = pose;
  } else {
    *object->mutable_primitive_poses(0) = pose;
  }
  if (object->mesh_poses_size() > 0) {
    *object->mutable_mesh_poses(0) = pose;
  }
  *object->mutable_pose() = pose;
}

/** @brief Whether the object carries at least one solid primitive. */
inline bool HasPrimitive(const CollisionObject& object) {
  return object.primitives_size() > 0;
}

/** @brief Whether the object carries at least one mesh. */
inline bool HasMesh(const CollisionObject& object) {
  return object.meshes_size() > 0;
}

/** @brief First primitive type, or TYPE_UNKNOWN. */
inline automsgs::msgs::shape_msgs::SolidPrimitive::Type GetPrimitiveType(
    const CollisionObject& object) {
  if (!HasPrimitive(object)) {
    return automsgs::msgs::shape_msgs::SolidPrimitive::TYPE_UNKNOWN;
  }
  return object.primitives(0).type();
}

/**
 * @brief Box extents / sphere radius / cylinder (height, radius) from first primitive.
 * @return false if no usable primitive (caller may fall back to mesh AABB).
 */
inline bool GetPrimitiveSizes(const CollisionObject& object, double* size_x,
                              double* size_y, double* size_z) {
  if (!size_x || !size_y || !size_z || !HasPrimitive(object)) {
    return false;
  }
  const auto& prim = object.primitives(0);
  *size_x = 0.0;
  *size_y = 0.0;
  *size_z = 0.0;
  using SP = automsgs::msgs::shape_msgs::SolidPrimitive;
  if (prim.type() == SP::SPHERE) {
    *size_x = prim.dimensions_size() > 0 ? prim.dimensions(0) : 0.0;
    return true;
  }
  if (prim.type() == SP::CYLINDER) {
    // ROS: dimensions[0]=height, dimensions[1]=radius.
    *size_z = prim.dimensions_size() > 0 ? prim.dimensions(0) : 0.0;
    *size_x = prim.dimensions_size() > 1 ? prim.dimensions(1) : 0.0;
    *size_y = *size_x;
    return true;
  }
  // BOX (default): X Y Z
  *size_x = prim.dimensions_size() > 0 ? prim.dimensions(0) : 0.0;
  *size_y = prim.dimensions_size() > 1 ? prim.dimensions(1) : 0.0;
  *size_z = prim.dimensions_size() > 2 ? prim.dimensions(2) : 0.0;
  return true;
}

/** @brief Build a box CollisionObject at (@p x,@p y,@p z) with extents. */
inline CollisionObject MakeBoxObject(const std::string& id, double x, double y,
                                     double z, double sx, double sy, double sz) {
  CollisionObject obj;
  obj.set_id(id);
  obj.set_operation(CollisionObjectMsg::ADD);
  auto* prim = obj.add_primitives();
  prim->set_type(automsgs::msgs::shape_msgs::SolidPrimitive::BOX);
  prim->add_dimensions(sx);
  prim->add_dimensions(sy);
  prim->add_dimensions(sz);
  auto* pose = obj.add_primitive_poses();
  pose->mutable_position()->set_x(x);
  pose->mutable_position()->set_y(y);
  pose->mutable_position()->set_z(z);
  pose->mutable_orientation()->set_w(1.0);
  *obj.mutable_pose() = *pose;
  return obj;
}

/** @brief Build a sphere CollisionObject (radius @p r). */
inline CollisionObject MakeSphereObject(const std::string& id, double x, double y,
                                        double z, double r) {
  CollisionObject obj;
  obj.set_id(id);
  obj.set_operation(CollisionObjectMsg::ADD);
  auto* prim = obj.add_primitives();
  prim->set_type(automsgs::msgs::shape_msgs::SolidPrimitive::SPHERE);
  prim->add_dimensions(r);
  auto* pose = obj.add_primitive_poses();
  pose->mutable_position()->set_x(x);
  pose->mutable_position()->set_y(y);
  pose->mutable_position()->set_z(z);
  pose->mutable_orientation()->set_w(1.0);
  *obj.mutable_pose() = *pose;
  return obj;
}

/** @brief Build an upright cylinder (radius @p r, height @p h). */
inline CollisionObject MakeCylinderObject(const std::string& id, double x,
                                          double y, double z, double r,
                                          double h) {
  CollisionObject obj;
  obj.set_id(id);
  obj.set_operation(CollisionObjectMsg::ADD);
  auto* prim = obj.add_primitives();
  prim->set_type(automsgs::msgs::shape_msgs::SolidPrimitive::CYLINDER);
  prim->add_dimensions(h);
  prim->add_dimensions(r);
  auto* pose = obj.add_primitive_poses();
  pose->mutable_position()->set_x(x);
  pose->mutable_position()->set_y(y);
  pose->mutable_position()->set_z(z);
  pose->mutable_orientation()->set_w(1.0);
  *obj.mutable_pose() = *pose;
  return obj;
}

/**
 * @brief Fill a BOX primitive AABB from mesh vertices (object-local).
 * No-op if meshes empty.
 */
inline void UpdateMeshAabb(CollisionObject* object) {
  if (!object || object->meshes_size() == 0 ||
      object->meshes(0).vertices_size() == 0) {
    return;
  }
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double min_z = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();
  for (const auto& v : object->meshes(0).vertices()) {
    min_x = std::min(min_x, v.x());
    min_y = std::min(min_y, v.y());
    min_z = std::min(min_z, v.z());
    max_x = std::max(max_x, v.x());
    max_y = std::max(max_y, v.y());
    max_z = std::max(max_z, v.z());
  }
  object->clear_primitives();
  object->clear_primitive_poses();
  auto* prim = object->add_primitives();
  prim->set_type(automsgs::msgs::shape_msgs::SolidPrimitive::BOX);
  prim->add_dimensions(std::max(1e-4, max_x - min_x));
  prim->add_dimensions(std::max(1e-4, max_y - min_y));
  prim->add_dimensions(std::max(1e-4, max_z - min_z));
  Pose pose;
  pose.mutable_position()->set_x(0.5 * (min_x + max_x));
  pose.mutable_position()->set_y(0.5 * (min_y + max_y));
  pose.mutable_position()->set_z(0.5 * (min_z + max_z));
  pose.mutable_orientation()->set_w(1.0);
  *object->add_primitive_poses() = pose;
  if (object->mesh_poses_size() == 0) {
    *object->add_mesh_poses() = pose;
  }
  *object->mutable_pose() = pose;
}

/**
 * @brief Pose @p object from link frame into world using @p link_tf.
 * Transforms all primitive_poses / mesh_poses / pose.
 */
inline CollisionObject TransformAttached(const CollisionObject& object,
                                         const core::Transform& link_tf) {
  CollisionObject out = object;
  auto transform_pose = [&](Pose* pose) {
    if (!pose) {
      return;
    }
    double wx = 0.0;
    double wy = 0.0;
    double wz = 0.0;
    detail::RotateVec(link_tf, pose->position().x(), pose->position().y(),
                      pose->position().z(), &wx, &wy, &wz);
    pose->mutable_position()->set_x(link_tf.x + wx);
    pose->mutable_position()->set_y(link_tf.y + wy);
    pose->mutable_position()->set_z(link_tf.z + wz);
    double ow = 0.0;
    double ox = 0.0;
    double oy = 0.0;
    double oz = 0.0;
    detail::ComposeQuat(link_tf.qw, link_tf.qx, link_tf.qy, link_tf.qz,
                        pose->orientation().w(), pose->orientation().x(),
                        pose->orientation().y(), pose->orientation().z(), &ow,
                        &ox, &oy, &oz);
    pose->mutable_orientation()->set_w(ow);
    pose->mutable_orientation()->set_x(ox);
    pose->mutable_orientation()->set_y(oy);
    pose->mutable_orientation()->set_z(oz);
  };
  for (int i = 0; i < out.primitive_poses_size(); ++i) {
    transform_pose(out.mutable_primitive_poses(i));
  }
  for (int i = 0; i < out.mesh_poses_size(); ++i) {
    transform_pose(out.mutable_mesh_poses(i));
  }
  if (out.has_pose()) {
    transform_pose(out.mutable_pose());
  }
  return out;
}

/** @brief Translate object origin by (@p dx,@p dy,@p dz) in world. */
inline void TranslateObject(CollisionObject* object, double dx, double dy,
                            double dz) {
  if (!object) {
    return;
  }
  auto shift = [&](Pose* pose) {
    if (!pose) {
      return;
    }
    pose->mutable_position()->set_x(pose->position().x() + dx);
    pose->mutable_position()->set_y(pose->position().y() + dy);
    pose->mutable_position()->set_z(pose->position().z() + dz);
  };
  for (int i = 0; i < object->primitive_poses_size(); ++i) {
    shift(object->mutable_primitive_poses(i));
  }
  for (int i = 0; i < object->mesh_poses_size(); ++i) {
    shift(object->mutable_mesh_poses(i));
  }
  if (object->has_pose()) {
    shift(object->mutable_pose());
  }
}

}  // namespace scene
}  // namespace manipulation
}  // namespace autonomy
