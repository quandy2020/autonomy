/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/scene/msg_convert.hpp"

namespace autonomy {
namespace manipulation {
namespace scene {
namespace {

void SetDuration(double seconds,
                 automsgs::msgs::builtin_interfaces::Duration* duration) {
  if (!duration) {
    return;
  }
  const int64_t sec = static_cast<int64_t>(seconds);
  const int32_t nanosec =
      static_cast<int32_t>((seconds - static_cast<double>(sec)) * 1e9);
  duration->set_sec(sec);
  duration->set_nanosec(nanosec);
}

double DurationSeconds(
    const automsgs::msgs::builtin_interfaces::Duration& duration) {
  return static_cast<double>(duration.sec()) +
         static_cast<double>(duration.nanosec()) * 1e-9;
}

}  // namespace

automsgs::msgs::sensor_msgs::JointState ToMsg(const core::JointState& state) {
  automsgs::msgs::sensor_msgs::JointState msg;
  for (const auto& name : state.names) {
    msg.add_name(name);
  }
  for (double q : state.positions) {
    msg.add_position(q);
  }
  for (double v : state.velocities) {
    msg.add_velocity(v);
  }
  return msg;
}

core::JointState FromMsg(const automsgs::msgs::sensor_msgs::JointState& msg) {
  core::JointState state;
  state.names.assign(msg.name().begin(), msg.name().end());
  state.positions.assign(msg.position().begin(), msg.position().end());
  state.velocities.assign(msg.velocity().begin(), msg.velocity().end());
  return state;
}

automsgs::msgs::trajectory_msgs::JointTrajectory ToJointTrajectoryMsg(
    const core::RobotTrajectory& trajectory) {
  automsgs::msgs::trajectory_msgs::JointTrajectory msg;
  if (!trajectory.waypoints.empty()) {
    for (const auto& name : trajectory.waypoints.front().names) {
      msg.add_joint_names(name);
    }
  }
  for (std::size_t i = 0; i < trajectory.waypoints.size(); ++i) {
    auto* point = msg.add_points();
    for (double q : trajectory.waypoints[i].positions) {
      point->add_positions(q);
    }
    for (double v : trajectory.waypoints[i].velocities) {
      point->add_velocities(v);
    }
    double t = 0.0;
    if (i < trajectory.time_from_start.size()) {
      t = trajectory.time_from_start[i];
    }
    SetDuration(t, point->mutable_time_from_start());
  }
  return msg;
}

core::RobotTrajectory FromJointTrajectoryMsg(
    const automsgs::msgs::trajectory_msgs::JointTrajectory& msg) {
  core::RobotTrajectory trajectory;
  trajectory.waypoints.reserve(static_cast<std::size_t>(msg.points_size()));
  trajectory.time_from_start.reserve(
      static_cast<std::size_t>(msg.points_size()));
  for (const auto& point : msg.points()) {
    core::JointState waypoint;
    waypoint.names.assign(msg.joint_names().begin(), msg.joint_names().end());
    waypoint.positions.assign(point.positions().begin(),
                              point.positions().end());
    waypoint.velocities.assign(point.velocities().begin(),
                               point.velocities().end());
    trajectory.waypoints.push_back(std::move(waypoint));
    trajectory.time_from_start.push_back(
        DurationSeconds(point.time_from_start()));
  }
  return trajectory;
}

automsgs::msgs::moveit_msgs::RobotTrajectory ToMsg(
    const core::RobotTrajectory& trajectory) {
  automsgs::msgs::moveit_msgs::RobotTrajectory msg;
  *msg.mutable_joint_trajectory() = ToJointTrajectoryMsg(trajectory);
  return msg;
}

core::RobotTrajectory FromMsg(
    const automsgs::msgs::moveit_msgs::RobotTrajectory& msg) {
  return FromJointTrajectoryMsg(msg.joint_trajectory());
}

automsgs::msgs::moveit_msgs::CollisionObject ToMsg(
    const CollisionObject& object) {
  automsgs::msgs::moveit_msgs::CollisionObject msg;
  msg.set_id(object.id);
  msg.set_operation(automsgs::msgs::moveit_msgs::CollisionObject::ADD);
  auto* prim = msg.add_primitives();
  auto* pose = msg.add_primitive_poses();
  pose->mutable_position()->set_x(object.x);
  pose->mutable_position()->set_y(object.y);
  pose->mutable_position()->set_z(object.z);
  pose->mutable_orientation()->set_w(1.0);

  if (object.type == ShapeType::kSphere) {
    prim->set_type(automsgs::msgs::shape_msgs::SolidPrimitive::SPHERE);
    prim->add_dimensions(object.size_x);
  } else if (object.type == ShapeType::kCylinder) {
    prim->set_type(automsgs::msgs::shape_msgs::SolidPrimitive::CYLINDER);
    prim->add_dimensions(object.size_z);  // height
    prim->add_dimensions(object.size_x);  // radius
  } else if (object.type == ShapeType::kMesh) {
    if (!object.mesh_vertices.empty()) {
      auto* mesh = msg.add_meshes();
      for (const auto& v : object.mesh_vertices) {
        auto* p = mesh->add_vertices();
        p->set_x(v.x);
        p->set_y(v.y);
        p->set_z(v.z);
      }
      auto* mesh_pose = msg.add_mesh_poses();
      mesh_pose->mutable_orientation()->set_w(1.0);
      // Still publish AABB primitive for simple consumers.
    }
    prim->set_type(automsgs::msgs::shape_msgs::SolidPrimitive::BOX);
    prim->add_dimensions(object.size_x);
    prim->add_dimensions(object.size_y);
    prim->add_dimensions(object.size_z);
  } else {
    prim->set_type(automsgs::msgs::shape_msgs::SolidPrimitive::BOX);
    prim->add_dimensions(object.size_x);
    prim->add_dimensions(object.size_y);
    prim->add_dimensions(object.size_z);
  }
  return msg;
}

CollisionObject FromMsg(
    const automsgs::msgs::moveit_msgs::CollisionObject& msg) {
  CollisionObject object;
  object.id = msg.id();
  if (msg.primitive_poses_size() > 0) {
    const auto& pose = msg.primitive_poses(0);
    object.x = pose.position().x();
    object.y = pose.position().y();
    object.z = pose.position().z();
  } else if (msg.has_pose()) {
    object.x = msg.pose().position().x();
    object.y = msg.pose().position().y();
    object.z = msg.pose().position().z();
  }

  if (msg.meshes_size() > 0) {
    object.type = ShapeType::kMesh;
    const auto& mesh = msg.meshes(0);
    object.mesh_vertices.reserve(static_cast<std::size_t>(mesh.vertices_size()));
    for (const auto& v : mesh.vertices()) {
      object.mesh_vertices.push_back({v.x(), v.y(), v.z()});
    }
    if (msg.mesh_poses_size() > 0) {
      const auto& pose = msg.mesh_poses(0);
      // Vertices are treated in object frame; shift by mesh pose translation.
      for (auto& mv : object.mesh_vertices) {
        mv.x += pose.position().x();
        mv.y += pose.position().y();
        mv.z += pose.position().z();
      }
    }
    UpdateMeshAabb(&object);
    return object;
  }

  if (msg.primitives_size() == 0) {
    return object;
  }
  const auto& prim = msg.primitives(0);

  if (prim.type() == automsgs::msgs::shape_msgs::SolidPrimitive::SPHERE) {
    object.type = ShapeType::kSphere;
    object.size_x = prim.dimensions_size() > 0 ? prim.dimensions(0) : 0.0;
  } else if (prim.type() ==
             automsgs::msgs::shape_msgs::SolidPrimitive::CYLINDER) {
    object.type = ShapeType::kCylinder;
    object.size_z = prim.dimensions_size() > 0 ? prim.dimensions(0) : 0.0;
    object.size_x = prim.dimensions_size() > 1 ? prim.dimensions(1) : 0.0;
  } else if (prim.type() == automsgs::msgs::shape_msgs::SolidPrimitive::BOX) {
    object.type = ShapeType::kBox;
    object.size_x = prim.dimensions_size() > 0 ? prim.dimensions(0) : 0.0;
    object.size_y = prim.dimensions_size() > 1 ? prim.dimensions(1) : 0.0;
    object.size_z = prim.dimensions_size() > 2 ? prim.dimensions(2) : 0.0;
  } else {
    object.type = ShapeType::kUnknown;
  }
  return object;
}

}  // namespace scene
}  // namespace manipulation
}  // namespace autonomy
