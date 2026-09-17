/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/model/pose_math.hpp"

#include "Eigen/Geometry"
#include "autonomy/common/math/math.hpp"

namespace autonomy {
namespace manipulation {
namespace model {
namespace {

Eigen::Quaterniond OrientationOf(
    const automsgs::msgs::geometry_msgs::Pose& pose) {
  return Eigen::Quaterniond(pose.orientation().w(), pose.orientation().x(),
                            pose.orientation().y(), pose.orientation().z());
}

void SetOrientation(automsgs::msgs::geometry_msgs::Pose* pose,
                    const Eigen::Quaterniond& q) {
  pose->mutable_orientation()->set_w(q.w());
  pose->mutable_orientation()->set_x(q.x());
  pose->mutable_orientation()->set_y(q.y());
  pose->mutable_orientation()->set_z(q.z());
}

void RotateVectorByPose(const automsgs::msgs::geometry_msgs::Pose& pose,
                        double x, double y, double z, double* out_x,
                        double* out_y, double* out_z) {
  const Eigen::Vector3d out =
      OrientationOf(pose) * Eigen::Vector3d(x, y, z);
  *out_x = out.x();
  *out_y = out.y();
  *out_z = out.z();
}

}  // namespace

automsgs::msgs::geometry_msgs::Pose IdentityPose() {
  automsgs::msgs::geometry_msgs::Pose pose;
  pose.mutable_orientation()->set_w(1.0);
  return pose;
}

automsgs::msgs::geometry_msgs::Pose PoseFromRollPitchYawXyz(double roll,
                                                            double pitch,
                                                            double yaw,
                                                            double x, double y,
                                                            double z) {
  // URDF/ROS fixed-axis RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll).
  const Eigen::Quaterniond q =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  automsgs::msgs::geometry_msgs::Pose pose;
  SetOrientation(&pose, q);
  pose.mutable_position()->set_x(x);
  pose.mutable_position()->set_y(y);
  pose.mutable_position()->set_z(z);
  return pose;
}

automsgs::msgs::geometry_msgs::Pose PoseFromAxisAngle(double axis_x,
                                                      double axis_y,
                                                      double axis_z,
                                                      double angle) {
  Eigen::Vector3d axis(axis_x, axis_y, axis_z);
  if (axis.norm() > 1e-12) {
    axis.normalize();
  } else {
    axis = Eigen::Vector3d::UnitZ();
  }
  automsgs::msgs::geometry_msgs::Pose pose = IdentityPose();
  SetOrientation(&pose, Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)));
  return pose;
}

automsgs::msgs::geometry_msgs::Pose ComposePoses(
    const automsgs::msgs::geometry_msgs::Pose& parent,
    const automsgs::msgs::geometry_msgs::Pose& child) {
  const double z[4] = {parent.orientation().w(), parent.orientation().x(),
                       parent.orientation().y(), parent.orientation().z()};
  const double w[4] = {child.orientation().w(), child.orientation().x(),
                       child.orientation().y(), child.orientation().z()};
  double zw[4];
  common::QuaternionProduct(z, w, zw);

  automsgs::msgs::geometry_msgs::Pose out;
  out.mutable_orientation()->set_w(zw[0]);
  out.mutable_orientation()->set_x(zw[1]);
  out.mutable_orientation()->set_y(zw[2]);
  out.mutable_orientation()->set_z(zw[3]);

  double ox = 0.0;
  double oy = 0.0;
  double oz = 0.0;
  RotateVectorByPose(parent, child.position().x(), child.position().y(),
                     child.position().z(), &ox, &oy, &oz);
  out.mutable_position()->set_x(ox + parent.position().x());
  out.mutable_position()->set_y(oy + parent.position().y());
  out.mutable_position()->set_z(oz + parent.position().z());
  return out;
}

}  // namespace model
}  // namespace manipulation
}  // namespace autonomy
