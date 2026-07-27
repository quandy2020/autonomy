/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/tools/pose_estimate_tool.hpp"

#include <QtMath>

#include <automsgs/msgs/geometry_msgs/pose_with_covariance_stamped.pb.h>
#include "autoviz/tools/publish_tool_utils.hpp"

namespace autoviz {
namespace tools {
namespace {

constexpr float kCovarianceX = 0.5f * 0.5f;
constexpr float kCovarianceY = 0.5f * 0.5f;
constexpr float kCovarianceYaw =
    static_cast<float>(M_PI / 12.0 * M_PI / 12.0);

}  // namespace

void PoseEstimateTool::onPoseSet(const QVector3D& position, float yaw) {
  if (context() == nullptr || context()->autolink_node == nullptr) {
    return;
  }
  automsgs::msgs::geometry_msgs::PoseWithCovarianceStamped pose;
  FillHeader(pose.mutable_header(), context()->fixed_frame);
  pose.mutable_pose()->mutable_pose()->mutable_pose()->mutable_position()->set_x(
      position.x());
  pose.mutable_pose()->mutable_pose()->mutable_pose()->mutable_position()->set_y(
      position.y());
  pose.mutable_pose()->mutable_pose()->mutable_pose()->mutable_position()->set_z(
      0.0);
  SetYawQuaternion(
      pose.mutable_pose()->mutable_pose()->mutable_pose()->mutable_orientation(),
      yaw);
  auto* cov = pose.mutable_pose()->mutable_covariance();
  cov->Resize(36, 0.0);
  cov->Set(0, kCovarianceX);
  cov->Set(7, kCovarianceY);
  cov->Set(35, kCovarianceYaw);
  PublishMessage(context()->autolink_node, publishChannel(), pose);
}

}  // namespace tools
}  // namespace autoviz
