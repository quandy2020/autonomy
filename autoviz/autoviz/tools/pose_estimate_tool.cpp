/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/tools/pose_estimate_tool.hpp"

#include <QtMath>

#include <automsgs/msgs/geometry_msgs/pose_with_covariance_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/tools/publish_tool_utils.hpp"

namespace autoviz {
namespace tools {
namespace {

constexpr float kDefaultCovarianceX = 0.5f * 0.5f;
constexpr float kDefaultCovarianceY = 0.5f * 0.5f;
constexpr float kDefaultCovarianceYaw =
    static_cast<float>(M_PI / 12.0 * M_PI / 12.0);

}  // namespace

void PoseEstimateTool::onPoseSet(const QVector3D& position, float yaw) {
  if (context() == nullptr || context()->autolink_node == nullptr) {
    return;
  }
  const float covariance_x = common::ParseFloatProperty(
      propertyValue("covariance_x", "0.25"), kDefaultCovarianceX);
  const float covariance_y = common::ParseFloatProperty(
      propertyValue("covariance_y", "0.25"), kDefaultCovarianceY);
  const float covariance_yaw = common::ParseFloatProperty(
      propertyValue("covariance_yaw", "0.0685385"), kDefaultCovarianceYaw);

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
  cov->Set(0, covariance_x);
  cov->Set(7, covariance_y);
  cov->Set(35, covariance_yaw);
  const std::string channel = publishChannel();
  if (!PublishMessage(context()->autolink_node, channel, pose)) {
    if (context()->set_status) {
      context()->set_status(
          QStringLiteral("Failed to publish initial pose to %1")
              .arg(QString::fromStdString(channel)));
    }
    return;
  }
  if (context()->set_status) {
    context()->set_status(
        QStringLiteral("Setting estimate pose on %1")
            .arg(QString::fromStdString(channel)));
  }
}

}  // namespace tools
}  // namespace autoviz
