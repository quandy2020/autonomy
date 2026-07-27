/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/tools/nav_goal_tool.hpp"

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include "autoviz/tools/publish_tool_utils.hpp"

namespace autoviz {
namespace tools {

void NavGoalTool::onPoseSet(const QVector3D& position, float yaw) {
  if (context() == nullptr || context()->autolink_node == nullptr) {
    return;
  }
  automsgs::msgs::geometry_msgs::PoseStamped pose;
  FillHeader(pose.mutable_header(), context()->fixed_frame);
  pose.mutable_pose()->mutable_position()->set_x(position.x());
  pose.mutable_pose()->mutable_position()->set_y(position.y());
  pose.mutable_pose()->mutable_position()->set_z(position.z());
  SetYawQuaternion(pose.mutable_pose()->mutable_orientation(), yaw);
  if (!PublishMessage(context()->autolink_node, publishChannel(), pose)) {
    if (context()->set_status) {
      context()->set_status(
          QStringLiteral("发布失败：Topic %1")
              .arg(QString::fromStdString(publishChannel())));
    }
    return;
  }
  if (context()->set_status) {
    context()->set_status(
        QStringLiteral("已发布 Nav Goal → %1")
            .arg(QString::fromStdString(publishChannel())));
  }
}

}  // namespace tools
}  // namespace autoviz
