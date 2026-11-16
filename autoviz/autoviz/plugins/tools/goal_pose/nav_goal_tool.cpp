/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Publish path mirrors rviz_default_plugins::tools::GoalTool::onPoseSet.
 * GoalUpdater path mirrors nav2_rviz_plugins::GoalTool::onPoseSet.
 *****************************************************************************/

#include "autoviz/plugins/tools/goal_pose/nav_goal_tool.hpp"

#include "autoviz/plugins/tools/goal_pose/goal_common.hpp"

namespace autoviz {
namespace tools {

bool NavGoalTool::ensureWriter(const std::string& channel) {
  if (context() == nullptr || context()->autolink_node == nullptr ||
      channel.empty()) {
    writer_.reset();
    writer_channel_.clear();
    return false;
  }
  if (writer_ != nullptr && writer_channel_ == channel) {
    return true;
  }
  writer_ = context()->autolink_node
                ->CreateWriter<automsgs::msgs::geometry_msgs::PoseStamped>(
                    channel);
  writer_channel_ = writer_ != nullptr ? channel : std::string{};
  return writer_ != nullptr;
}

void NavGoalTool::onPoseSet(const QVector3D& position, float yaw) {
  if (context() == nullptr) {
    return;
  }

  const std::string frame = context()->fixed_frame;
  const QString frame_q = QString::fromStdString(frame);
  const std::string channel = publishChannel();

  const bool ok = PublishNavigationGoal(context()->autolink_node, channel,
                                        position, yaw, frame);
  (void)ensureWriter(channel);

  if (!ok && context()->set_status) {
    context()->set_status(
        QStringLiteral("Failed to publish goal on %1")
            .arg(QString::fromStdString(channel)));
    return;
  }

  if (context()->set_status) {
    context()->set_status(QStringLiteral(
        "Setting goal pose: Frame:%1 Position(%2, %3, 0) Angle:%4 → %5")
                              .arg(frame_q)
                              .arg(position.x(), 0, 'f', 3)
                              .arg(position.y(), 0, 'f', 3)
                              .arg(yaw, 0, 'f', 3)
                              .arg(QString::fromStdString(channel)));
  }
}

}  // namespace tools
}  // namespace autoviz
