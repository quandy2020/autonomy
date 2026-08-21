/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Publish path mirrors rviz_default_plugins::tools::GoalTool::onPoseSet.
 * GoalUpdater path mirrors nav2_rviz_plugins::GoalTool::onPoseSet.
 *****************************************************************************/

#include "autoviz/tools/nav_goal_tool.hpp"

#include "autoviz/tools/goal_common.hpp"
#include "autoviz/tools/publish_tool_utils.hpp"

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

  // Nav2 GoalTool: notify panels (navigate_to_pose / waypoint accumulate).
  GoalUpdater.setGoal(static_cast<double>(position.x()),
                      static_cast<double>(position.y()),
                      static_cast<double>(yaw), frame_q);

  // RViz GoalTool: publish PoseStamped for autonomy::task TaskServer (/goal_pose).
  const std::string channel = publishChannel();
  if (!ensureWriter(channel)) {
    if (context()->set_status) {
      context()->set_status(
          QStringLiteral("Failed to publish goal on %1")
              .arg(QString::fromStdString(channel)));
    }
    return;
  }

  automsgs::msgs::geometry_msgs::PoseStamped goal;
  FillHeader(goal.mutable_header(), frame);
  goal.mutable_pose()->mutable_position()->set_x(position.x());
  goal.mutable_pose()->mutable_position()->set_y(position.y());
  // Ground-plane goal; RViz GoalTool forces z = 0.
  goal.mutable_pose()->mutable_position()->set_z(0.0);
  SetYawQuaternion(goal.mutable_pose()->mutable_orientation(), yaw);

  if (!writer_->Write(goal)) {
    if (context()->set_status) {
      context()->set_status(
          QStringLiteral("Failed to publish goal on %1")
              .arg(QString::fromStdString(channel)));
    }
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
