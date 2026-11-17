/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from nav2_rviz_plugins (GoalUpdater definition in nav2_panel.cpp).
 *****************************************************************************/

#include "autoviz/plugins/tools/goal_pose/goal_common.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <QObject>
#include <QString>
#include <QTimer>

#include "autolink/node/node.hpp"
#include "autolink/node/writer.hpp"
#include "autoviz/plugins/tools/common/publish_tool_utils.hpp"

namespace autoviz {
namespace tools {
namespace {

using GoalWriter =
    ::autolink::Writer<automsgs::msgs::geometry_msgs::PoseStamped>;

std::mutex g_writer_mutex;
std::shared_ptr<::autolink::Node> g_writer_node;
std::string g_writer_channel;
std::shared_ptr<GoalWriter> g_writer;

std::shared_ptr<GoalWriter> CachedGoalWriter(
    const std::shared_ptr<::autolink::Node>& node, const std::string& channel) {
  if (node == nullptr || channel.empty()) {
    return nullptr;
  }
  std::lock_guard<std::mutex> lock(g_writer_mutex);
  if (g_writer != nullptr && g_writer_node == node &&
      g_writer_channel == channel) {
    return g_writer;
  }
  g_writer = node->CreateWriter<automsgs::msgs::geometry_msgs::PoseStamped>(
      channel);
  g_writer_node = node;
  g_writer_channel = g_writer != nullptr ? channel : std::string{};
  return g_writer;
}

bool WriteCached(
    const std::shared_ptr<::autolink::Node>& node, const std::string& channel,
    const automsgs::msgs::geometry_msgs::PoseStamped& goal) {
  auto writer = CachedGoalWriter(node, channel);
  if (writer == nullptr) {
    return false;
  }
  return writer->Write(
      std::make_shared<automsgs::msgs::geometry_msgs::PoseStamped>(goal));
}

}  // namespace

GoalPoseUpdater GoalUpdater;

bool PublishNavigationGoal(
    const std::shared_ptr<::autolink::Node>& node, const std::string& channel,
    const QVector3D& position, float yaw, const std::string& frame) {
  const QString frame_q = QString::fromStdString(frame);
  GoalUpdater.setGoal(static_cast<double>(position.x()),
                      static_cast<double>(position.y()),
                      static_cast<double>(yaw), frame_q);

  if (GoalUpdater.accumulateOnly()) {
    return true;
  }

  if (node == nullptr || channel.empty()) {
    return false;
  }

  automsgs::msgs::geometry_msgs::PoseStamped goal;
  FillHeader(goal.mutable_header(), frame);
  goal.mutable_pose()->mutable_position()->set_x(position.x());
  goal.mutable_pose()->mutable_position()->set_y(position.y());
  goal.mutable_pose()->mutable_position()->set_z(0.0);
  SetYawQuaternion(goal.mutable_pose()->mutable_orientation(), yaw);

  // Keep the writer alive across clicks. Creating a writer per publish tears
  // down SHM before TaskServer's receiver is enabled, so /goal_pose never
  // reaches NavigationTask (rings still appear via GoalUpdater).
  const bool ok = WriteCached(node, channel, goal);

  QTimer::singleShot(50, &GoalUpdater, [node, channel, goal]() {
    WriteCached(node, channel, goal);
  });
  QTimer::singleShot(200, &GoalUpdater, [node, channel, goal]() {
    WriteCached(node, channel, goal);
  });
  return ok;
}

}  // namespace tools
}  // namespace autoviz
