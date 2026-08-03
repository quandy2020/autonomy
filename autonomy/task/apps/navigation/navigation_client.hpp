/*
 * Copyright 2026 The Openbot Authors
 *
 * Thin RPC facade: task process delegates planning/control/map work to other
 * processes. No in-process planner, controller, or costmap access.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autolink/node/node.hpp"
#include "autolink/service/client.hpp"
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/planning_msgs/planning_msgs.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/actions/nav_actions.pb.h>
#include "autonomy/task/apps/navigation/action_session.hpp"
#include "autonomy/task/common/task_action_client.hpp"

namespace BT {
class Blackboard;
class TreeNode;
}  // namespace BT

namespace autonomy {
namespace task {
namespace navigation {

namespace nav_proto = automsgs::actions;

constexpr char kComputePathToPoseAction[] = "compute_path_to_pose";
constexpr char kComputePathThroughPosesAction[] = "compute_path_through_poses";
constexpr char kSmoothPathAction[] = "smooth_path";
constexpr char kIsPathValidService[] = "is_path_valid";
constexpr char kFollowPathAction[] = "follow_path";
constexpr char kSpinAction[] = "spin";
constexpr char kBackUpAction[] = "backup";
constexpr char kWaitAction[] = "wait";
constexpr char kClearGlobalCostmapService[] =
    "global_costmap/clear_entirely_global_costmap";
constexpr char kNavigationClientBlackboardKey[] = "navigation_client";

/** Cross-process RPC bundle for navigation BT plugins. */
class NavigationClient
{
public:
    using Ptr = std::shared_ptr<NavigationClient>;

    static Ptr Create(std::shared_ptr<autolink::Node> node);
    static void SetShared(const Ptr& client);
    static Ptr FromBlackboard(const std::shared_ptr<BT::Blackboard>& blackboard);
    static Ptr FromNode(const BT::TreeNode& node);

    bool IsPlanningReady() const;
    bool IsControlReady() const;

    bool ComputePathToPose(const automsgs::msgs::geometry_msgs::PoseStamped& goal,
                           const std::string& planner_id,
                           automsgs::msgs::planning_msgs::Path& path,
                           int* error_code = nullptr,
                           std::string* error_msg = nullptr);

    bool ComputePathThroughPoses(
        const std::vector<automsgs::msgs::geometry_msgs::PoseStamped>& goals,
        const std::string& planner_id, automsgs::msgs::planning_msgs::Path& path,
        int* error_code = nullptr, std::string* error_msg = nullptr);

    bool SmoothPath(const automsgs::msgs::planning_msgs::Path& unsmoothed,
                    const std::string& smoother_id,
                    automsgs::msgs::planning_msgs::Path& smoothed,
                    int* error_code = nullptr,
                    std::string* error_msg = nullptr);

    bool IsPathValid(const automsgs::msgs::planning_msgs::Path& path, uint8_t max_cost,
                     bool consider_unknown_as_obstacle) const;

    bool ClearCostmap() const;

    bool IsGoalReached(double distance_tolerance) const;

    void CancelActiveMotion();

    common::TaskActionClient<nav_proto::FollowPathAction>& follow_path_client()
    {
        return *follow_path_client_;
    }
    common::TaskActionClient<nav_proto::SpinAction>& spin_client()
    {
        return *spin_client_;
    }
    common::TaskActionClient<nav_proto::BackUpAction>& backup_client()
    {
        return *backup_client_;
    }
    common::TaskActionClient<nav_proto::WaitAction>& wait_client()
    {
        return *wait_client_;
    }

    ActionSession<nav_proto::FollowPathAction>& follow_session()
    {
        return follow_session_;
    }

    explicit NavigationClient(std::shared_ptr<autolink::Node> node);

private:
    std::shared_ptr<autolink::Node> node_;

    std::shared_ptr<common::TaskActionClient<nav_proto::ComputePathToPoseAction>>
        compute_path_client_;
    std::shared_ptr<
        common::TaskActionClient<nav_proto::ComputePathThroughPosesAction>>
        compute_through_poses_client_;
    std::shared_ptr<common::TaskActionClient<nav_proto::SmoothPathAction>>
        smooth_path_client_;
    std::shared_ptr<common::TaskActionClient<nav_proto::FollowPathAction>>
        follow_path_client_;
    std::shared_ptr<common::TaskActionClient<nav_proto::SpinAction>>
        spin_client_;
    std::shared_ptr<common::TaskActionClient<nav_proto::BackUpAction>>
        backup_client_;
    std::shared_ptr<common::TaskActionClient<nav_proto::WaitAction>>
        wait_client_;

    std::shared_ptr<
        autolink::Client<nav_proto::IsPathValid_Request,
                         nav_proto::IsPathValid_Response>>
        path_valid_client_;
    std::shared_ptr<
        autolink::Client<nav_proto::ClearEntireCostmap_Request,
                         nav_proto::ClearEntireCostmap_Response>>
        clear_costmap_client_;

    ActionSession<nav_proto::FollowPathAction> follow_session_;
};

}  // namespace navigation
}  // namespace task
}  // namespace autonomy
