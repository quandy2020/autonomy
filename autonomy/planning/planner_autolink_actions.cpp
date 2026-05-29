/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/planning/planner_server.hpp"

#include <memory>

#include "autolink/action/simple_action_server.hpp"
#include "autolink/autolink.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/navigators/action_type.hpp"

namespace autonomy {
namespace planning {
namespace {

namespace task_proto = tasks::behavior_tree::task_proto;
using ComputePathToPoseServer = autolink::action::SimpleActionServer<
    tasks::behavior_tree::ComputePathToPoseActionTraits>;
using ComputePathThroughPosesServer = autolink::action::SimpleActionServer<
    tasks::behavior_tree::ComputePathThroughPosesActionTraits>;

}  // namespace

struct PlannerServer::AutolinkActionServers {
    std::shared_ptr<ComputePathToPoseServer> compute_path_to_pose;
    std::shared_ptr<ComputePathThroughPosesServer> compute_path_through_poses;
};

bool PlannerServer::AttachAutolinkNode(std::shared_ptr<autolink::Node> node) {
    if (!node || autolink_actions_) {
        return node != nullptr;
    }
    autolink_actions_ = new AutolinkActionServers();
    PlannerServer* self = this;

    autolink_actions_->compute_path_to_pose =
        std::make_shared<ComputePathToPoseServer>(
            node, tasks::behavior_tree::kComputePathToPoseActionName,
            [self]() {
                auto server = self->autolink_actions_->compute_path_to_pose;
                auto goal = server->GetCurrentGoal();
                if (!goal) {
                    return;
                }
                commsgs::geometry_msgs::PoseStamped start;
                commsgs::geometry_msgs::PoseStamped goal_pose =
                    commsgs::geometry_msgs::FromProto(goal->goal());
                if (goal->use_start() && goal->has_start()) {
                    start = commsgs::geometry_msgs::FromProto(goal->start());
                } else if (!self->costmap_wrapper_ ||
                           !self->costmap_wrapper_->getRobotPose(start)) {
                    auto result = std::make_shared<
                        task_proto::ComputePathToPoseAction_Result>();
                    result->set_error_code(task_proto::COMPUTE_PATH_TO_POSE_TF_ERROR);
                    server->TerminateCurrent(result);
                    return;
                }
                auto cancel = [&]() { return server->IsCancelRequested(); };
                try {
                    auto path = self->ComputePathToPose(
                        start, goal_pose, goal->planner_id(), cancel);
                    auto result = std::make_shared<
                        task_proto::ComputePathToPoseAction_Result>();
                    *result->mutable_path() =
                        commsgs::planning_msgs::ToProto(path);
                    result->set_error_code(task_proto::COMPUTE_PATH_TO_POSE_NONE);
                    server->SucceededCurrent(result);
                } catch (const std::exception& ex) {
                    auto result = std::make_shared<
                        task_proto::ComputePathToPoseAction_Result>();
                    result->set_error_code(
                        task_proto::COMPUTE_PATH_TO_POSE_UNKNOWN);
                    result->set_error_msg(ex.what());
                    server->TerminateCurrent(result);
                }
            });

  autolink_actions_->compute_path_through_poses =
      std::make_shared<ComputePathThroughPosesServer>(
          node, tasks::behavior_tree::kComputePathThroughPosesActionName,
          [self]() {
              auto server =
                  self->autolink_actions_->compute_path_through_poses;
              auto goal = server->GetCurrentGoal();
              if (!goal) {
                  return;
              }
              commsgs::geometry_msgs::PoseStamped start;
              std::vector<commsgs::geometry_msgs::PoseStamped> goals;
              for (const auto& p : goal->goals().goals()) {
                  goals.push_back(commsgs::geometry_msgs::FromProto(p));
              }
              if (goal->use_start() && goal->has_start()) {
                  start = commsgs::geometry_msgs::FromProto(goal->start());
              } else if (!self->costmap_wrapper_ ||
                         !self->costmap_wrapper_->getRobotPose(start)) {
                  auto result = std::make_shared<
                      task_proto::ComputePathThroughPosesAction_Result>();
                  result->set_error_code(
                      task_proto::COMPUTE_PATH_THROUGH_POSES_TF_ERROR);
                  server->TerminateCurrent(result);
                  return;
              }
              auto cancel = [&]() { return server->IsCancelRequested(); };
              try {
                  auto path = self->ComputePathThroughPoses(
                      start, goals, goal->planner_id(), cancel);
                  auto result = std::make_shared<
                      task_proto::ComputePathThroughPosesAction_Result>();
                  *result->mutable_path() =
                      commsgs::planning_msgs::ToProto(path);
                  result->set_error_code(
                      task_proto::COMPUTE_PATH_THROUGH_POSES_NONE);
                  server->SucceededCurrent(result);
              } catch (const std::exception& ex) {
                  auto result = std::make_shared<
                      task_proto::ComputePathThroughPosesAction_Result>();
                  result->set_error_code(
                      task_proto::COMPUTE_PATH_THROUGH_POSES_UNKNOWN);
                  result->set_error_msg(ex.what());
                  server->TerminateCurrent(result);
              }
          });

    AINFO << "PlannerServer autolink actions attached.";
    return true;
}

PlannerServer::~PlannerServer() {
    DetachAutolinkNode();
    Shutdown();
}

void PlannerServer::DetachAutolinkNode() {
    if (autolink_actions_) {
        if (autolink_actions_->compute_path_to_pose) {
            autolink_actions_->compute_path_to_pose->Deactivate();
        }
        if (autolink_actions_->compute_path_through_poses) {
            autolink_actions_->compute_path_through_poses->Deactivate();
        }
        delete autolink_actions_;
        autolink_actions_ = nullptr;
    }
}

}  // namespace planning
}  // namespace autonomy
