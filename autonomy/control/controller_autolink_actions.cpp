/*
 * Copyright 2026 The Openbot Authors
 *
 * Minimal autolink action servers for ControllerServer (task BT RPC).
 */

#include "autonomy/control/controller_server.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "autolink/action/simple_action_server.hpp"
#include "autolink/node/reader.hpp"
#include "autolink/node/writer.hpp"

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>

#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <automsgs/actions/nav_actions.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include "autonomy/common/logging.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/control/constants.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"

namespace autonomy {
namespace control {
namespace {

namespace nav_proto = automsgs::actions;
namespace err_proto = automsgs::msgs::status_msgs;

using FollowPathServer =
    autolink::action::SimpleActionServer<nav_proto::FollowPathAction>;
using SpinServer = autolink::action::SimpleActionServer<nav_proto::SpinAction>;
using BackUpServer =
    autolink::action::SimpleActionServer<nav_proto::BackUpAction>;
using WaitServer = autolink::action::SimpleActionServer<nav_proto::WaitAction>;

automsgs::msgs::nav_msgs::Path PathFromGoal(
    const nav_proto::FollowPathAction::Goal& goal) {
    if (goal.has_path()) {
        return goal.path();
    }
    return {};
}

double DistanceToPathGoal(const automsgs::msgs::nav_msgs::Path& path,
                          const automsgs::msgs::geometry_msgs::PoseStamped& pose) {
    if (path.poses().empty()) {
        return 0.0;
    }
    const auto& goal = path.poses(path.poses_size() - 1).pose().position();
    const auto& current = pose.pose().position();
    return map::costmap_2d::utils::euclidean_distance(goal, current);
}

std::chrono::nanoseconds ControlPeriod(double controller_frequency) {
    const double hz = controller_frequency > 0.0 ? controller_frequency : 20.0;
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / hz));
}

}  // namespace

struct ControllerServer::AutolinkActionServers {
    std::shared_ptr<FollowPathServer> follow_path;
    std::shared_ptr<SpinServer> spin;
    std::shared_ptr<BackUpServer> backup;
    std::shared_ptr<WaitServer> wait;
};

bool ControllerServer::AttachAutolinkNode(std::shared_ptr<autolink::Node> node) {
    if (!node) {
        return false;
    }
    node_ = node;

    if (!cmd_vel_writer_) {
        cmd_vel_writer_ =
            node_->CreateWriter<automsgs::msgs::geometry_msgs::TwistStamped>(
                kCmdVelChannel);
    }

    if (!odom_reader_) {
        ControllerServer* self = this;
        odom_reader_ = node_->CreateReader<automsgs::msgs::nav_msgs::Odometry>(
            kOdomTopicName,
            [self](const std::shared_ptr<automsgs::msgs::nav_msgs::Odometry>&
                       msg) {
                if (msg) {
                    self->UpdateOdometry(*msg);
                }
            });
        if (odom_reader_) {
            AINFO << "ControllerServer: Odometry on " << kOdomTopicName;
        } else {
            AWARN << "ControllerServer: failed to subscribe " << kOdomTopicName;
        }
    }

    if (!map_reader_ && costmap_wrapper_) {
        ControllerServer* self = this;
        map_reader_ =
            node_->CreateReader<automsgs::msgs::map_msgs::OccupancyGrid>(
                kMapTopicName,
                [self](
                    const std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid>&
                        msg) {
                    if (!msg || !self->costmap_wrapper_) {
                        return;
                    }
                    if (!self->costmap_wrapper_->applyOccupancyGrid(*msg)) {
                        AWARN << "ControllerServer: applyOccupancyGrid(/map) "
                                 "failed";
                    }
                });
        if (map_reader_) {
            AINFO << "ControllerServer: OccupancyGrid on " << kMapTopicName;
        } else {
            AWARN << "ControllerServer: failed to subscribe " << kMapTopicName;
        }
    }

    if (!scan_reader_ && costmap_wrapper_) {
        costmap_wrapper_->AttachSensorReaders(node_);
    }

    if (!costmap_writer_ && costmap_wrapper_) {
        costmap_writer_ =
            node_->CreateWriter<automsgs::msgs::map_msgs::OccupancyGrid>(
                kLocalCostmapTopicName);
        if (costmap_writer_) {
            auto writer = costmap_writer_;
            costmap_wrapper_->SetMapPublishCallback(
                [writer](const automsgs::msgs::map_msgs::OccupancyGrid& grid) {
                    if (writer) {
                        writer->Write(grid);
                    }
                });
            AINFO << "ControllerServer: OccupancyGrid on "
                  << kLocalCostmapTopicName;
        } else {
            AWARN << "ControllerServer: failed to create writer "
                  << kLocalCostmapTopicName;
        }
    }

    if (autolink_actions_) {
        return true;
    }

    autolink_actions_ = new AutolinkActionServers();
    ControllerServer* self = this;

    autolink_actions_->follow_path = std::make_shared<FollowPathServer>(
        node, kFollowPathActionName, [self]() {
            auto& server = self->autolink_actions_->follow_path;
            auto goal = server->GetCurrentGoal();
            if (!goal) {
                return;
            }

            const auto path = PathFromGoal(*goal);
            if (path.poses().empty()) {
                auto result =
                    std::make_shared<nav_proto::FollowPathAction::Result>();
                result->set_error_code(err_proto::CONTROL_INVALID_PATH);
                server->TerminateCurrent(result);
                return;
            }

            self->current_path_ = path;
            self->follow_controller_id_ = goal->controller_id();
            self->follow_goal_checker_id_ = goal->goal_checker_id();
            self->follow_progress_checker_id_ = goal->progress_checker_id();

            try {
                self->ComputeControl();
            } catch (const common::ControllerException& ex) {
                AWARN << "FollowPath setup failed: " << ex.what();
                auto result =
                    std::make_shared<nav_proto::FollowPathAction::Result>();
                result->set_error_code(err_proto::CONTROL_INVALID_PATH);
                result->set_error_msg(ex.what());
                self->OnGoalExit();
                server->TerminateCurrent(result);
                return;
            }

            const auto period = ControlPeriod(self->controller_frequency_);
            auto should_stop = [&]() {
                return server->IsCancelRequested() ||
                       server->IsPreemptRequested();
            };

            while (autolink::OK() && !should_stop() &&
                   self->follow_path_active_) {
                automsgs::msgs::geometry_msgs::PoseStamped pose;
                if (self->GetRobotPose(pose)) {
                    const double distance =
                        DistanceToPathGoal(self->current_path_, pose);
                    auto feedback =
                        std::make_shared<nav_proto::FollowPathAction::Feedback>();
                    feedback->set_distance_to_goal(static_cast<float>(distance));
                    automsgs::msgs::nav_msgs::Odometry odom;
                    if (self->GetLatestOdometry(odom)) {
                        feedback->set_speed(static_cast<float>(std::hypot(
                            odom.twist().twist().linear().x(),
                            odom.twist().twist().linear().y())));
                    }
                    server->PublishFeedback(feedback);
                }

                if (self->IsGoalReached()) {
                    auto result =
                        std::make_shared<nav_proto::FollowPathAction::Result>();
                    result->set_error_code(err_proto::OK);
                    self->OnGoalExit();
                    self->PublishZeroVelocity();
                    server->SucceededCurrent(result);
                    return;
                }

                try {
                    self->ComputeAndPublishVelocity();
                } catch (const common::FailedToMakeProgress& ex) {
                    AWARN << "FollowPath progress failure: " << ex.what();
                    auto result =
                        std::make_shared<nav_proto::FollowPathAction::Result>();
                    result->set_error_code(err_proto::CONTROL_FAILED_TO_MAKE_PROGRESS);
                    result->set_error_msg(ex.what());
                    self->OnGoalExit();
                    self->PublishZeroVelocity();
                    server->TerminateCurrent(result);
                    return;
                } catch (const common::PatienceExceeded& ex) {
                    AWARN << "FollowPath patience exceeded: " << ex.what();
                    auto result =
                        std::make_shared<nav_proto::FollowPathAction::Result>();
                    result->set_error_code(err_proto::CONTROL_PATIENCE_EXCEEDED);
                    result->set_error_msg(ex.what());
                    self->OnGoalExit();
                    self->PublishZeroVelocity();
                    server->TerminateCurrent(result);
                    return;
                } catch (const common::ControllerException& ex) {
                    AWARN << "FollowPath controller error: " << ex.what();
                    auto result =
                        std::make_shared<nav_proto::FollowPathAction::Result>();
                    result->set_error_code(err_proto::CONTROL_UNKNOWN);
                    result->set_error_msg(ex.what());
                    self->OnGoalExit();
                    self->PublishZeroVelocity();
                    server->TerminateCurrent(result);
                    return;
                } catch (const std::exception& ex) {
                    AERROR << "FollowPath unexpected error: " << ex.what();
                    auto result =
                        std::make_shared<nav_proto::FollowPathAction::Result>();
                    result->set_error_code(err_proto::CONTROL_UNKNOWN);
                    result->set_error_msg(ex.what());
                    self->OnGoalExit();
                    self->PublishZeroVelocity();
                    server->TerminateCurrent(result);
                    return;
                }

                std::this_thread::sleep_for(period);
            }

            // Cancel: terminate. Preempt: just exit — SimpleActionServer::Work
            // will AcceptPendingGoal and re-enter execute for the new path.
            self->OnGoalExit();
            self->PublishZeroVelocity();
            if (server->IsCancelRequested()) {
                auto result =
                    std::make_shared<nav_proto::FollowPathAction::Result>();
                result->set_error_code(err_proto::CONTROL_UNKNOWN);
                result->set_error_msg("FollowPath canceled");
                server->TerminateCurrent(result);
            } else if (server->IsPreemptRequested()) {
                AINFO << "FollowPath preempted; switching to pending goal";
            } else {
                auto result =
                    std::make_shared<nav_proto::FollowPathAction::Result>();
                result->set_error_code(err_proto::CONTROL_UNKNOWN);
                server->TerminateCurrent(result);
            }
        });

    autolink_actions_->spin = std::make_shared<SpinServer>(
        node, kSpinActionName, [self]() {
            auto& server = self->autolink_actions_->spin;
            auto goal = server->GetCurrentGoal();
            if (!goal) {
                return;
            }

            const double allowance_sec =
                goal->has_time_allowance()
                    ? automsgs::msgs::builtin_interfaces::DurationToSeconds(
                          goal->time_allowance())
                    : 10.0;
            const auto deadline = std::chrono::steady_clock::now() +
                                  std::chrono::duration<double>(allowance_sec);

            while (autolink::OK()) {
                if (server->IsCancelRequested()) {
                    auto result = std::make_shared<nav_proto::SpinAction::Result>();
                    result->set_error_code(err_proto::SPIN_ERROR);
                    server->TerminateCurrent(result);
                    return;
                }
                if (std::chrono::steady_clock::now() >= deadline) {
                    auto result = std::make_shared<nav_proto::SpinAction::Result>();
                    result->set_error_code(err_proto::OK);
                    server->SucceededCurrent(result);
                    return;
                }
                std::this_thread::sleep_for(
                    ControlPeriod(self->controller_frequency_));
            }
        });

    autolink_actions_->backup = std::make_shared<BackUpServer>(
        node, kBackUpActionName, [self]() {
            auto& server = self->autolink_actions_->backup;
            auto goal = server->GetCurrentGoal();
            if (!goal) {
                return;
            }

            const double speed = goal->speed() > 0.f ? goal->speed() : 0.1f;
            const double dist =
                goal->has_target()
                    ? std::hypot(goal->target().x(), goal->target().y())
                    : 0.3;
            const double allowance_sec =
                goal->has_time_allowance()
                    ? automsgs::msgs::builtin_interfaces::DurationToSeconds(
                          goal->time_allowance())
                    : std::max(10.0, dist / speed);
            const auto deadline = std::chrono::steady_clock::now() +
                                  std::chrono::duration<double>(allowance_sec);

            while (autolink::OK()) {
                if (server->IsCancelRequested()) {
                    auto result =
                        std::make_shared<nav_proto::BackUpAction::Result>();
                    result->set_error_code(err_proto::BACK_UP_ERROR);
                    server->TerminateCurrent(result);
                    return;
                }
                if (std::chrono::steady_clock::now() >= deadline) {
                    auto result =
                        std::make_shared<nav_proto::BackUpAction::Result>();
                    result->set_error_code(err_proto::OK);
                    server->SucceededCurrent(result);
                    return;
                }
                std::this_thread::sleep_for(
                    ControlPeriod(self->controller_frequency_));
            }
        });

    autolink_actions_->wait = std::make_shared<WaitServer>(
        node, kWaitActionName, [self]() {
            auto& server = self->autolink_actions_->wait;
            auto goal = server->GetCurrentGoal();
            if (!goal || !goal->has_time()) {
                auto result = std::make_shared<nav_proto::WaitAction::Result>();
                result->set_error_code(err_proto::WAIT_ERROR);
                server->TerminateCurrent(result);
                return;
            }

            const double wait_sec =
                automsgs::msgs::builtin_interfaces::DurationToSeconds(goal->time());
            const auto deadline = std::chrono::steady_clock::now() +
                                  std::chrono::duration<double>(wait_sec);

            while (autolink::OK()) {
                if (server->IsCancelRequested()) {
                    auto result = std::make_shared<nav_proto::WaitAction::Result>();
                    result->set_error_code(err_proto::WAIT_ERROR);
                    server->TerminateCurrent(result);
                    return;
                }
                if (std::chrono::steady_clock::now() >= deadline) {
                    auto result = std::make_shared<nav_proto::WaitAction::Result>();
                    result->set_error_code(err_proto::OK);
                    server->SucceededCurrent(result);
                    return;
                }
                std::this_thread::sleep_for(
                    ControlPeriod(self->controller_frequency_));
            }
        });

    AINFO << "ControllerServer autolink actions attached.";
    return true;
}

void ControllerServer::DetachAutolinkNode() {
    if (!autolink_actions_) {
        return;
    }

    if (autolink_actions_->follow_path) {
        autolink_actions_->follow_path->Deactivate();
    }
    if (autolink_actions_->spin) {
        autolink_actions_->spin->Deactivate();
    }
    if (autolink_actions_->backup) {
        autolink_actions_->backup->Deactivate();
    }
    if (autolink_actions_->wait) {
        autolink_actions_->wait->Deactivate();
    }

    delete autolink_actions_;
    autolink_actions_ = nullptr;
}

}  // namespace control
}  // namespace autonomy
