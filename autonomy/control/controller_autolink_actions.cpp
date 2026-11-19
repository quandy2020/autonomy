/*
 * Copyright 2026 The Openbot Authors
 *
 * Minimal autolink action servers for ControllerServer (task BT RPC).
 */

#include "autonomy/control/controller_server.hpp"

#include <algorithm>
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
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/std_msgs/float32_multi_array.pb.h>
#include <automsgs/msgs/std_msgs/string.pb.h>
#include <automsgs/msgs/diagnostic_msgs/diagnostic_array.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include "autonomy/common/logging.hpp"
#include "autonomy/common/nav_log.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/control/constants.hpp"
#include "autonomy/control/controller/mppi_controller/controller.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
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

double PoseYaw(const automsgs::msgs::geometry_msgs::PoseStamped& pose) {
    const auto& q = pose.pose().orientation();
    const double n = q.x() * q.x() + q.y() * q.y() + q.z() * q.z() + q.w() * q.w();
    if (n < 1e-12) {
        return 0.0;
    }
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      n - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

void PublishBodyCmd(ControllerServer* self, double vx, double wz) {
    automsgs::msgs::geometry_msgs::TwistStamped cmd;
    *cmd.mutable_header()->mutable_stamp() =
        automsgs::msgs::builtin_interfaces::TimeNow();
    cmd.mutable_header()->set_frame_id(self->GetRobotBaseFrame());
    cmd.mutable_twist()->mutable_linear()->set_x(vx);
    cmd.mutable_twist()->mutable_angular()->set_z(wz);
    self->PublishVelocity(cmd);
}

bool NearestFreeWorld(map::costmap_2d::Costmap2D* grid, double x, double y,
                      double* ox, double* oy) {
    if (grid == nullptr || ox == nullptr || oy == nullptr) {
        return false;
    }
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!grid->worldToMap(x, y, mx, my)) {
        return false;
    }
    const int size_x = static_cast<int>(grid->getSizeInCellsX());
    const int size_y = static_cast<int>(grid->getSizeInCellsY());
    auto is_free = [&](int cx, int cy) {
        if (cx < 0 || cy < 0 || cx >= size_x || cy >= size_y) {
            return false;
        }
        return grid->getCost(static_cast<unsigned int>(cx),
                             static_cast<unsigned int>(cy)) <
               map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    };
    const double res = std::max(grid->getResolution(), 0.01);
    const int max_r = std::max(1, static_cast<int>(std::ceil(1.5 / res)));
    for (int r = 0; r <= max_r; ++r) {
        for (int dx = -r; dx <= r; ++dx) {
            for (int dy : {-r, r}) {
                if (is_free(static_cast<int>(mx) + dx, static_cast<int>(my) + dy)) {
                    grid->mapToWorld(
                        static_cast<unsigned int>(static_cast<int>(mx) + dx),
                        static_cast<unsigned int>(static_cast<int>(my) + dy),
                        *ox, *oy);
                    return true;
                }
            }
        }
        for (int dy = -r + 1; dy <= r - 1; ++dy) {
            for (int dx : {-r, r}) {
                if (is_free(static_cast<int>(mx) + dx, static_cast<int>(my) + dy)) {
                    grid->mapToWorld(
                        static_cast<unsigned int>(static_cast<int>(mx) + dx),
                        static_cast<unsigned int>(static_cast<int>(my) + dy),
                        *ox, *oy);
                    return true;
                }
            }
        }
    }
    return false;
}

// Drive toward the nearest free cell instead of always reversing in base_link.
// Blind -vx walks into the wall when the robot is already in inflation.
void PublishEscapeCmd(ControllerServer* self, double speed,
                      const automsgs::msgs::geometry_msgs::PoseStamped& pose,
                      map::costmap_2d::Costmap2D* grid) {
    double vx = -std::abs(speed);
    double wz = 0.0;
    double fx = 0.0;
    double fy = 0.0;
    const double x = pose.pose().position().x();
    const double y = pose.pose().position().y();
    if (grid != nullptr && NearestFreeWorld(grid, x, y, &fx, &fy)) {
        const double dx = fx - x;
        const double dy = fy - y;
        if (std::hypot(dx, dy) > 0.02) {
            const double yaw = PoseYaw(pose);
            const double c = std::cos(yaw);
            const double s = std::sin(yaw);
            const double body_x = c * dx + s * dy;
            const double body_y = -s * dx + c * dy;
            vx = (body_x >= 0.0) ? std::abs(speed) : -std::abs(speed);
            const double heading = std::atan2(body_y, std::fabs(body_x) + 1e-3);
            wz = std::clamp(heading * 2.0, -0.8, 0.8);
            if (body_x < 0.0) {
                wz = -wz;
            }
        }
    }
    PublishBodyCmd(self, vx, wz);
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

    if (!native_io_ && !cmd_vel_writer_) {
        cmd_vel_writer_ =
            node_->CreateWriter<automsgs::msgs::geometry_msgs::TwistStamped>(
                options_.publish_navigation_candidate()
                    ? kNavigationCandidateChannel
                    : kCmdVelChannel);
        if (!cmd_vel_writer_ && options_.publish_navigation_candidate()) {
            AERROR << "ControllerServer: failed to open navigation candidate channel";
            return false;
        }
    }

    if (!native_io_ && !plan_clear_writer_) {
        // Same channel as PlannerServer::PublishPlan — empty Path is ignored by
        // NavigationClient and clears Autoviz PathDisplay latch on goal exit.
        plan_clear_writer_ =
            node_->CreateWriter<automsgs::msgs::nav_msgs::Path>("/plan");
    }

    if (!native_io_ && !odom_reader_) {
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

    if (!native_io_ && !map_reader_ && costmap_wrapper_) {
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

    if (!follow_grid_buffer_) {
        follow_grid_buffer_ = std::make_shared<
            controller::mppi_controller::tools::GridMapBuffer>();
        for (auto& entry : controllers_) {
            auto* mppi = dynamic_cast<
                controller::mppi_controller::MPPIController*>(
                entry.second.get());
            if (mppi != nullptr) {
                mppi->SetGridMapBuffer(follow_grid_buffer_);
            }
        }
    }

    if (!native_io_ && !follow_grid_reader_ && follow_grid_buffer_) {
        ControllerServer* self = this;
        follow_grid_reader_ =
            node_->CreateReader<automsgs::msgs::map_msgs::GridMap>(
                kFollowGridTopicName,
                [self](const std::shared_ptr<automsgs::msgs::map_msgs::GridMap>&
                           msg) {
                    if (!msg || !self->follow_grid_buffer_) {
                        return;
                    }
                    if (!self->follow_grid_buffer_->Update(*msg)) {
                        AWARN << "ControllerServer: follow GridMap update "
                                 "failed";
                    }
                });
        if (follow_grid_reader_) {
            AINFO << "ControllerServer: follow GridMap (2.5D) on "
                  << kFollowGridTopicName;
        } else {
            AWARN << "ControllerServer: failed to subscribe "
                  << kFollowGridTopicName;
        }
    }

    if (!native_io_ && !scan_reader_ && costmap_wrapper_) {
        costmap_wrapper_->AttachSensorReaders(node_);
    }

    const auto& policy_opts = options_.policy_controller_options();
    const std::string scan_topic = policy_opts.scan_topic().empty()
                                       ? kScanTopicName
                                       : policy_opts.scan_topic();
    const std::string image_topic = policy_opts.image_topic().empty()
                                        ? kImageTopicName
                                        : policy_opts.image_topic();
    const std::string points_topic = policy_opts.point_cloud_topic().empty()
                                         ? kPointCloudTopicName
                                         : policy_opts.point_cloud_topic();
    const std::string bev_topic = policy_opts.lidar_bev_topic().empty()
                                      ? kLidarBevTopicName
                                      : policy_opts.lidar_bev_topic();
    const std::string raw_topic = policy_opts.raw_tensor_topic().empty()
                                      ? kRawTensorTopicName
                                      : policy_opts.raw_tensor_topic();
    const std::string diag_topic = policy_opts.diagnostics_topic().empty()
                                       ? kPolicyDiagnosticsTopicName
                                       : policy_opts.diagnostics_topic();
    const std::string reload_topic = policy_opts.reload_config_topic().empty()
                                         ? kPolicyReloadConfigTopicName
                                         : policy_opts.reload_config_topic();

    if (!scan_reader_ && (!native_io_ || !policy_opts.scan_topic().empty())) {
        ControllerServer* self = this;
        scan_reader_ =
            node_->CreateReader<automsgs::msgs::sensor_msgs::LaserScan>(
                scan_topic,
                [self](const std::shared_ptr<
                       automsgs::msgs::sensor_msgs::LaserScan>& msg) {
                    if (self != nullptr && msg) {
                        self->PublishLaserScanToControllers(*msg);
                    }
                });
        if (scan_reader_) {
            AINFO << "ControllerServer: LaserScan on " << scan_topic
                  << " (policy_controller / neupan_controller)";
        }
    }

    if (!image_reader_ && (!native_io_ || !policy_opts.image_topic().empty())) {
        ControllerServer* self = this;
        image_reader_ =
            node_->CreateReader<automsgs::msgs::sensor_msgs::Image>(
                image_topic,
                [self](const std::shared_ptr<
                       automsgs::msgs::sensor_msgs::Image>& msg) {
                    if (self != nullptr && msg) {
                        self->PublishImageToControllers(*msg);
                    }
                });
        if (image_reader_) {
            AINFO << "ControllerServer: Image on " << image_topic
                  << " (policy_controller)";
        }
    }

    if (!point_cloud_reader_ && (!native_io_ || !policy_opts.point_cloud_topic().empty())) {
        ControllerServer* self = this;
        point_cloud_reader_ =
            node_->CreateReader<automsgs::msgs::sensor_msgs::PointCloud2>(
                points_topic,
                [self](const std::shared_ptr<
                       automsgs::msgs::sensor_msgs::PointCloud2>& msg) {
                    if (self != nullptr && msg) {
                        self->PublishPointCloudToControllers(*msg);
                    }
                });
        if (point_cloud_reader_) {
            AINFO << "ControllerServer: PointCloud2 on " << points_topic
                  << " (policy_controller)";
        }
    }

    if (!lidar_bev_reader_ && (!native_io_ || !policy_opts.lidar_bev_topic().empty())) {
        ControllerServer* self = this;
        lidar_bev_reader_ =
            node_->CreateReader<automsgs::msgs::std_msgs::Float32MultiArray>(
                bev_topic,
                [self](const std::shared_ptr<
                       automsgs::msgs::std_msgs::Float32MultiArray>& msg) {
                    if (self != nullptr && msg) {
                        self->PublishLidarBevToControllers(*msg);
                    }
                });
        if (lidar_bev_reader_) {
            AINFO << "ControllerServer: LidarBEV Float32MultiArray on "
                  << bev_topic;
        }
    }

    if (!raw_tensor_reader_ && (!native_io_ || !policy_opts.raw_tensor_topic().empty())) {
        ControllerServer* self = this;
        raw_tensor_reader_ =
            node_->CreateReader<automsgs::msgs::std_msgs::Float32MultiArray>(
                raw_topic,
                [self](const std::shared_ptr<
                       automsgs::msgs::std_msgs::Float32MultiArray>& msg) {
                    if (self != nullptr && msg) {
                        self->PublishRawTensorToControllers(*msg);
                    }
                });
        if (raw_tensor_reader_) {
            AINFO << "ControllerServer: RAW Float32MultiArray on " << raw_topic;
        }
    }

    if (!policy_reload_reader_ && (!native_io_ || !policy_opts.reload_config_topic().empty())) {
        ControllerServer* self = this;
        policy_reload_reader_ =
            node_->CreateReader<automsgs::msgs::std_msgs::String>(
                reload_topic,
                [self](const std::shared_ptr<automsgs::msgs::std_msgs::String>&
                           msg) {
                    if (self != nullptr && msg && !msg->data().empty()) {
                        self->ReloadPolicyConfig(msg->data());
                    }
                });
        if (policy_reload_reader_) {
            AINFO << "ControllerServer: policy reload on " << reload_topic;
        }
    }

    if (!policy_diagnostics_writer_ && (!native_io_ || !policy_opts.diagnostics_topic().empty())) {
        policy_diagnostics_writer_ =
            node_->CreateWriter<
                automsgs::msgs::diagnostic_msgs::DiagnosticArray>(diag_topic);
        if (policy_diagnostics_writer_) {
            AINFO << "ControllerServer: policy diagnostics on " << diag_topic;
        }
    }

    if (!native_io_ && !costmap_writer_ && costmap_wrapper_) {
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
                self->PublishZeroVelocity();
                server->TerminateCurrent(result);
                return;
            }

            self->current_path_ = path;
            self->follow_controller_id_ = goal->controller_id();
            self->follow_goal_checker_id_ = goal->goal_checker_id();
            self->follow_progress_checker_id_ = goal->progress_checker_id();

            try {
                const bool reuse = self->follow_path_active_;
                self->ComputeControl(/*reset_plugins=*/!reuse);
            } catch (const std::exception& ex) {
                self->PublishZeroVelocity();
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
                    self->PublishZeroVelocity();
                    self->OnGoalExit();
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
                    self->PublishZeroVelocity();
                    self->OnGoalExit();
                    server->TerminateCurrent(result);
                    return;
                } catch (const common::PatienceExceeded& ex) {
                    AWARN << "FollowPath patience exceeded: " << ex.what();
                    auto result =
                        std::make_shared<nav_proto::FollowPathAction::Result>();
                    result->set_error_code(err_proto::CONTROL_PATIENCE_EXCEEDED);
                    result->set_error_msg(ex.what());
                    self->PublishZeroVelocity();
                    self->OnGoalExit();
                    server->TerminateCurrent(result);
                    return;
                } catch (const common::ControllerException& ex) {
                    AWARN << "FollowPath controller error: " << ex.what();
                    auto result =
                        std::make_shared<nav_proto::FollowPathAction::Result>();
                    result->set_error_code(err_proto::CONTROL_UNKNOWN);
                    result->set_error_msg(ex.what());
                    self->PublishZeroVelocity();
                    self->OnGoalExit();
                    server->TerminateCurrent(result);
                    return;
                } catch (const std::exception& ex) {
                    AERROR << "FollowPath unexpected error: " << ex.what();
                    auto result =
                        std::make_shared<nav_proto::FollowPathAction::Result>();
                    result->set_error_code(err_proto::CONTROL_UNKNOWN);
                    result->set_error_msg(ex.what());
                    self->PublishZeroVelocity();
                    self->OnGoalExit();
                    server->TerminateCurrent(result);
                    return;
                }

                std::this_thread::sleep_for(period);
            }

            // Cancel: terminate. Preempt: just exit — SimpleActionServer::Work
            // will AcceptPendingGoal and re-enter execute for the new path.
            // Do not clear /plan on preempt: the next execute already has a
            // path; emptying the overlay is the one-frame Autoviz flash.
            const bool preempt = server->IsPreemptRequested();
            if (server->IsCancelRequested()) {
                self->OnGoalExit(/*clear_visual_plan=*/true);
                self->PublishZeroVelocity();
                auto result =
                    std::make_shared<nav_proto::FollowPathAction::Result>();
                result->set_error_code(err_proto::CONTROL_UNKNOWN);
                result->set_error_msg("FollowPath canceled");
                server->TerminateCurrent(result);
            } else if (preempt) {
                // Keep cmd_vel and MPPI state; the next execute only SetPlan.
                AINFO << "nav follow path_update poses="
                      << self->current_path_.poses_size();
            } else {
                self->OnGoalExit(/*clear_visual_plan=*/true);
                self->PublishZeroVelocity();
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

            const double target = goal->target_yaw() != 0.f
                                      ? static_cast<double>(goal->target_yaw())
                                      : 0.6;
            const double allowance_sec =
                goal->has_time_allowance()
                    ? automsgs::msgs::builtin_interfaces::DurationToSeconds(
                          goal->time_allowance())
                    : 10.0;
            const auto deadline = std::chrono::steady_clock::now() +
                                  std::chrono::duration<double>(allowance_sec);
            automsgs::msgs::geometry_msgs::PoseStamped start_pose;
            const bool have_start = self->GetRobotPose(start_pose);
            const double start_yaw = have_start ? PoseYaw(start_pose) : 0.0;
            const double sign = target >= 0.0 ? 1.0 : -1.0;
            double previous_yaw = start_yaw;
            double cumulative_turn = 0.0;
            const auto started = std::chrono::steady_clock::now();
            AINFO << "nav recover spin yaw=" << target;

            while (autolink::OK()) {
                if (server->IsCancelRequested()) {
                    self->PublishZeroVelocity();
                    auto result = std::make_shared<nav_proto::SpinAction::Result>();
                    result->set_error_code(err_proto::SPIN_ERROR);
                    server->TerminateCurrent(result);
                    return;
                }
                automsgs::msgs::geometry_msgs::PoseStamped pose;
                double turned = cumulative_turn;
                if (self->GetRobotPose(pose) && have_start) {
                    const double current_yaw = PoseYaw(pose);
                    const double delta = std::remainder(
                        current_yaw - previous_yaw, 2.0 * std::acos(-1.0));
                    // Accumulate signed progress before angle wrap. Re-measuring
                    // the absolute remainder from start made a sweep crossing
                    // +/-pi stop increasing before a half-turn target.
                    if (delta * sign > 0.0) cumulative_turn += std::abs(delta);
                    previous_yaw = current_yaw;
                    turned = cumulative_turn;
                } else {
                    const double elapsed = std::chrono::duration<double>(
                                               std::chrono::steady_clock::now() -
                                               started)
                                               .count();
                    turned = elapsed * 0.6;
                }
                if (turned >= std::abs(target) ||
                    std::chrono::steady_clock::now() >= deadline) {
                    self->PublishZeroVelocity();
                    auto result = std::make_shared<nav_proto::SpinAction::Result>();
                    result->set_error_code(err_proto::OK);
                    server->SucceededCurrent(result);
                    return;
                }
                const double remaining = std::abs(target) - turned;
                const double wz = sign * std::clamp(remaining * 2.0, 0.25, 0.8);
                PublishBodyCmd(self, 0.0, wz);
                std::this_thread::sleep_for(
                    ControlPeriod(self->controller_frequency_));
            }
            self->PublishZeroVelocity();
        });

    autolink_actions_->backup = std::make_shared<BackUpServer>(
        node, kBackUpActionName, [self]() {
            auto& server = self->autolink_actions_->backup;
            auto goal = server->GetCurrentGoal();
            if (!goal) {
                return;
            }

            const double speed = goal->speed() > 0.f ? goal->speed() : 0.15;
            const double dist =
                goal->has_target()
                    ? std::hypot(goal->target().x(), goal->target().y())
                    : 0.35;
            const double allowance_sec =
                goal->has_time_allowance()
                    ? automsgs::msgs::builtin_interfaces::DurationToSeconds(
                          goal->time_allowance())
                    : std::max(4.0, dist / std::max(speed, 0.05) + 1.0);
            const auto deadline = std::chrono::steady_clock::now() +
                                  std::chrono::duration<double>(allowance_sec);
            automsgs::msgs::geometry_msgs::PoseStamped start_pose;
            const bool have_start = self->GetRobotPose(start_pose);
            const auto started = std::chrono::steady_clock::now();
            double free_x = 0.0;
            double free_y = 0.0;
            bool have_free = false;
            if (have_start && self->costmap_wrapper_ &&
                self->costmap_wrapper_->getCostmap()) {
                have_free = NearestFreeWorld(
                    self->costmap_wrapper_->getCostmap(),
                    start_pose.pose().position().x(),
                    start_pose.pose().position().y(), &free_x, &free_y);
            }
            if (have_free) {
                AINFO << "nav recover backup dist=" << dist << " speed=" << speed
                      << " toward="
                      << ::autonomy::common::NavXY{free_x, free_y};
            } else {
                AINFO << "nav recover backup dist=" << dist << " speed=" << speed
                      << " toward=reverse";
            }

            while (autolink::OK()) {
                if (server->IsCancelRequested()) {
                    self->PublishZeroVelocity();
                    auto result =
                        std::make_shared<nav_proto::BackUpAction::Result>();
                    result->set_error_code(err_proto::BACK_UP_ERROR);
                    server->TerminateCurrent(result);
                    return;
                }
                double traveled = 0.0;
                automsgs::msgs::geometry_msgs::PoseStamped pose;
                if (self->GetRobotPose(pose) && have_start) {
                    traveled = std::hypot(
                        pose.pose().position().x() -
                            start_pose.pose().position().x(),
                        pose.pose().position().y() -
                            start_pose.pose().position().y());
                } else {
                    traveled = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - started)
                                   .count() *
                               speed;
                }
                if (traveled >= dist ||
                    std::chrono::steady_clock::now() >= deadline) {
                    self->PublishZeroVelocity();
                    auto result =
                        std::make_shared<nav_proto::BackUpAction::Result>();
                    result->set_error_code(err_proto::OK);
                    server->SucceededCurrent(result);
                    return;
                }
                if (self->GetRobotPose(pose)) {
                    auto* grid =
                        (self->costmap_wrapper_ &&
                         self->costmap_wrapper_->getCostmap())
                            ? self->costmap_wrapper_->getCostmap()
                            : nullptr;
                    PublishEscapeCmd(self, speed, pose, grid);
                } else {
                    PublishBodyCmd(self, -speed, 0.0);
                }
                std::this_thread::sleep_for(
                    ControlPeriod(self->controller_frequency_));
            }
            self->PublishZeroVelocity();
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
