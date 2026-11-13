/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file signals.cpp
 * @brief Parse odom, path, twist, cost, and task feedback from automsgs payloads.
 */

#include "autonomy/system/recorder/signals.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include "automsgs/msgs/geometry_msgs/pose_stamped.pb.h"
#include "automsgs/msgs/geometry_msgs/twist_stamped.pb.h"
#include "automsgs/msgs/nav_msgs/odometry.pb.h"
#include "automsgs/msgs/std_msgs/float32.pb.h"
#include "automsgs/msgs/std_msgs/float32_multi_array.pb.h"
#include "automsgs/msgs/vehicle_msgs/robot_task_status.pb.h"
#include "automsgs/task/charging.pb.h"
#include "automsgs/task/exploration.pb.h"
#include "automsgs/task/localization.pb.h"
#include "automsgs/task/mapping.pb.h"
#include "automsgs/task/navigation.pb.h"
#include "automsgs/task/teleop.pb.h"
#include "automsgs/task/tracker.pb.h"
#include "automsgs/task/voice.pb.h"

namespace autonomy {
namespace system {
namespace recorder {
namespace {

using automsgs::msgs::geometry_msgs::Pose2D;
using automsgs::msgs::geometry_msgs::Twist;
using automsgs::msgs::nav_msgs::Path;
using automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_CANCELED;
using automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_FAILED;
using automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_SUCCEEDED;
using automsgs::msgs::vehicle_msgs::RobotTaskStatus;

double Dist(const Pose2D& a, const Pose2D& b) {
    return std::hypot(a.x() - b.x(), a.y() - b.y());
}

double PointToSegment(const Pose2D& p, const Pose2D& a, const Pose2D& b) {
    const double vx = b.x() - a.x();
    const double vy = b.y() - a.y();
    const double len2 = vx * vx + vy * vy;
    if (len2 <= 1e-12) {
        return Dist(p, a);
    }
    double t = ((p.x() - a.x()) * vx + (p.y() - a.y()) * vy) / len2;
    t = std::clamp(t, 0.0, 1.0);
    Pose2D mid;
    mid.set_x(a.x() + t * vx);
    mid.set_y(a.y() + t * vy);
    return Dist(p, mid);
}

Pose2D Pose2DFromXY(double x, double y) {
    Pose2D p;
    p.set_x(x);
    p.set_y(y);
    return p;
}

autonomy::task::proto::TaskResult MakeTaskResult(RobotTaskStatus status,
                                                 const std::string& message) {
    autonomy::task::proto::TaskResult result;
    result.set_status(status);
    result.set_message(message);
    return result;
}

std::string TaskDomain(const std::string& channel) {
    const std::string prefix = "/autonomy/task/";
    if (channel.compare(0, prefix.size(), prefix) != 0) {
        return {};
    }
    auto rest = channel.substr(prefix.size());
    const auto slash = rest.find('/');
    return slash == std::string::npos ? rest : rest.substr(0, slash);
}

}  // namespace

std::optional<Pose2D> ParseOdom(const Sample& sample) {
    automsgs::msgs::nav_msgs::Odometry odom;
    if (!odom.ParseFromArray(sample.payload.data(),
                             static_cast<int>(sample.payload.size()))) {
        return std::nullopt;
    }
    const auto& p = odom.pose().pose().pose().position();
    const auto& q = odom.pose().pose().pose().orientation();
    const double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                  1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    Pose2D pose;
    pose.set_x(p.x());
    pose.set_y(p.y());
    pose.set_theta(yaw);
    return pose;
}

std::optional<Path> ParsePath(const Sample& sample) {
    Path path;
    if (!path.ParseFromArray(sample.payload.data(),
                             static_cast<int>(sample.payload.size())) ||
        path.poses_size() == 0) {
        return std::nullopt;
    }
    return path;
}

std::optional<Twist> ParseTwist(const Sample& sample) {
    const bool want_stamped =
        sample.message_type.find("TwistStamped") != std::string::npos;
    if (want_stamped) {
        automsgs::msgs::geometry_msgs::TwistStamped stamped;
        if (!stamped.ParseFromArray(sample.payload.data(),
                                    static_cast<int>(sample.payload.size()))) {
            return std::nullopt;
        }
        return stamped.twist();
    }
    Twist twist;
    if (twist.ParseFromArray(sample.payload.data(),
                             static_cast<int>(sample.payload.size()))) {
        return twist;
    }
    if (sample.message_type.empty()) {
        automsgs::msgs::geometry_msgs::TwistStamped stamped;
        if (stamped.ParseFromArray(sample.payload.data(),
                                   static_cast<int>(sample.payload.size())) &&
            stamped.SerializeAsString() == sample.payload) {
            return stamped.twist();
        }
    }
    return std::nullopt;
}

std::optional<double> ParseCost(const Sample& sample) {
    automsgs::msgs::std_msgs::Float32 f;
    if (f.ParseFromArray(sample.payload.data(),
                         static_cast<int>(sample.payload.size()))) {
        return static_cast<double>(f.data());
    }
    automsgs::msgs::std_msgs::Float32MultiArray arr;
    if (arr.ParseFromArray(sample.payload.data(),
                           static_cast<int>(sample.payload.size())) &&
        arr.data_size() > 0) {
        return static_cast<double>(arr.data(0));
    }
    return std::nullopt;
}

bool IsTeleopGoalChannel(const std::string& channel) {
    return channel.find("/teleop/goal") != std::string::npos;
}

bool IsTaskGoalChannel(const std::string& channel) {
    if (channel == "/goal_pose" || channel == "/goal_poses") {
        return true;
    }
    return channel.find("/autonomy/task/") != std::string::npos &&
           channel.find("/goal") != std::string::npos;
}

bool IsTaskFeedbackChannel(const std::string& channel) {
    return channel.find("/autonomy/task/") != std::string::npos &&
           channel.find("/feedback") != std::string::npos;
}

uint16_t SampleFlagsFor(const std::string& channel,
                        const proto::RecorderOptions& options) {
    uint16_t flags = 0;
    if (channel == options.odom_channel() || channel == "/odom") {
        flags |= kSampleOdom;
    }
    if (channel == options.plan_channel() || channel == "/plan") {
        flags |= kSamplePlan;
    }
    if (channel == options.cmd_vel_channel() || channel == "/cmd_vel") {
        flags |= kSampleCmdVel;
    }
    if (!options.bt_log_channel().empty() &&
        channel == options.bt_log_channel()) {
        flags |= kSampleBt;
    }
    if (!options.cost_channel().empty() &&
        channel == options.cost_channel()) {
        flags |= kSampleCost;
    }
    if (IsTaskGoalChannel(channel)) {
        flags |= kSampleGoal;
    }
    if (IsTaskFeedbackChannel(channel)) {
        flags |= kSampleFeedback;
    }
    if (IsTeleopGoalChannel(channel) ||
        channel == options.teleop_goal_channel()) {
        flags |= kSampleTeleop;
    }
    return flags;
}

std::optional<autonomy::task::proto::TaskResult> ParseTaskResult(
    const Sample& sample) {
    if (!IsTaskFeedbackChannel(sample.channel) || sample.payload.empty()) {
        return std::nullopt;
    }
    const std::string domain = TaskDomain(sample.channel);
    const auto* data = sample.payload.data();
    const int n = static_cast<int>(sample.payload.size());
    auto fail = [&](bool failed, bool canceled, const std::string& name) {
        if (failed) {
            return MakeTaskResult(ROBOT_TASK_STATUS_FAILED, name);
        }
        if (canceled) {
            return MakeTaskResult(ROBOT_TASK_STATUS_CANCELED, name);
        }
        return MakeTaskResult(ROBOT_TASK_STATUS_SUCCEEDED, name);
    };
    if (domain == "localization") {
        autonomy::task::proto::LocalizationFeedback fb;
        if (!fb.ParseFromArray(data, n)) {
            return std::nullopt;
        }
        return fail(fb.status() ==
                        autonomy::task::proto::LOCALIZATION_STATUS_FAILED,
                    fb.status() ==
                        autonomy::task::proto::LOCALIZATION_STATUS_CANCELED,
                    autonomy::task::proto::LocalizationStatus_Name(fb.status()));
    }
    if (domain == "navigation") {
        autonomy::task::proto::NavigationFeedback fb;
        if (!fb.ParseFromArray(data, n)) {
            return std::nullopt;
        }
        return fail(fb.status() == autonomy::task::proto::NAV_STATUS_FAILED,
                    fb.status() == autonomy::task::proto::NAV_STATUS_CANCELED,
                    autonomy::task::proto::NavigationStatus_Name(fb.status()));
    }
    if (domain == "mapping") {
        autonomy::task::proto::MappingFeedback fb;
        if (!fb.ParseFromArray(data, n)) {
            return std::nullopt;
        }
        return fail(fb.status() == autonomy::task::proto::MAP_STATUS_FAILED,
                    false,
                    autonomy::task::proto::MapStatus_Name(fb.status()));
    }
    if (domain == "charging") {
        autonomy::task::proto::ChargingFeedback fb;
        if (!fb.ParseFromArray(data, n)) {
            return std::nullopt;
        }
        return fail(fb.status() == autonomy::task::proto::DOCK_STATUS_FAILED,
                    fb.status() == autonomy::task::proto::DOCK_STATUS_CANCELED,
                    autonomy::task::proto::DockStatus_Name(fb.status()));
    }
    if (domain == "tracking") {
        autonomy::task::proto::TrackerFeedback fb;
        if (!fb.ParseFromArray(data, n)) {
            return std::nullopt;
        }
        return fail(fb.status() == autonomy::task::proto::TRACKER_STATUS_FAILED ||
                        fb.status() ==
                            autonomy::task::proto::TRACKER_STATUS_TARGET_LOST,
                    fb.status() == autonomy::task::proto::TRACKER_STATUS_CANCELED,
                    autonomy::task::proto::TrackerStatus_Name(fb.status()));
    }
    if (domain == "teleop") {
        autonomy::task::proto::TeleopFeedback fb;
        if (!fb.ParseFromArray(data, n)) {
            return std::nullopt;
        }
        return fail(fb.status() == autonomy::task::proto::TELEOP_STATUS_TIMEOUT ||
                        fb.status() ==
                            autonomy::task::proto::TELEOP_STATUS_REJECTED,
                    false,
                    autonomy::task::proto::TeleopStatus_Name(fb.status()));
    }
    if (domain == "voice") {
        autonomy::task::proto::VoiceFeedback fb;
        if (!fb.ParseFromArray(data, n)) {
            return std::nullopt;
        }
        return fail(fb.status() == autonomy::task::proto::VOICE_STATUS_FAILED,
                    fb.status() == autonomy::task::proto::VOICE_STATUS_CANCELED,
                    autonomy::task::proto::VoiceStatus_Name(fb.status()));
    }
    if (domain == "exploration") {
        autonomy::task::proto::ExplorationFeedback fb;
        if (!fb.ParseFromArray(data, n)) {
            return std::nullopt;
        }
        return fail(
            fb.status() == autonomy::task::proto::EXPLORATION_STATUS_FAILED,
            fb.status() == autonomy::task::proto::EXPLORATION_STATUS_CANCELED,
            autonomy::task::proto::ExplorationStatus_Name(fb.status()));
    }
    return std::nullopt;
}

std::optional<float> ParseLocalizationQuality(const Sample& sample) {
    if (TaskDomain(sample.channel) != "localization" || sample.payload.empty()) {
        return std::nullopt;
    }
    autonomy::task::proto::LocalizationFeedback fb;
    if (!fb.ParseFromArray(sample.payload.data(),
                           static_cast<int>(sample.payload.size()))) {
        return std::nullopt;
    }
    if (fb.localization_quality() <= 0.0f) {
        return std::nullopt;
    }
    return fb.localization_quality();
}

double DistanceToPath(const Pose2D& pose, const Path& path) {
    if (path.poses_size() == 0) {
        return 0.0;
    }
    auto Vertex = [](const automsgs::msgs::geometry_msgs::PoseStamped& ps) {
        return Pose2DFromXY(ps.pose().position().x(), ps.pose().position().y());
    };
    if (path.poses_size() == 1) {
        return Dist(pose, Vertex(path.poses(0)));
    }
    double best = Dist(pose, Vertex(path.poses(0)));
    for (int i = 1; i < path.poses_size(); ++i) {
        best = std::min(best, PointToSegment(pose, Vertex(path.poses(i - 1)),
                                             Vertex(path.poses(i))));
    }
    return best;
}

proto::ActionEvent TwistToAction(const Twist& twist, uint32_t seq,
                                 int64_t monotonic_ns) {
    proto::ActionEvent ev;
    ev.set_sequence(seq);
    ev.set_type("cmd_vel");
    ev.set_target("base");
    std::ostringstream params;
    params << twist.linear().x() << "," << twist.angular().z();
    ev.set_params(params.str());
    ev.set_monotonic_ns(monotonic_ns);
    return ev;
}

void PathTracker::SetOdom(Pose2D pose) { odom_ = std::move(pose); }

void PathTracker::SetPath(Path path) { path_ = std::move(path); }

std::optional<double> PathTracker::Deviation() const {
    if (!odom_ || path_.poses_size() == 0) {
        return std::nullopt;
    }
    return DistanceToPath(*odom_, path_);
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
