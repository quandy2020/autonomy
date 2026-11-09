/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/navigation_component.hpp"
#include "navigation/behavior/engine/behavior_session.hpp"

#include <chrono>
#include <thread>

#include "autolink/common/log.hpp"
#include "automsgs/msgs/builtin_interfaces/duration.pb.h"
#include "automsgs/msgs/geometry_msgs/pose_stamped.pb.h"
#include "automsgs/msgs/geometry_msgs/twist.pb.h"
#include "automsgs/msgs/status_msgs/status_msgs.pb.h"
#include "automsgs/msgs/time_utils.hpp"
#include "navigation/common/configuration_loader.hpp"
#include "navigation/common/pose2d.hpp"
#include "navigation/common/pose2d_convert.hpp"
#include "navigation/common/twist2d_convert.hpp"
#include "navigation/planning/simulation_plugins.hpp"

namespace navigation {
namespace {

std::int64_t StampNs(const observation::Observation& observation) {
    if (observation.has_header()) {
        return static_cast<std::int64_t>(
            automsgs::msgs::builtin_interfaces::TimeToNanoseconds(
                observation.header().stamp()));
    }
    return 0;
}

GoalSpec MakeActionGoal(
    const automsgs::msgs::geometry_msgs::PoseStamped& pose,
    const std::string& goal_id) {
    GoalSpec goal;
    goal.set_type(GoalSpec::TYPE_POSE);
    goal.set_goal_id(goal_id);
    goal.set_sequence(1);
    *goal.mutable_pose() = pose;
    return goal;
}

bool SamePoseStamped(
    const automsgs::msgs::geometry_msgs::PoseStamped& left,
    const automsgs::msgs::geometry_msgs::PoseStamped& right) {
    return left.header().stamp().sec() == right.header().stamp().sec() &&
           left.header().stamp().nanosec() ==
               right.header().stamp().nanosec() &&
           left.header().frame_id() == right.header().frame_id() &&
           left.pose().position().x() == right.pose().position().x() &&
           left.pose().position().y() == right.pose().position().y() &&
           left.pose().position().z() == right.pose().position().z() &&
           left.pose().orientation().x() == right.pose().orientation().x() &&
           left.pose().orientation().y() == right.pose().orientation().y() &&
           left.pose().orientation().z() == right.pose().orientation().z() &&
           left.pose().orientation().w() == right.pose().orientation().w();
}

}  // namespace

NavigationComponent::NavigationComponent() = default;

NavigationComponent::~NavigationComponent() {
    Clear();
}

bool NavigationComponent::Init() {
    config::NavigationConfig configuration;
    if (!common::LoadTextProto(ConfigFilePath(), &configuration).Ok()) {
        AERROR << "NavigationComponent failed to load "
               << ConfigFilePath();
        return false;
    }
    session_id_ = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    planning::RegisterSimulationPlugins();
    runner_ = Runner::Create(configuration);
    if (runner_ == nullptr || !runner_->Start()) {
        AERROR << "NavigationComponent Runner::Create/Start failed";
        return false;
    }
    if (configuration.has_initial_goal()) {
        goal_ = configuration.initial_goal();
        goal_sequence_ = goal_.sequence();
        goal_epoch_.fetch_add(1);
        has_goal_ = true;
        runner_->SetGoal(goal_);
    }
    command_writer_ = node_->CreateWriter<automsgs::msgs::geometry_msgs::Twist>(
        "/navigation/cmd_vel_raw");
    goal_reader_ = node_->CreateReader<GoalSpec>("/navigation/goal");
    pose_goal_reader_ = node_->CreateReader<
        automsgs::msgs::geometry_msgs::PoseStamped>("/goal_pose");
    behavior_request_reader_ =
        node_->CreateReader<BehaviorRequest>("/navigation/behavior_request");
    behavior_heartbeat_reader_ = node_->CreateReader<lifecycle::Heartbeat>(
        "/behavior/heartbeat", [this](const auto&) {
            behavior_received_ns_.store(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count());
        });
    status_writer_ =
        node_->CreateWriter<NavigationStatus>("/navigation/status");
    heartbeat_writer_ =
        node_->CreateWriter<lifecycle::Heartbeat>("/navigation/heartbeat");
    action_server_ = std::make_shared<autolink::action::SimpleActionServer<
        automsgs::actions::NavigateToPoseAction>>(
        node_, "/navigate_to_pose", [this] { RunNavigateToPose(); });
    if (pose_goal_reader_ == nullptr) {
        AWARN << "NavigationComponent failed to subscribe to /goal_pose";
    }
    const bool ok = command_writer_ != nullptr && goal_reader_ != nullptr &&
                    status_writer_ != nullptr && heartbeat_writer_ != nullptr &&
                    action_server_ != nullptr &&
                    behavior_request_reader_ != nullptr &&
                    behavior_heartbeat_reader_ != nullptr;
    if (!ok) {
        AERROR << "NavigationComponent failed to create IO"
               << " cmd=" << (command_writer_ != nullptr)
               << " goal=" << (goal_reader_ != nullptr)
               << " status=" << (status_writer_ != nullptr)
               << " heartbeat=" << (heartbeat_writer_ != nullptr)
               << " action=" << (action_server_ != nullptr)
               << " behavior_req=" << (behavior_request_reader_ != nullptr)
               << " behavior_hb=" << (behavior_heartbeat_reader_ != nullptr);
    }
    return ok;
}

bool NavigationComponent::Proc(
    const std::shared_ptr<observation::Observation>& observation) {
    const std::lock_guard<std::mutex> state_lock(goal_state_mutex_);
    if (observation == nullptr || runner_ == nullptr ||
        command_writer_ == nullptr) {
        return false;
    }
    common::Pose2D pose;
    common::Twist2D velocity;
    if (observation->has_odom() && observation->odom().has_pose()) {
        pose = common::FromProto(observation->odom().pose().pose());
        velocity = common::FromProto(observation->odom().twist().twist());
    }
    std::int64_t now_ns = StampNs(*observation);
    const auto steady_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    if (now_ns <= 0) {
        now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                     std::chrono::steady_clock::now().time_since_epoch())
                     .count();
    }

    {
        const std::lock_guard<std::mutex> lock(action_pose_mutex_);
        action_pose_ = pose;
        action_pose_stamp_ns_ = now_ns;
    }

    if (goal_reader_ != nullptr) {
        goal_reader_->Observe();
        if (const auto goal = goal_reader_->GetLatestObserved();
            goal != nullptr) {
            if (!last_topic_goal_.has_value() ||
                goal->sequence() != last_topic_goal_->sequence() ||
                goal->goal_id() != last_topic_goal_->goal_id() ||
                goal->type() != last_topic_goal_->type()) {
                last_topic_goal_ = *goal;
                goal_ = *goal;
                goal_sequence_ = goal->sequence();
                has_goal_ = true;
                failure_reason_.clear();
                // A topic goal is a new goal point and preempts any action
                // goal.
                goal_epoch_.fetch_add(1);
                action_active_ = false;
                runner_->SetGoal(goal_);
            }
        }
    }

    if (pose_goal_reader_ != nullptr) {
        pose_goal_reader_->Observe();
        if (const auto pose = pose_goal_reader_->GetLatestObserved();
            pose != nullptr) {
            if (!last_pose_goal_.has_value() ||
                !SamePoseStamped(*pose, *last_pose_goal_)) {
                last_pose_goal_ = *pose;
                goal_ = MakeActionGoal(*pose, "goal_pose");
                goal_epoch_.fetch_add(1);
                goal_.set_sequence(goal_epoch_.load());
                goal_sequence_ = goal_.sequence();
                has_goal_ = true;
                failure_reason_.clear();
                action_active_ = false;
                runner_->SetGoal(goal_);
            }
        }
    }

    const bool pose_valid =
        observation->has_odom() && observation->odom().has_pose() &&
        (observation->validity() & observation::OBSERVATION_FLAG_ODOM_VALID) !=
            0 &&
        common::IsFinite(pose);
    behavior_request_reader_->Observe();
    if (const auto request = behavior_request_reader_->GetLatestObserved();
        request &&
        behavior::MatchesBehaviorRequest(
            *request, MakeStatus(steady_ns, pose_valid), steady_ns)) {
        if (request->kind() == BehaviorRequest::RECOVER) {
            (void)runner_->RequestRecovery();
        } else {
            failure_reason_ = request->reason();
            runner_->FailGoal();
            if (!action_active_) {
                has_goal_ = false;
                runner_->ClearGoal();
            }
        }
    }
    behavior_heartbeat_reader_->Observe();
    const auto heartbeat = behavior_heartbeat_reader_->GetLatestObserved();
    const auto received = behavior_received_ns_.load();
    const bool behavior_fresh =
        heartbeat && heartbeat->module() == "behavior" && received > 0 &&
        steady_ns >= received && steady_ns - received <= 500000000LL &&
        steady_ns >= heartbeat->stamp_ns() &&
        steady_ns - heartbeat->stamp_ns() <= 500000000LL;
    // Missing odometry must not be interpreted as a valid pose at the origin:
    // it could advance a recovery phase or falsely complete the active goal.
    const auto command =
        pose_valid ? runner_->Tick(*observation, pose, velocity, behavior_fresh)
                   : std::optional<common::Twist2D>{};
    automsgs::msgs::geometry_msgs::Twist message;
    common::ToProto(command.value_or(common::Twist2D{}), &message);
    if (!command_writer_->Write(message)) {
        return false;
    }

    if (status_writer_ != nullptr) {
        NavigationStatus status = MakeStatus(steady_ns, pose_valid);
        status.set_sequence(++status_sequence_);
        status.set_stamp_ns(now_ns);
        (void)status_writer_->Write(status);
    }
    if (heartbeat_writer_ != nullptr) {
        lifecycle::Heartbeat heartbeat;
        heartbeat.set_module("navigation");
        heartbeat.set_sequence(++heartbeat_sequence_);
        heartbeat.set_stamp_ns(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
        heartbeat.set_state(lifecycle::MODULE_STATE_READY);
        (void)heartbeat_writer_->Write(heartbeat);
    }
    return true;
}

NavigationStatus NavigationComponent::MakeStatus(std::int64_t steady_ns,
                                                 bool pose_valid) const {
    NavigationStatus status;
    status.set_session_id(session_id_);
    status.set_goal_epoch(goal_epoch_.load());
    status.set_has_active_goal(has_goal_);
    status.set_pose_valid(pose_valid);
    status.set_steady_stamp_ns(steady_ns);
    status.set_goal_reached(runner_->IsGoalReached());
    status.set_recovery_state(runner_->RecoveryStatus());
    status.set_recovery_count(runner_->RecoveryCount());
    status.set_stage(runner_->RecoveryPhase());
    if (status.recovery_state() == RECOVERY_REQUIRED)
        status.set_reason("no_progress");
    if (status.recovery_state() == RECOVERY_FAILED)
        status.set_reason("recovery_blocked");
    if (!failure_reason_.empty())
        status.set_reason(failure_reason_);
    if (has_goal_)
        *status.mutable_goal() = goal_;
    return status;
}

void NavigationComponent::RunNavigateToPose() {
    if (action_server_ == nullptr) {
        return;
    }
    while (autolink::OK() && action_server_->IsServerActive()) {
        const auto goal = action_server_->GetCurrentGoal();
        if (!goal) {
            return;
        }
        std::uint64_t goal_epoch;
        {
            const std::lock_guard<std::mutex> state_lock(goal_state_mutex_);
            goal_epoch = goal_epoch_.fetch_add(1) + 1;
            action_active_ = true;
            goal_ = MakeActionGoal(goal->pose(), "navigate_to_pose");
            goal_.set_sequence(goal_epoch);
            goal_sequence_ = goal_epoch;
            has_goal_ = true;
            failure_reason_.clear();
            runner_->SetGoal(goal_);
        }
        const auto start = std::chrono::steady_clock::now();
        bool preempted = false;
        while (autolink::OK() && action_server_->IsServerActive()) {
            {
                const std::lock_guard<std::mutex> state_lock(goal_state_mutex_);
                if (goal_epoch_.load() != goal_epoch) {
                    // A newer /navigation/goal target took over; do not clear
                    // it.
                    action_active_ = false;
                    auto result = std::make_shared<
                        automsgs::actions::NavigateToPoseAction::Result>();
                    result->set_error_code(
                        automsgs::msgs::status_msgs::NAVIGATION_CANCELLED);
                    result->set_error_msg(
                        "superseded by /navigation/goal or /goal_pose");
                    action_server_->TerminateCurrent(result);
                    return;
                }
                if (action_server_->IsCancelRequested()) {
                    runner_->ClearGoal();
                    action_active_ = false;
                    has_goal_ = false;
                    auto result = std::make_shared<
                        automsgs::actions::NavigateToPoseAction::Result>();
                    result->set_error_code(
                        automsgs::msgs::status_msgs::NAVIGATION_CANCELLED);
                    result->set_error_msg("cancelled");
                    action_server_->TerminateCurrent(result);
                    return;
                }
                if (action_server_->IsPreemptRequested()) {
                    runner_->ClearGoal();
                    auto preempted_result = std::make_shared<
                        automsgs::actions::NavigateToPoseAction::Result>();
                    preempted_result->set_error_code(
                        automsgs::msgs::status_msgs::NAVIGATION_CANCELLED);
                    preempted_result->set_error_msg("preempted by newer goal");
                    action_server_->TerminateCurrent(preempted_result);
                    if (action_server_->AcceptPendingGoal() != nullptr) {
                        preempted = true;
                        break;
                    }
                    action_active_ = false;
                    has_goal_ = false;
                    return;
                }
                if (runner_->IsGoalFailed()) {
                    action_active_ = false;
                    has_goal_ = false;
                    runner_->ClearGoal();
                    auto result = std::make_shared<
                        automsgs::actions::NavigateToPoseAction::Result>();
                    result->set_error_code(
                        automsgs::msgs::status_msgs::NAVIGATION_PATH_BLOCKED);
                    result->set_error_msg(failure_reason_);
                    action_server_->TerminateCurrent(result);
                    return;
                }
                if (runner_->IsGoalReached()) {
                    action_active_ = false;
                    has_goal_ = false;
                    runner_->ClearGoal();
                    auto result = std::make_shared<
                        automsgs::actions::NavigateToPoseAction::Result>();
                    result->set_error_code(automsgs::msgs::status_msgs::OK);
                    action_server_->SucceededCurrent(result);
                    return;
                }
                common::Pose2D pose;
                std::int64_t pose_stamp_ns;
                {
                    const std::lock_guard<std::mutex> lock(action_pose_mutex_);
                    pose = action_pose_;
                    pose_stamp_ns = action_pose_stamp_ns_;
                }
                const auto now = std::chrono::steady_clock::now();
                auto feedback = std::make_shared<
                    automsgs::actions::NavigateToPoseAction::Feedback>();
                common::ToProto(pose, "map", pose_stamp_ns,
                                feedback->mutable_current_pose());
                const auto elapsed =
                    std::chrono::duration_cast<std::chrono::nanoseconds>(now -
                                                                         start);
                *feedback->mutable_navigation_time() =
                    automsgs::msgs::builtin_interfaces::DurationFromNanoseconds(
                        elapsed.count());
                const common::Pose2D target =
                    common::FromProto(goal->pose().pose());
                feedback->set_distance_remaining(
                    static_cast<float>(common::Distance(pose, target)));
                feedback->set_number_of_recoveries(runner_->RecoveryCount());
                action_server_->PublishFeedback(feedback);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (!preempted) {
            return;
        }
    }
}

void NavigationComponent::Clear() {
    // Deactivate while the shared pointer is still valid for the action worker.
    if (action_server_ != nullptr)
        action_server_->Deactivate();
    const std::lock_guard<std::mutex> state_lock(goal_state_mutex_);
    action_server_.reset();
    action_active_ = false;
    if (runner_ != nullptr) {
        runner_->Stop();
        runner_.reset();
    }
    command_writer_.reset();
    if (goal_reader_ != nullptr)
        goal_reader_->Shutdown();
    goal_reader_.reset();
    if (pose_goal_reader_ != nullptr)
        pose_goal_reader_->Shutdown();
    pose_goal_reader_.reset();
    if (behavior_request_reader_)
        behavior_request_reader_->Shutdown();
    if (behavior_heartbeat_reader_)
        behavior_heartbeat_reader_->Shutdown();
    behavior_request_reader_.reset();
    behavior_heartbeat_reader_.reset();
    status_writer_.reset();
    heartbeat_writer_.reset();
}

}  // namespace navigation

AUTOLINK_REGISTER_COMPONENT(navigation::NavigationComponent)
