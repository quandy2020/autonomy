/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/navigation_component.hpp"

#include <chrono>
#include <thread>

#include "automsgs/msgs/geometry_msgs/twist.pb.h"
#include "automsgs/msgs/builtin_interfaces/duration.pb.h"
#include "automsgs/msgs/status_msgs/status_msgs.pb.h"
#include "automsgs/msgs/time_utils.hpp"
#include "navigation/common/configuration_loader.hpp"
#include "navigation/common/pose2d.hpp"
#include "navigation/common/pose2d_convert.hpp"
#include "navigation/common/twist2d_convert.hpp"
#include "navigation/planning/simulation_plugins.hpp"

namespace navigation
{
namespace
{

std::int64_t StampNs(const observation::Observation& observation)
{
    if (observation.has_header())
    {
        return static_cast<std::int64_t>(
            automsgs::msgs::builtin_interfaces::TimeToNanoseconds(
                observation.header().stamp()));
    }
    return 0;
}

GoalSpec MakeActionGoal(const automsgs::msgs::geometry_msgs::PoseStamped& pose)
{
    GoalSpec goal;
    goal.set_type(GoalSpec::TYPE_POSE);
    goal.set_goal_id("navigate_to_pose");
    goal.set_sequence(1);
    *goal.mutable_pose() = pose;
    return goal;
}

}  // namespace

NavigationComponent::NavigationComponent() = default;

NavigationComponent::~NavigationComponent()
{
    Clear();
}

bool NavigationComponent::Init()
{
    config::NavigationConfig configuration;
    if (!common::LoadTextProto(ConfigFilePath(), &configuration).Ok())
    {
        return false;
    }
    planning::RegisterSimulationPlugins();
    runner_ = Runner::Create(configuration);
    if (runner_ == nullptr || !runner_->Start())
    {
        return false;
    }
    if (configuration.has_initial_goal())
    {
        goal_ = configuration.initial_goal();
        goal_sequence_ = goal_.sequence();
        has_goal_ = true;
        runner_->SetGoal(goal_);
    }
    command_writer_ = node_->CreateWriter<automsgs::msgs::geometry_msgs::Twist>(
        "/navigation/cmd_vel_raw");
    goal_reader_ = node_->CreateReader<GoalSpec>("/navigation/goal");
    status_writer_ =
        node_->CreateWriter<NavigationStatus>("/navigation/status");
    heartbeat_writer_ =
        node_->CreateWriter<lifecycle::Heartbeat>("/navigation/heartbeat");
    action_server_ = std::make_shared<autolink::action::SimpleActionServer<
        automsgs::actions::NavigateToPoseAction>>(
        node_, "/navigate_to_pose", [this] { RunNavigateToPose(); });
    return command_writer_ != nullptr && goal_reader_ != nullptr &&
           status_writer_ != nullptr && heartbeat_writer_ != nullptr &&
           action_server_ != nullptr;
}

bool NavigationComponent::Proc(
    const std::shared_ptr<observation::Observation>& observation)
{
    if (observation == nullptr || runner_ == nullptr || command_writer_ == nullptr)
    {
        return false;
    }
    common::Pose2D pose;
    common::Twist2D velocity;
    if (observation->has_odom() && observation->odom().has_pose())
    {
        pose = common::FromProto(observation->odom().pose().pose());
        velocity = common::FromProto(observation->odom().twist().twist());
    }
    std::int64_t now_ns = StampNs(*observation);
    if (now_ns <= 0)
    {
        now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                     std::chrono::steady_clock::now().time_since_epoch())
                     .count();
    }

    {
        const std::lock_guard<std::mutex> lock(action_pose_mutex_);
        action_pose_ = pose;
    }

    if (goal_reader_ != nullptr)
    {
        if (const auto goal = goal_reader_->GetLatestObserved(); goal != nullptr)
        {
            if (!has_goal_ || goal->sequence() != goal_sequence_ ||
                goal->goal_id() != goal_.goal_id() ||
                goal->type() != goal_.type())
            {
                goal_ = *goal;
                goal_sequence_ = goal->sequence();
                has_goal_ = true;
                best_distance_ = 1e9;
                last_progress_ns_ = now_ns;
                // A topic goal is a new goal point and preempts any action goal.
                goal_epoch_.fetch_add(1);
                action_active_ = false;
                runner_->SetGoal(goal_);
            }
        }
    }

    if (!action_active_ && has_goal_ && goal_.has_pose() && !runner_->IsGoalReached())
    {
        const common::Pose2D target = common::FromProto(goal_.pose().pose());
        const double distance = common::Distance(pose, target);
        if (distance < best_distance_ - 0.1)
        {
            best_distance_ = distance;
            last_progress_ns_ = now_ns;
        }
        else if (now_ns - last_progress_ns_ > 8000000000LL)
        {
            // Simple recovery: clear the stuck goal and re-submit it once.
            runner_->ClearGoal();
            runner_->SetGoal(goal_);
            best_distance_ = distance;
            last_progress_ns_ = now_ns;
        }
    }

    const auto command = runner_->Tick(*observation, pose, velocity);
    automsgs::msgs::geometry_msgs::Twist message;
    common::ToProto(command.value_or(common::Twist2D{}), &message);
    if (!command_writer_->Write(message))
    {
        return false;
    }

    if (status_writer_ != nullptr)
    {
        NavigationStatus status;
        status.set_sequence(++status_sequence_);
        status.set_stamp_ns(now_ns);
        status.set_goal_reached(runner_->IsGoalReached());
        if (has_goal_)
        {
            *status.mutable_goal() = goal_;
        }
        (void)status_writer_->Write(status);
    }
    if (heartbeat_writer_ != nullptr)
    {
        lifecycle::Heartbeat heartbeat;
        heartbeat.set_module("navigation");
        heartbeat.set_sequence(++heartbeat_sequence_);
        heartbeat.set_stamp_ns(now_ns);
        heartbeat.set_state(lifecycle::MODULE_STATE_READY);
        (void)heartbeat_writer_->Write(heartbeat);
    }
    return true;
}

void NavigationComponent::RunNavigateToPose()
{
    if (action_server_ == nullptr)
    {
        return;
    }
    while (autolink::OK())
    {
        const auto goal = action_server_->GetCurrentGoal();
        if (!goal)
        {
            return;
        }
        const std::uint64_t goal_epoch = goal_epoch_.fetch_add(1) + 1;
        action_active_ = true;
        runner_->SetGoal(MakeActionGoal(goal->pose()));
        const auto start = std::chrono::steady_clock::now();
        bool preempted = false;
        while (autolink::OK())
        {
            if (goal_epoch_.load() != goal_epoch)
            {
                // A newer /navigation/goal target took over; do not clear it.
                action_active_ = false;
                auto result = std::make_shared<
                    automsgs::actions::NavigateToPoseAction::Result>();
                result->set_error_code(
                    automsgs::msgs::status_msgs::NAVIGATION_CANCELLED);
                result->set_error_msg("superseded by /navigation/goal");
                action_server_->TerminateCurrent(result);
                return;
            }
            if (action_server_->IsCancelRequested())
            {
                runner_->ClearGoal();
                action_active_ = false;
                auto result = std::make_shared<
                    automsgs::actions::NavigateToPoseAction::Result>();
                result->set_error_code(
                    automsgs::msgs::status_msgs::NAVIGATION_CANCELLED);
                result->set_error_msg("cancelled");
                action_server_->TerminateCurrent(result);
                return;
            }
            if (action_server_->IsPreemptRequested())
            {
                runner_->ClearGoal();
                auto preempted_result = std::make_shared<
                    automsgs::actions::NavigateToPoseAction::Result>();
                preempted_result->set_error_code(
                    automsgs::msgs::status_msgs::NAVIGATION_CANCELLED);
                preempted_result->set_error_msg("preempted by newer goal");
                action_server_->TerminateCurrent(preempted_result);
                if (action_server_->AcceptPendingGoal() != nullptr)
                {
                    preempted = true;
                    break;
                }
                action_active_ = false;
                return;
            }
            if (runner_->IsGoalReached())
            {
                action_active_ = false;
                auto result = std::make_shared<
                    automsgs::actions::NavigateToPoseAction::Result>();
                result->set_error_code(automsgs::msgs::status_msgs::OK);
                action_server_->SucceededCurrent(result);
                return;
            }
            common::Pose2D pose;
            {
                const std::lock_guard<std::mutex> lock(action_pose_mutex_);
                pose = action_pose_;
            }
            const auto now = std::chrono::steady_clock::now();
            auto feedback = std::make_shared<
                automsgs::actions::NavigateToPoseAction::Feedback>();
            common::ToProto(pose, "map",
                            std::chrono::duration_cast<std::chrono::nanoseconds>(
                                now.time_since_epoch())
                                .count(),
                            feedback->mutable_current_pose());
            const auto elapsed =
                std::chrono::duration_cast<std::chrono::nanoseconds>(now - start);
            *feedback->mutable_navigation_time() =
                automsgs::msgs::builtin_interfaces::DurationFromNanoseconds(
                    elapsed.count());
            const common::Pose2D target =
                common::FromProto(goal->pose().pose());
            feedback->set_distance_remaining(
                static_cast<float>(common::Distance(pose, target)));
            action_server_->PublishFeedback(feedback);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (!preempted)
        {
            return;
        }
    }
}

void NavigationComponent::Clear()
{
    action_server_.reset();
    action_active_ = false;
    if (runner_ != nullptr)
    {
        runner_->Stop();
        runner_.reset();
    }
    command_writer_.reset();
    goal_reader_.reset();
    status_writer_.reset();
    heartbeat_writer_.reset();
}

}  // namespace navigation

AUTOLINK_REGISTER_COMPONENT(navigation::NavigationComponent)
