/*
 * Copyright 2026 The Openbot Authors
 */
/** @file navigation.cpp @brief Native navigation admission, readiness and result ownership. */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "autolink/autolink.hpp"
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include "autonomy/common/logging.hpp"
#include "autonomy/common/nav_log.hpp"
#include "autonomy/task/navigation/navigation.hpp"
#include "autonomy/task/navigation/path_update.hpp"

namespace autonomy {
namespace task {
namespace {

constexpr char kGlobalFrame[] = "map";
constexpr char kRobotBaseFrame[] = "base_link";
constexpr char kDefaultPlannerId[] = "navfn_planner";
constexpr char kDefaultControllerId[] = "mppi_controller";
constexpr char kDefaultSmootherId[] = "simple_smoother";
constexpr double kDefaultGoalReachedTol = 0.10;
constexpr float kMaxReportedDistance = 100.f;

automsgs::msgs::geometry_msgs::PoseStamped ToPoseStamped(
    const ::automsgs::msgs::geometry_msgs::PoseStamped& proto)
{
    return proto;
}

bool SameNavigationGoals(const ::autonomy::task::proto::NavigationGoal& a,
                         const ::autonomy::task::proto::NavigationGoal& b)
{
    // Anonymous requests and REPLAN always create a new invocation. Exact
    // retry matching includes modes, plugins and all task constraints.
    if (a.command() != ::autonomy::task::proto::NAV_CMD_START ||
        b.command() != ::autonomy::task::proto::NAV_CMD_START ||
        !a.has_header() || !b.has_header() || a.header().task_id().empty() ||
        a.header().task_id() != b.header().task_id() ||
        a.header().client_id() != b.header().client_id() || a.goals().empty()) {
        return false;
    }
    return a.SerializeAsString() == b.SerializeAsString();
}

/// Validate all geometry before disturbing a previously accepted invocation.
bool ValidNavigationGeometry(const ::autonomy::task::proto::NavigationGoal& goal)
{
    if (goal.goals().empty()) return false;
    if (goal.mode() != ::autonomy::task::proto::NAV_MODE_UNSPECIFIED &&
        goal.mode() != ::autonomy::task::proto::NAV_MODE_SINGLE_POSE &&
        goal.mode() != ::autonomy::task::proto::NAV_MODE_THROUGH_POSES) return false;
    if (goal.mode() == ::autonomy::task::proto::NAV_MODE_SINGLE_POSE &&
        goal.goals_size() != 1) return false;
    const auto& frame = goal.goals(0).header().frame_id();
    if (frame.empty()) return false;
    for (const auto& pose : goal.goals()) {
        const auto& p = pose.pose().position();
        const auto& q = pose.pose().orientation();
        if (pose.header().frame_id() != frame || !std::isfinite(p.x()) ||
            !std::isfinite(p.y()) || !std::isfinite(p.z()) ||
            !std::isfinite(q.x()) || !std::isfinite(q.y()) ||
            !std::isfinite(q.z()) || !std::isfinite(q.w()))
            return false;
        const double norm = std::hypot(std::hypot(q.x(), q.y()),
                                       std::hypot(q.z(), q.w()));
        if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) return false;
    }
    return true;
}

}  // namespace

using RobotTaskType = ::automsgs::msgs::vehicle_msgs::RobotTaskType;
namespace task_proto = ::autonomy::task::proto;

RobotTaskType NavigationTask::GetTaskType() const
{
    return RobotTaskType::ROBOT_TASK_NAVIGATION;
}

bool NavigationTask::EnsureNavigationClient()
{
    return static_cast<bool>(navigation());
}

bool NavigationTask::NavigationServersReady() const
{
    return navigation() && navigation()->IsPlanningReady() &&
           navigation()->IsControlReady();
}

std::chrono::milliseconds NavigationTask::NavigationReadyTimeout() const
{
    return std::chrono::seconds(20);
}

void NavigationTask::CancelRemoteNavigation()
{
    if (navigation()) navigation()->CancelActiveMotion();
}

NavigationTask::~NavigationTask()
{
    try { Shutdown(); }
    catch (...) { AERROR << "nav shutdown failed during destruction"; }
}

void NavigationTask::FinishPendingGoal(uint64_t epoch)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (pending_epoch_ == epoch) {
        pending_goal_.reset();
        pending_epoch_ = 0;
    }
}

bool NavigationTask::CancelMatchingGoal(const std::string& expected_task_id)
{
    uint64_t canceled_epoch = 0;
    bool changed = false;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        const bool all = expected_task_id.empty();
        const bool pending_matches = pending_goal_ &&
            (all || pending_goal_->header().task_id() == expected_task_id);
        const bool active_matches = active_epoch_ != 0 && active_goal_ &&
            (all || active_goal_->header().task_id() == expected_task_id);
        if (!all && !pending_matches && !active_matches) return false;
        // Targeted cancellation must not supersede an unrelated newer waiter.
        if (all || pending_matches ||
            (active_matches && goal_epoch_.load() == active_epoch_)) {
            if (goal_epoch_.load() != std::numeric_limits<uint64_t>::max())
                ++goal_epoch_;
            if (all || pending_matches) {
                pending_goal_.reset();
                pending_epoch_ = 0;
            }
        }
        changed = pending_matches || active_matches;
        if (active_matches && canceling_epoch_ != active_epoch_) {
            canceled_epoch = active_epoch_;
            canceling_epoch_ = canceled_epoch;
            motion_epoch_.store(0);
        }
    }
    ready_wake_.notify_all();
    if (canceled_epoch == 0) return changed;

    // Revoke before waiting for replacement. A callback cannot wait for the
    // lock held by an external owner that is joining that very callback.
    std::unique_lock<std::mutex> goal_lock(goal_mutex_, std::defer_lock);
    if (!runner()->IsWorkerThread()) goal_lock.lock();
    if (tree_epoch_.load() != canceled_epoch) return changed;
    try {
        StopTree();
        if (runner()->state() == BtRunState::kFailed) motion_fault_.store(true);
        CancelRemoteNavigation();
    } catch (...) {
        motion_fault_.store(true);
        ready_wake_.notify_all();
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (tree_epoch_.load() == canceled_epoch) {
            motion_epoch_.store(0);
            SetLifecycle(TaskLifecycle::kFailed);
            SetProgress(progress_.progress(), "navigation cancellation failed");
        }
        throw;
    }
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (tree_epoch_.load() == canceled_epoch) {
            active_goal_.reset();
            active_epoch_ = 0;
            canceling_epoch_ = 0;
            initial_distance_ = -1.f;
            SetLifecycle(motion_fault_.load() || runner()->state() == BtRunState::kFailed
                         ? TaskLifecycle::kFailed : TaskLifecycle::kCanceled);
        }
    }
    return changed;
}

bool NavigationTask::Cancel() { return CancelMatchingGoal({}); }

bool NavigationTask::CancelForTask(const std::string& expected_task_id)
{
    return !expected_task_id.empty() && CancelMatchingGoal(expected_task_id);
}

void NavigationTask::Shutdown()
{
    shutting_down_.store(true);
    ready_wake_.notify_all();
    std::exception_ptr failure;
    try { Cancel(); }
    catch (...) { failure = std::current_exception(); }
    // The base invokes virtual Cancel again. Never hold goal_mutex_ around it;
    // a preceding replacement observes shutting_down_ and cannot install work.
    try { BtTaskApp::Shutdown(); }
    catch (...) { if (!failure) failure = std::current_exception(); }
    if (failure) std::rethrow_exception(failure);
}

bool NavigationTask::Pause() { return PauseMatchingGoal({}); }

bool NavigationTask::PauseMatchingGoal(const std::string& expected_task_id)
{
    std::unique_lock<std::mutex> goal_lock(goal_mutex_, std::defer_lock);
    if (!runner()->IsWorkerThread()) goal_lock.lock();
    bool paused_pending = false;
    uint64_t epoch = 0;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (shutting_down_.load() || motion_fault_.load()) return false;
        const bool all = expected_task_id.empty();
        if (pending_goal_ &&
            (all || pending_goal_->header().task_id() == expected_task_id)) {
            if (goal_epoch_.load() != std::numeric_limits<uint64_t>::max())
                ++goal_epoch_;
            pending_goal_.reset();
            pending_epoch_ = 0;
            paused_pending = true;
        }
        if (active_epoch_ != 0 && active_goal_ &&
            (all || active_goal_->header().task_id() == expected_task_id) &&
            Lifecycle() == TaskLifecycle::kRunning &&
            motion_epoch_.load() == active_epoch_) {
            epoch = active_epoch_;
            motion_epoch_.store(0);
            PauseTree();
            SetLifecycle(TaskLifecycle::kPaused);
        }
    }
    ready_wake_.notify_all();
    if (epoch == 0) return paused_pending;
    try { CancelRemoteNavigation(); }
    catch (...) {
        motion_fault_.store(true);
        ready_wake_.notify_all();
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (active_epoch_ == epoch) {
            SetLifecycle(TaskLifecycle::kFailed);
            SetProgress(progress_.progress(), "navigation pause cancellation failed");
        }
        throw;
    }
    return true;
}

bool NavigationTask::Resume() { return ResumeMatchingGoal({}); }

bool NavigationTask::ResumeMatchingGoal(const std::string& expected_task_id)
{
    std::unique_lock<std::mutex> goal_lock(goal_mutex_, std::defer_lock);
    if (!runner()->IsWorkerThread()) goal_lock.lock();
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (shutting_down_.load() || motion_fault_.load() || active_epoch_ == 0 ||
        canceling_epoch_ == active_epoch_ || !active_goal_ ||
        (!expected_task_id.empty() &&
         active_goal_->header().task_id() != expected_task_id) ||
        Lifecycle() != TaskLifecycle::kPaused) return false;
    SetLifecycle(TaskLifecycle::kRunning);
    motion_epoch_.store(active_epoch_);
    if (ResumeTree()) return true;
    motion_epoch_.store(0);
    SetLifecycle(TaskLifecycle::kPaused);
    return false;
}

bool NavigationTask::ShouldUpdateGoalIdentity(const task_proto::NavigationGoal& goal) const
{
    return goal.command() == task_proto::NAV_CMD_START ||
           goal.command() == task_proto::NAV_CMD_REPLAN;
}

bool NavigationTask::BeforeTreeTick()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (shutting_down_.load() || motion_fault_.load() || active_epoch_ == 0 ||
        active_epoch_ != tree_epoch_.load()) return false;
    // The runner rechecks its paused flag after this callback. A concurrent
    // pause must skip the tick rather than cancel the retained invocation.
    return Lifecycle() == TaskLifecycle::kPaused ||
           motion_epoch_.load() == active_epoch_;
}

bool NavigationTask::OnTreeInitialize(
    const task_proto::TaskServerOptions& /*options*/)
{
    const bool ready = EnsureNavigationClient();
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    shutting_down_.store(!ready);
    if (ready) {
        motion_fault_.store(false);
        active_goal_.reset();
        pending_goal_.reset();
        active_epoch_ = pending_epoch_ = canceling_epoch_ = 0;
        motion_epoch_.store(0);
        SetLifecycle(TaskLifecycle::kIdle);
    }
    return ready;
}

void NavigationTask::ApplyGoalToBlackboard(
    const BT::Blackboard::Ptr& blackboard,
    const task_proto::NavigationGoal& goal)
{
    if (!blackboard) {
        return;
    }
    if (goal.has_plugins()) {
        if (!goal.plugins().planner_id().empty()) {
            blackboard->set("default_planner_id", goal.plugins().planner_id());
        }
        if (!goal.plugins().controller_id().empty()) {
            blackboard->set("default_controller_id",
                            goal.plugins().controller_id());
        }
    }

    std::vector<automsgs::msgs::geometry_msgs::PoseStamped> goals;
    goals.reserve(static_cast<size_t>(goal.goals_size()));
    for (const auto& pose_proto : goal.goals()) {
        goals.push_back(ToPoseStamped(pose_proto));
    }
    blackboard->set("goals", goals);
    blackboard->set("number_of_goals", static_cast<int>(goals.size()));
    if (!goals.empty()) {
        blackboard->set("goal", goals.back());
    }
}

void NavigationTask::PopulateBlackboard(const BT::Blackboard::Ptr& blackboard)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!blackboard) {
        return;
    }

    if (navigation()) {
        blackboard->set(navigation::kNavigationClientBlackboardKey,
                        navigation());
    }

    blackboard->set("global_frame", std::string(kGlobalFrame));
    blackboard->set("robot_base_frame", std::string(kRobotBaseFrame));
    blackboard->set("default_planner_id", std::string(kDefaultPlannerId));
    blackboard->set("default_controller_id", std::string(kDefaultControllerId));
    blackboard->set("default_smoother_id", std::string(kDefaultSmootherId));
    blackboard->set("goal_reached_tol", kDefaultGoalReachedTol);
    blackboard->set("initial_pose_received", true);
    blackboard->set("local_survival_timeout", 120.0);
    blackboard->set("goal_epoch", static_cast<int>(replan_epoch_));

    if (active_goal_.has_value()) {
        blackboard->set("navigation_task_id", active_goal_->header().task_id());
        blackboard->set("navigation_client_id", active_goal_->header().client_id());
        blackboard->set("navigation_invocation_epoch", active_epoch_);
        ApplyGoalToBlackboard(blackboard, *active_goal_);
    }
}

std::string NavigationTask::ResolveTreeForGoal(
    const task_proto::NavigationGoal& goal) const
{
    if (goal.has_plugins() && !goal.plugins().behavior_tree().empty()) {
        return PickTreePath(goal.plugins().behavior_tree(), "");
    }
    const bool through_poses =
        goal.mode() == task_proto::NAV_MODE_THROUGH_POSES ||
        (goal.mode() == task_proto::NAV_MODE_UNSPECIFIED &&
         goal.goals_size() > 1);
    if (through_poses) {
        return profile().AlternateTreePath(config_directory());
    }
    return profile().DefaultTreePath(config_directory());
}

bool NavigationTask::OnGoal(const task_proto::NavigationGoal& goal)
{
    using Command = task_proto::NavigationCommand;
    const std::string target = goal.has_header() ? goal.header().task_id() : std::string{};
    switch (goal.command()) {
    case Command::NAV_CMD_STOP:
    case Command::NAV_CMD_CANCEL:
        return CancelMatchingGoal(target);
    case Command::NAV_CMD_PAUSE:
        return PauseMatchingGoal(target);
    case Command::NAV_CMD_RESUME:
        return ResumeMatchingGoal(target);
    case Command::NAV_CMD_START:
    case Command::NAV_CMD_REPLAN:
        break;
    default:
        return false;
    }
    if (runner()->IsWorkerThread() || shutting_down_.load() || motion_fault_.load() ||
        !ValidNavigationGeometry(goal) || !EnsureNavigationClient()) return false;
    const std::string tree = ResolveTreeForGoal(goal);
    std::error_code error;
    if (tree.empty() || !std::filesystem::is_regular_file(tree, error)) return false;

    uint64_t epoch;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (shutting_down_.load() || motion_fault_.load() ||
            goal_epoch_.load() == std::numeric_limits<uint64_t>::max() ||
            replan_epoch_ == static_cast<uint64_t>(std::numeric_limits<int>::max()))
            return false;
        // An exact named retry is accepted without restarting, but it must
        // supersede an older pending replacement before publishing its identity.
        if (active_goal_ && active_epoch_ != 0 &&
            canceling_epoch_ != active_epoch_ &&
            (motion_epoch_.load() == active_epoch_ || Lifecycle() == TaskLifecycle::kPaused) &&
            SameNavigationGoals(*active_goal_, goal)) {
            if (pending_goal_) {
                ++goal_epoch_;
                pending_goal_.reset();
                pending_epoch_ = 0;
                ready_wake_.notify_all();
            }
            return true;
        }
        epoch = ++goal_epoch_;
        pending_epoch_ = epoch;
        pending_goal_ = goal;
    }
    ready_wake_.notify_all();

    std::unique_lock<std::mutex> goal_lock(goal_mutex_, std::defer_lock);
    bool replacing = false;
    try {
        const auto deadline = std::chrono::steady_clock::now() +
                              NavigationReadyTimeout();
        bool ready = false;
        while (!shutting_down_.load() && !motion_fault_.load() &&
               goal_epoch_.load() == epoch) {
            if (NavigationServersReady()) { ready = true; break; }
            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline) break;
            std::unique_lock<std::mutex> wait_lock(ready_mutex_);
            ready_wake_.wait_until(wait_lock,
                std::min(deadline, now + std::chrono::milliseconds(50)),
                [this, epoch] {
                    return shutting_down_.load() || motion_fault_.load() ||
                           goal_epoch_.load() != epoch;
                });
        }
        if (!ready) { FinishPendingGoal(epoch); return false; }

        // Readiness never owns this mutex, so cancellation can interrupt it.
        // No lifecycle lock is held while halting/joining a preceding worker.
        goal_lock.lock();
        bool had_active = false;
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            if (shutting_down_.load() || motion_fault_.load() ||
                goal_epoch_.load() != epoch || pending_epoch_ != epoch) {
                FinishPendingGoal(epoch);
                return false;
            }
            had_active = active_epoch_ != 0;
            motion_epoch_.store(0);
            replacing = true;
        }
        StopTree();
        if (had_active && runner()->state() == BtRunState::kFailed)
            motion_fault_.store(true);
        if (had_active) CancelRemoteNavigation();
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            if (shutting_down_.load() || motion_fault_.load() ||
                goal_epoch_.load() != epoch || pending_epoch_ != epoch) {
                FinishPendingGoal(epoch);
                return false;
            }
            active_goal_ = goal;
            active_epoch_ = epoch;
            canceling_epoch_ = 0;
            tree_epoch_.store(epoch);
            motion_epoch_.store(epoch);
            pending_goal_.reset();
            pending_epoch_ = 0;
            ++replan_epoch_;
            initial_distance_ = -1.f;
            // Publish running before Run: a fast native completion must not be
            // overwritten with running state by the admitting caller afterward.
            SetLifecycle(TaskLifecycle::kRunning);
            SetProgress(0.f, "bt:" + tree);
        }
        if (!StartTree(tree)) {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            if (tree_epoch_.load() == epoch) {
                active_goal_.reset();
                active_epoch_ = 0;
                motion_epoch_.store(0);
                SetLifecycle(TaskLifecycle::kFailed);
                SetProgress(0.f, "navigation tree start failed");
            }
            return false;
        }
        bool current;
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            current = !shutting_down_.load() && !motion_fault_.load() &&
                      goal_epoch_.load() == epoch && canceling_epoch_ != epoch;
            if (!current && tree_epoch_.load() == epoch) motion_epoch_.store(0);
        }
        if (!current) {
            StopTree();
            CancelRemoteNavigation();
            return false;
        }
        return true;
    } catch (...) {
        FinishPendingGoal(epoch);
        // A failed readiness probe preserves accepted work. Exceptions after
        // replacement begins may leave remote cancellation uncertain.
        if (!replacing) throw;
        motion_fault_.store(true);
        motion_epoch_.store(0);
        ready_wake_.notify_all();
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        SetLifecycle(TaskLifecycle::kFailed);
        SetProgress(progress_.progress(), "navigation admission failed");
        throw;
    }
}

void NavigationTask::OnTreeTick()
{
    uint64_t epoch;
    bool terminal = false;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        epoch = active_epoch_;
        if (epoch == 0 || epoch != tree_epoch_.load()) return;
        switch (runner()->state()) {
        case BtRunState::kSucceeded:
            SetLifecycle(motion_fault_.load() ? TaskLifecycle::kFailed
                                             : TaskLifecycle::kSucceeded);
            SetProgress(1.f, "bt succeeded: " + runner()->active_tree());
            terminal = true;
            break;
        case BtRunState::kFailed:
            SetLifecycle(TaskLifecycle::kFailed);
            SetProgress(progress_.progress(), "bt failed: " + runner()->active_tree());
            terminal = true;
            break;
        case BtRunState::kCanceled:
            SetLifecycle(motion_fault_.load() ? TaskLifecycle::kFailed
                                             : TaskLifecycle::kCanceled);
            terminal = true;
            break;
        default:
            break;
        }
        if (terminal) {
            // Clear ownership before a remote cancellation can synchronously
            // reenter Cancel. The preceding worker still owns its tree epoch.
            active_goal_.reset();
            active_epoch_ = 0;
            motion_epoch_.store(0);
            initial_distance_ = -1.f;
        } else if (Lifecycle() != TaskLifecycle::kRunning ||
                   motion_epoch_.load() != epoch) {
            return;
        }
    }
    if (terminal) {
        try { CancelRemoteNavigation(); }
        catch (...) {
            motion_fault_.store(true);
            ready_wake_.notify_all();
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            if (tree_epoch_.load() == epoch) {
                SetLifecycle(TaskLifecycle::kFailed);
                SetProgress(progress_.progress(), "navigation terminal cancellation failed");
            }
            throw;
        }
        return;
    }
    float distance = 0.f;
    const bool has_distance = navigation() && navigation()->TryGetDistanceToGoal(&distance);
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (active_epoch_ != epoch || Lifecycle() != TaskLifecycle::kRunning ||
        motion_epoch_.load() != epoch) return;
    if (has_distance && std::isfinite(distance) && distance >= 0.f) {
        if (initial_distance_ < 0.f) initial_distance_ = std::max(distance, 0.1f);
        SetProgress(std::clamp(1.f - distance / initial_distance_, 0.f, 0.99f),
                    "following path");
    } else {
        SetProgress(std::min(0.5f, progress_.progress() + 0.005f),
                    "bt tick: " + runner()->active_tree());
    }
}

task_proto::NavigationStatus NavigationTask::MapStatus() const
{
    using Status = task_proto::NavigationStatus;
    switch (Lifecycle()) {
    case TaskLifecycle::kIdle:
        return Status::NAV_STATUS_IDLE;
    case TaskLifecycle::kRunning: {
        float distance = 0.f;
        if (navigation() && navigation()->TryGetDistanceToGoal(&distance)) {
            return Status::NAV_STATUS_NAVIGATING;
        }
        return Status::NAV_STATUS_PLANNING;
    }
    case TaskLifecycle::kPaused:
        return Status::NAV_STATUS_PAUSED;
    case TaskLifecycle::kSucceeded:
        return Status::NAV_STATUS_SUCCEEDED;
    case TaskLifecycle::kFailed:
        return Status::NAV_STATUS_FAILED;
    case TaskLifecycle::kCanceled:
        return Status::NAV_STATUS_CANCELED;
    }
    return Status::NAV_STATUS_UNKNOWN;
}

void NavigationTask::FillFeedback(task_proto::NavigationFeedback* feedback) const
{
    feedback->set_status(MapStatus());
    *feedback->mutable_progress() = progress_;

    if (active_goal_.has_value()) {
        const int total = active_goal_->goals_size();
        feedback->set_total_waypoints(total);
        feedback->set_current_waypoint_index(total > 0 ? total - 1 : 0);
        if (total > 0) {
            *feedback->mutable_current_pose() =
                active_goal_->goals(total - 1);
        }
    }

    float distance = 0.f;
    if (navigation() && navigation()->TryGetDistanceToGoal(&distance)) {
        feedback->set_distance_remaining(
            std::min(distance, kMaxReportedDistance));
    }
}

void NavigationTask::FillResult(task_proto::NavigationResult* result) const
{
    *result->mutable_result() = MakeTaskResult();
    result->set_final_status(MapStatus());
}

}  // namespace task
}  // namespace autonomy
