/**
 * @file navigator.cc
 * @brief Internal Opt navigator implementation reused by native navigation.
 * @details Keeps the geometry implementation independent of ROS and map ownership.
 */
#include "autonomy/common/navigation/navigator.hpp"   

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace autonomy::navigation {

Navigator::Navigator(NavigatorOptions options, std::shared_ptr<const IGoalRegionProvider> regions)
    : options_(options), adapter_(options.safety, options.adaptation, std::move(regions)) {
  const double values[] = {options.adaptation_period, options.original_retry_period,
      options.approach_distance, options.approach_cooldown, options.max_map_age, options.max_pose_age,
      options.position_tolerance, options.yaw_tolerance, options.planning_timeout,
      options.failed_candidate_ttl};
  for (double value : values) {
    if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument("Invalid navigator timing or tolerance");
  }
  if (!std::isfinite(options.clearance_drop_fraction) || options.clearance_drop_fraction <= 0.0 ||
      options.clearance_drop_fraction >= 1.0 || options.max_planning_failures < 1) {
    throw std::invalid_argument("Invalid navigation retry or approach configuration");
  }
}

void Navigator::SetGoal(const GoalRequest& goal, double now) {
  if (!IsFinite(goal.original) || !IsValidGoalOrientation(goal) || goal.frame_id.empty() || !std::isfinite(now) ||
      now < last_now_ || !std::isfinite(goal.timeout) || goal.timeout <= 0.0 ||
      (goal.region && (goal.region->id.empty() || !IsFinite(goal.region->center) ||
                       !std::isfinite(goal.region->radius) || goal.region->radius <= 0.0))) {
    throw std::invalid_argument("Invalid goal, region, or monotonic timestamp");
  }
  ++goal_epoch_;
  goal_revision_ = 0;
  original_ = goal;
  map_ = {};
  safety_.reset();
  adapted_.reset();
  active_path_.reset();
  pending_.reset();
  exclusions_.clear();
  last_adaptation_ = {};
  goal_started_ = now;
  last_now_ = now;
  last_adaptation_time_ = last_original_retry_ = last_approach_ = -std::numeric_limits<double>::infinity();
  planning_failures_ = 0;
  baseline_clearance_ = 0.0;
  state_ = NavigationState::kPlanning;
  trigger_ = AdaptationTrigger::kNone;
  reason_ = "new goal accepted";
}

void Navigator::Cancel() {
  ++goal_epoch_;
  original_.reset();
  adapted_.reset();
  active_path_.reset();
  pending_.reset();
  state_ = NavigationState::kIdle;
  trigger_ = AdaptationTrigger::kNone;
  reason_ = "cancelled";
}

NavigationUpdate Navigator::Current() const {
  NavigationUpdate update;
  update.state = state_;
  update.goal = adapted_;
  update.path = active_path_;
  update.stop_required = state_ != NavigationState::kFollowing || !active_path_;
  update.reason = reason_;
  update.goal_epoch = goal_epoch_;
  update.goal_revision = goal_revision_;
  update.trigger = trigger_;
  return update;
}

void Navigator::Block(const std::string& reason) {
  pending_.reset();
  active_path_.reset();
  state_ = NavigationState::kBlocked;
  reason_ = reason;
}

AdaptationInput Navigator::Adaptation(bool exact_only, bool preserve_mode, double now) const {
  AdaptationInput input;
  input.goal = *original_;
  input.robot_pose = robot_pose_;
  input.map = map_.grid;
  input.safety = safety_;
  input.previous = adapted_;
  input.exact_only = exact_only;
  input.preserve_mode = preserve_mode;
  for (const auto& exclusion : exclusions_) if (exclusion.until > now) input.excluded.push_back(exclusion.pose);
  return input;
}

void Navigator::InstallGoal(const AdaptedGoal& goal) {
  const bool entering_alternative = goal.mode != GoalMode::kExact &&
      (!adapted_ || adapted_->mode == GoalMode::kExact);
  const bool changed = !adapted_ || adapted_->mode != goal.mode ||
      adapted_->region_id != goal.region_id || Distance(adapted_->pose, goal.pose) > 1e-9 ||
      std::abs(NormalizeAngle(adapted_->pose.yaw - goal.pose.yaw)) > 1e-9;
  adapted_ = goal;
  if (entering_alternative) last_original_retry_ = last_now_;
  if (changed) {
    ++goal_revision_;
    pending_.reset();
    active_path_.reset();
    state_ = NavigationState::kPlanning;
    reason_ = "adapted goal updated";
  }
}

Path Navigator::RemainingPath(const Path& path) const {
  Path remaining;
  remaining.frame_id = path.frame_id;
  if (path.poses.empty()) return remaining;
  std::size_t closest = 0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    if (Distance(robot_pose_, path.poses[i]) < Distance(robot_pose_, path.poses[closest])) closest = i;
  }
  remaining.poses.push_back(robot_pose_);
  remaining.poses.insert(remaining.poses.end(), path.poses.begin() + closest, path.poses.end());
  return remaining;
}

void Navigator::FinishIfReached() {
  // An outstanding newer request can never be completed by an older path.
  if (!adapted_ || !active_path_ || pending_ || state_ != NavigationState::kFollowing ||
      Distance(robot_pose_, adapted_->pose) > options_.position_tolerance ||
      std::abs(NormalizeAngle(robot_pose_.yaw - adapted_->pose.yaw)) > options_.yaw_tolerance) return;
  const bool accept = adapted_->mode == GoalMode::kExact ||
      (adapted_->mode == GoalMode::kApproximate && original_->completion != CompletionPolicy::kExactOnly) ||
      (adapted_->mode == GoalMode::kRegionFallback && original_->completion == CompletionPolicy::kAcceptRegion);
  active_path_.reset();
  if (!accept) {
    state_ = NavigationState::kWaitingForOriginal;
    reason_ = "alternative reached; original goal still required";
    return;
  }
  if (adapted_->mode == GoalMode::kExact) state_ = NavigationState::kReachedExact;
  else if (adapted_->mode == GoalMode::kApproximate) state_ = NavigationState::kReachedApproximate;
  else state_ = NavigationState::kReachedRegion;
  reason_ = "reached " + std::string(ToString(adapted_->mode));
}

void Navigator::PlanningFailed(double now, const std::string& reason) {
  if (adapted_) exclusions_.push_back({adapted_->pose, now + options_.failed_candidate_ttl});
  pending_.reset();
  active_path_.reset();
  adapted_.reset();
  ++planning_failures_;
  if (planning_failures_ >= options_.max_planning_failures) {
    state_ = NavigationState::kFailed;
    reason_ = "planning retry limit: " + reason;
  } else {
    state_ = NavigationState::kBlocked;
    reason_ = "planning rejected: " + reason;
    last_adaptation_time_ = -std::numeric_limits<double>::infinity();
  }
}

PlanAcceptance Navigator::AcceptPlan(const PlanningResult& result, double now) {
  if (!pending_ || IsTerminal(state_) || !original_ || !adapted_ ||
      result.request_id != pending_->request_id || result.goal_epoch != goal_epoch_ ||
      result.goal_revision != goal_revision_ || result.map_revision != map_.revision) {
    return PlanAcceptance::kStale;
  }
  if (!std::isfinite(now) || now < last_now_ || now - map_.stamp > options_.max_map_age ||
      now - pending_started_ > options_.planning_timeout) {
    Block("planning result or map expired");
    return PlanAcceptance::kStale;
  }
  last_now_ = now;
  if (!result.success || result.path.poses.empty() || result.path.frame_id != original_->frame_id ||
      !IsFinite(result.path.poses.back()) || Distance(result.path.poses.back(), adapted_->pose) > 1e-6 ||
      std::abs(NormalizeAngle(result.path.poses.back().yaw - adapted_->pose.yaw)) > 1e-6) {
    PlanningFailed(now, result.reason.empty() ? "invalid path endpoint" : result.reason);
    return PlanAcceptance::kRejected;
  }
  const auto whole = safety_->VerifyPath(result.path);
  const auto remaining = safety_->VerifyPath(RemainingPath(result.path));
  if (!whole.safe || !remaining.safe) {
    PlanningFailed(now, whole.safe ? remaining.reason : whole.reason);
    return PlanAcceptance::kRejected;
  }
  active_path_ = std::make_shared<const Path>(result.path);
  baseline_clearance_ = remaining.clearance;
  pending_.reset();
  planning_failures_ = 0;
  state_ = NavigationState::kFollowing;
  reason_ = "validated current-version path";
  return PlanAcceptance::kAccepted;
}

NavigationUpdate Navigator::Tick(const NavigationInput& input) {
  trigger_ = AdaptationTrigger::kNone;
  if (!original_ || IsTerminal(state_)) return Current();
  if (!std::isfinite(input.now) || input.now < last_now_) {
    Block("invalid monotonic timestamp");
    return Current();
  }
  last_now_ = input.now;
  // Task deadlines still expire when perception is unavailable or stale.
  if (input.now - goal_started_ >= original_->timeout) {
    Block("goal deadline exceeded");
    state_ = NavigationState::kFailed;
    return Current();
  }
  if (!IsFinite(input.robot_pose) || !std::isfinite(input.pose_stamp) ||
      input.pose_stamp > input.now + 1e-9 || input.now - input.pose_stamp > options_.max_pose_age ||
      !input.map.grid || input.frame_id != original_->frame_id ||
      input.map.grid->frame_id() != original_->frame_id || !std::isfinite(input.map.stamp) ||
      input.map.stamp > input.now + 1e-9 || input.now - input.map.stamp > options_.max_map_age ||
      input.map.revision == 0 || input.map.revision < map_.revision) {
    Block("invalid pose/frame/map revision, or stale pose/map");
    return Current();
  }
  robot_pose_ = input.robot_pose;
  const bool map_changed = !safety_ || input.map.revision != map_.revision ||
                           &safety_->map() != input.map.grid.get();
  map_ = input.map;
  if (map_changed) {
    safety_ = std::make_shared<GeometrySafety>(map_.grid, options_.safety);
    pending_.reset();
  }
  if (!safety_->VerifyPose(robot_pose_).safe) {
    Block("robot footprint is not safe in current map");
    return Current();
  }

  bool safety_event = false;
  if (adapted_ && !safety_->VerifyPose(adapted_->pose).safe) {
    Block("adapted goal became unsafe");
    adapted_.reset();
    safety_event = true;
  }
  SafetyResult current_path;
  if (active_path_) {
    current_path = safety_->VerifyPath(RemainingPath(*active_path_));
    if (!current_path.safe) {
      Block("remaining path became unsafe: " + current_path.reason);
      safety_event = true;
    }
  }
  if (pending_ && input.now - pending_started_ > options_.planning_timeout) {
    PlanningFailed(input.now, "planner timeout");
    if (IsTerminal(state_)) return Current();
  }

  bool retried_original = false;
  if (adapted_ && adapted_->mode != GoalMode::kExact &&
      input.now - last_original_retry_ >= options_.original_retry_period) {
    last_original_retry_ = input.now;
    last_adaptation_ = adapter_.Adapt(Adaptation(true, false, input.now));
    trigger_ = AdaptationTrigger::kOriginalRetry;
    retried_original = true;
    if (last_adaptation_.selected) InstallGoal(*last_adaptation_.selected);
  }

  const bool approach = active_path_ && adapted_ && input.goal_region_observed &&
      Distance(robot_pose_, adapted_->pose) < options_.approach_distance &&
      baseline_clearance_ > 0.0 &&
      current_path.clearance < baseline_clearance_ * (1.0 - options_.clearance_drop_fraction) &&
      input.now - last_approach_ >= options_.approach_cooldown;
  // Waiting for the original target does not endlessly reposition the robot.
  const bool periodic = state_ != NavigationState::kWaitingForOriginal &&
      input.now - last_adaptation_time_ >= options_.adaptation_period;
  const bool initial = !adapted_ && (map_changed || periodic);
  if (safety_event || approach || initial || (periodic && !pending_ && !retried_original)) {
    trigger_ = safety_event ? AdaptationTrigger::kSafety : approach ? AdaptationTrigger::kApproach :
               initial ? AdaptationTrigger::kInitial : AdaptationTrigger::kPeriodic;
    last_adaptation_time_ = input.now;
    if (approach) last_approach_ = input.now;
    const bool preserve_mode = adapted_.has_value() && !safety_event;
    last_adaptation_ = adapter_.Adapt(Adaptation(false, preserve_mode, input.now));
    if (!last_adaptation_.selected && preserve_mode) {
      // Preserve mode for ordinary approach refinement; unavailable targets use
      // a separate recovery search that is allowed to change the mode.
      last_adaptation_ = adapter_.Adapt(Adaptation(false, false, input.now));
    }
    if (!last_adaptation_.selected) {
      adapted_.reset();
      Block(last_adaptation_.reason);
      return Current();
    }
    InstallGoal(*last_adaptation_.selected);
    if (approach && current_path.safe) baseline_clearance_ = current_path.clearance;
  }

  if (adapted_ && !active_path_ && !pending_ && state_ != NavigationState::kWaitingForOriginal) {
    PlanningRequest request;
    request.request_id = ++next_request_id_;
    request.goal_epoch = goal_epoch_;
    request.goal_revision = goal_revision_;
    request.map = map_;
    request.start = robot_pose_;
    request.target = adapted_->pose;
    pending_ = request;
    pending_started_ = input.now;
    state_ = NavigationState::kPlanning;
    reason_ = "awaiting planner";
    auto update = Current();
    update.planning_request = request;
    return update;
  }
  // A validated retained path can resume after transient input faults.
  if (active_path_) state_ = NavigationState::kFollowing;
  FinishIfReached();
  return Current();
}

}  // namespace autonomy::navigation
