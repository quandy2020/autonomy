#ifndef OPT_NAVIGATION_NAVIGATOR_H_
#define OPT_NAVIGATION_NAVIGATOR_H_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "opt/navigation/goal_adapter.h"

namespace opt::navigation {

struct NavigatorOptions {
  SafetyOptions safety;
  AdaptationOptions adaptation;
  double adaptation_period = 1.0;
  double original_retry_period = 5.0;
  double approach_distance = 3.0;
  double clearance_drop_fraction = 0.20;
  double approach_cooldown = 2.0;
  double max_map_age = 1.0;
  double max_pose_age = 0.5;
  double position_tolerance = 0.08;
  double yaw_tolerance = 0.10;
  double planning_timeout = 2.0;
  double failed_candidate_ttl = 5.0;
  int max_planning_failures = 5;
};
enum class PlanAcceptance { kAccepted, kStale, kRejected };

// All methods are called on one owner thread. A planner may run elsewhere with
// an immutable PlanningRequest and return a result to the owner thread.
// The caller obeys stop_required; no actuator/ROS/TF dependency exists here.
class Navigator {
 public:
  explicit Navigator(NavigatorOptions options = {},
      std::shared_ptr<const IGoalRegionProvider> regions = nullptr);
  void SetGoal(const GoalRequest& goal, double now);
  void Cancel();
  NavigationUpdate Tick(const NavigationInput& input);
  PlanAcceptance AcceptPlan(const PlanningResult& result, double now);
  NavigationUpdate Current() const;
  const AdaptationResult& last_adaptation() const { return last_adaptation_; }

 private:
  struct Exclusion { Pose2d pose; double until; };
  AdaptationInput Adaptation(bool exact_only, bool preserve_mode, double now) const;
  void InstallGoal(const AdaptedGoal& goal);
  void Block(const std::string& reason);
  void PlanningFailed(double now, const std::string& reason);
  Path RemainingPath(const Path& path) const;
  void FinishIfReached();
  NavigatorOptions options_;
  GoalAdapter adapter_;
  std::optional<GoalRequest> original_;
  std::optional<AdaptedGoal> adapted_;
  MapSnapshot map_;
  Pose2d robot_pose_;
  std::shared_ptr<const GeometrySafety> safety_;
  std::shared_ptr<const Path> active_path_;
  std::optional<PlanningRequest> pending_;
  std::vector<Exclusion> exclusions_;
  AdaptationResult last_adaptation_;
  NavigationState state_ = NavigationState::kIdle;
  AdaptationTrigger trigger_ = AdaptationTrigger::kNone;
  std::string reason_;
  std::uint64_t goal_epoch_ = 0;
  std::uint64_t goal_revision_ = 0;
  std::uint64_t next_request_id_ = 0;
  double goal_started_ = 0.0;
  double last_now_ = -std::numeric_limits<double>::infinity();
  double last_adaptation_time_ = -std::numeric_limits<double>::infinity();
  double last_original_retry_ = -std::numeric_limits<double>::infinity();
  double last_approach_ = -std::numeric_limits<double>::infinity();
  double pending_started_ = 0.0;
  double baseline_clearance_ = 0.0;
  int planning_failures_ = 0;
};

}  // namespace opt::navigation
#endif  // OPT_NAVIGATION_NAVIGATOR_H_
