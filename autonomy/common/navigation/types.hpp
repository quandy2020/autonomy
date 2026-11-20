#ifndef OPT_NAVIGATION_TYPES_H_
#define OPT_NAVIGATION_TYPES_H_

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace opt::navigation {

constexpr double kPi = 3.14159265358979323846;

struct Pose2d {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

inline bool IsFinite(const Pose2d& pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}
inline double Distance(const Pose2d& a, const Pose2d& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}
inline double NormalizeAngle(double angle) { return std::remainder(angle, 2.0 * kPi); }

enum class GoalMode { kExact, kApproximate, kRegionFallback, kUnknownApproach };
enum class TargetStatus { kUnknown, kFree, kBlocked };
enum class PerceptionStatus { kNormal, kLocallyDegraded, kUnavailable };
enum class SemanticStatus { kNotRequired, kAllowed, kUnknown, kRejected };
enum class GoalIssue {
  kNone, kInvalidInput, kPerceptionUnreliable, kUnconfirmed, kTargetBlocked,
  kPathBlocked, kExecutionFailed, kSemanticUnconfirmed, kSemanticRejected, kDeadline, kSearchBudgetExhausted
};
enum class CompletionPolicy { kExactOnly, kAcceptApproximate, kAcceptRegion };
enum class GoalOrientationPolicy {
  kPreserveRequested,
  kFaceOriginalGoal,
  kFaceReferencePoint,
  kAlignApproach
};
enum class AdaptationTrigger { kNone, kInitial, kPeriodic, kApproach, kSafety, kOriginalRetry };
enum class NavigationState {
  kIdle, kPlanning, kFollowing, kWaitingForOriginal,
  kReachedExact, kReachedApproximate, kReachedRegion, kBlocked, kFailed,
  kWaitingForConfirmation, kExploring
};

inline const char* ToString(GoalMode mode) {
  switch (mode) {
    case GoalMode::kExact: return "EXACT_GOAL";
    case GoalMode::kApproximate: return "APPROXIMATE_GOAL";
    case GoalMode::kRegionFallback: return "REGION_FALLBACK";
    case GoalMode::kUnknownApproach: return "UNKNOWN_APPROACH";
  }
  return "UNKNOWN";
}
inline const char* ToString(NavigationState state) {
  switch (state) {
    case NavigationState::kIdle: return "IDLE";
    case NavigationState::kPlanning: return "PLANNING";
    case NavigationState::kFollowing: return "FOLLOWING";
    case NavigationState::kWaitingForOriginal: return "WAITING_FOR_ORIGINAL";
    case NavigationState::kReachedExact: return "REACHED_EXACT";
    case NavigationState::kReachedApproximate: return "REACHED_APPROXIMATE";
    case NavigationState::kReachedRegion: return "REACHED_REGION";
    case NavigationState::kBlocked: return "BLOCKED";
    case NavigationState::kFailed: return "FAILED";
    case NavigationState::kWaitingForConfirmation: return "WAITING_FOR_CONFIRMATION";
    case NavigationState::kExploring: return "EXPLORING";
  }
  return "UNKNOWN";
}
inline const char* ToString(GoalIssue issue) {
  switch (issue) {
    case GoalIssue::kNone: return "NONE";
    case GoalIssue::kInvalidInput: return "INVALID_INPUT";
    case GoalIssue::kPerceptionUnreliable: return "PERCEPTION_UNRELIABLE";
    case GoalIssue::kUnconfirmed: return "UNCONFIRMED";
    case GoalIssue::kTargetBlocked: return "TARGET_BLOCKED";
    case GoalIssue::kPathBlocked: return "PATH_BLOCKED";
    case GoalIssue::kExecutionFailed: return "EXECUTION_FAILED";
    case GoalIssue::kSemanticUnconfirmed: return "SEMANTIC_UNCONFIRMED";
    case GoalIssue::kSemanticRejected: return "SEMANTIC_REJECTED";
    case GoalIssue::kDeadline: return "DEADLINE";
    case GoalIssue::kSearchBudgetExhausted: return "SEARCH_BUDGET_EXHAUSTED";
  }
  return "UNKNOWN";
}
inline bool IsTerminal(NavigationState state) {
  return state == NavigationState::kReachedExact ||
         state == NavigationState::kReachedApproximate ||
         state == NavigationState::kReachedRegion || state == NavigationState::kFailed;
}

struct GoalRegion {
  std::string id;
  Pose2d center;
  double radius = 0.0;
};

struct GoalRequest {
  Pose2d original;
  std::string frame_id = "map";
  bool allow_approximate = true;
  bool allow_region_fallback = true;
  CompletionPolicy completion = CompletionPolicy::kExactOnly;
  std::optional<GoalRegion> region;
  double timeout = 60.0;
  bool requires_semantic_confirmation = false;
  // Applies to alternative poses only. The original requested pose is immutable.
  GoalOrientationPolicy orientation_policy = GoalOrientationPolicy::kPreserveRequested;
  std::optional<Pose2d> orientation_reference;  // Same frame as original.
};

inline bool IsValidGoalOrientation(const GoalRequest& goal) {
  if (goal.orientation_reference && !IsFinite(*goal.orientation_reference)) return false;
  switch (goal.orientation_policy) {
    case GoalOrientationPolicy::kPreserveRequested:
    case GoalOrientationPolicy::kFaceOriginalGoal:
    case GoalOrientationPolicy::kAlignApproach:
      return true;
    case GoalOrientationPolicy::kFaceReferencePoint:
      return goal.orientation_reference.has_value();
  }
  return false;
}

struct AdaptedGoal {
  Pose2d pose;
  GoalMode mode = GoalMode::kExact;
  std::string region_id;
  double clearance = 0.0;
  double path_length = 0.0;
  double score = 0.0;
};

struct Path {
  std::vector<Pose2d> poses;
  std::string frame_id;
};

class GridMap;
// Revisions must increase when geometry changes. A new stamp with the same
// revision certifies freshly observed, unchanged geometry. Publish immutable maps.
struct MapSnapshot {
  std::shared_ptr<const GridMap> grid;
  std::uint64_t revision = 0;
  double stamp = 0.0;
};

struct PlanningRequest {
  std::uint64_t request_id = 0;
  std::uint64_t goal_epoch = 0;
  std::uint64_t goal_revision = 0;
  MapSnapshot map;
  Pose2d start;
  Pose2d target;
};

struct PlanningResult {
  std::uint64_t request_id = 0;
  std::uint64_t goal_epoch = 0;
  std::uint64_t goal_revision = 0;
  std::uint64_t map_revision = 0;
  bool success = false;
  Path path;
  std::string reason;
};

// Optional per-goal freshness. Without it, map.stamp certifies all known cells.
// With it, only a matching, recent observation confirms the target footprint.
struct GoalObservation {
  double stamp = 0.0;
  std::uint64_t map_revision = 0, goal_epoch = 0;
  bool valid = false;
};
struct NavigationInput {
  Pose2d robot_pose;
  double pose_stamp = 0.0;
  std::string frame_id = "map";
  MapSnapshot map;
  double now = 0.0;  // Seconds from one caller-owned monotonic clock.
  bool goal_region_observed = false;
  PerceptionStatus perception = PerceptionStatus::kNormal;
  // Mandatory for local degradation: the provider marked unreliable cells unknown.
  bool invalid_regions_marked_unknown = false;
  std::optional<GoalObservation> goal_observation;
  SemanticStatus semantics = SemanticStatus::kNotRequired;
};

struct NavigationUpdate {
  NavigationState state = NavigationState::kIdle;
  std::optional<AdaptedGoal> goal;
  std::optional<PlanningRequest> planning_request;
  std::shared_ptr<const Path> path;
  bool stop_required = true;
  std::string reason;
  std::uint64_t goal_epoch = 0;
  std::uint64_t goal_revision = 0;
  AdaptationTrigger trigger = AdaptationTrigger::kNone;
};

}  // namespace opt::navigation
#endif  // OPT_NAVIGATION_TYPES_H_
