#ifndef OPT_NAVIGATION_GOAL_ADAPTER_H_
#define OPT_NAVIGATION_GOAL_ADAPTER_H_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "opt/navigation/reachability.h"

namespace opt::navigation {

struct AdaptationOptions {
  double ring_min_radius = 0.30;
  double ring_max_radius = 0.80;
  int ring_count = 3;
  int angle_count = 16;
  double fallback_radius = 2.0;
  std::size_t max_candidates = 2048;
  // Bound Dijkstra work per selection. Budget exhaustion is not proof of
  // unreachability. Zero permits a full field for offline callers.
  std::size_t max_reachability_expansions = 50000;
  double preferred_clearance = 1.0;
  double goal_weight = 0.50;
  double clearance_weight = 0.15;
  double path_weight = 0.20;
  double continuity_weight = 0.15;
  double min_score_improvement = 0.03;
  double switch_distance = 0.15;
  // Hold small position/yaw changes unless safety or the orientation policy requires a switch.
  double switch_yaw = 0.10;
  double yaw_continuity_scale = 0.20;
  double heading_weight = 0.0;
  double region_sample_spacing = 0.20;
  // Fine search remains bounded by max_candidates and the permitted region.
  double refinement_radius = 0.20;
  int refinement_levels = 2;
  int refinement_seeds = 4;
};

// Geometry is the built-in baseline. A future semantic/depth adapter can provide
// a grounded region through this interface without changing the navigation core.
class IGoalRegionProvider {
 public:
  virtual ~IGoalRegionProvider() = default;
  virtual std::optional<GoalRegion> Resolve(const GoalRequest& goal,
                                           const GridMap& map) const = 0;
};
class GeometricRegionProvider final : public IGoalRegionProvider {
 public:
  explicit GeometricRegionProvider(double radius) : radius_(radius) {}
  std::optional<GoalRegion> Resolve(const GoalRequest& goal,
                                   const GridMap& map) const override;
 private:
  double radius_;
};

struct AdaptationInput {
  GoalRequest goal;
  Pose2d robot_pose;
  std::shared_ptr<const GridMap> map;
  std::optional<AdaptedGoal> previous;
  std::vector<Pose2d> excluded;
  bool exact_only = false;
  bool preserve_mode = false;
  bool region_only = false;  // Recovery searches must not silently retry point/ring modes.
  bool enable_refinement = true;
  // Reuse the current tick's distance field; it must match map and safety options.
  std::shared_ptr<const GeometrySafety> safety;
  std::function<bool()> cancel_checker;
};
struct PoseScoreComponents {
  double goal_proximity = 0.0;
  double clearance = 0.0;
  double path_efficiency = 0.0;
  double continuity = 0.0;
  double heading_alignment = 0.0;
  double total = 0.0;
};
struct CandidateEvaluation {
  Pose2d pose;
  GoalMode mode = GoalMode::kExact;
  bool accepted = false;
  double score = 0.0;
  double path_length = 0.0;
  SafetyResult safety;
  std::string reason;
  PoseScoreComponents score_components;
};
struct AdaptationResult {
  std::optional<AdaptedGoal> selected;
  std::vector<CandidateEvaluation> candidates;
  std::string reason;
  bool retained_previous = false;
  bool reachability_budget_exhausted = false;
  std::size_t reachability_expansions = 0;
  bool sampling_budget_exhausted = false;
};

class GoalAdapter {
 public:
  GoalAdapter(SafetyOptions safety = {}, AdaptationOptions adaptation = {},
              std::shared_ptr<const IGoalRegionProvider> regions = nullptr);
  AdaptationResult Adapt(const AdaptationInput& input) const;

 private:
  SafetyOptions safety_options_;
  AdaptationOptions options_;
  std::shared_ptr<const IGoalRegionProvider> regions_;
};

}  // namespace opt::navigation
#endif  // OPT_NAVIGATION_GOAL_ADAPTER_H_
