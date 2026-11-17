#ifndef OPT_NAVIGATION_GOAL_CONFIRMATION_H_
#define OPT_NAVIGATION_GOAL_CONFIRMATION_H_

#include "opt/navigation/goal_adapter.h"

namespace opt::navigation {

// Evidence, not physical ground truth. Includes the complete circular footprint,
// ground and overhead space; missing geometry remains unknown.
TargetStatus ClassifyTarget(const Pose2d& target, const GridMap& map,
                            const SafetyOptions& options);

struct ObservationOptions {
  double sensor_range = 4.0;
  double min_motion = 0.30;
  std::size_t max_candidates = 512;
  std::size_t max_sample_cells = 50000;
  std::size_t max_reachability_expansions = 50000;
};

// Selects a staging pose facing B, never B itself. A ray toward B is only a
// geometric visibility heuristic; sensor coverage must be confirmed externally.
// All candidates and routes use known-space footprint safety.
class ObservationGoalSelector {
 public:
  ObservationGoalSelector(SafetyOptions safety = {}, ObservationOptions options = {});
  AdaptationResult Select(const AdaptationInput& input,
                         const std::vector<Pose2d>& suggestions = {}) const;
 private:
  bool ObservationOpportunity(const Pose2d& pose, const Pose2d& target,
                              const GridMap& map) const;
  SafetyOptions safety_;
  ObservationOptions options_;
};

}  // namespace opt::navigation
#endif  // OPT_NAVIGATION_GOAL_CONFIRMATION_H_
