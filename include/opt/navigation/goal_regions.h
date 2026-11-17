#ifndef OPT_NAVIGATION_GOAL_REGIONS_H_
#define OPT_NAVIGATION_GOAL_REGIONS_H_

#include "opt/navigation/goal_adapter.h"

namespace opt::navigation {

struct GeometricRegion {
  std::string id;
  bool obstacle = false;
  Pose2d center;
  double radius = 0.0;
  std::vector<CellIndex> cells;
};

// Grounded geometry from an externally supplied map. No RGB class semantics,
// foundation-model inference, or depth reconstruction is performed here.
class GeometricSemanticParser {
 public:
  std::vector<GeometricRegion> Parse(const GridMap& map) const;
};

// Associate an obstructed goal with its connected obstacle instance. The search
// stays inside a bounded neighborhood of the original goal. Candidate Safety
// and reachability checks still decide which actual poses can be selected.
class ConnectedGoalRegionProvider final : public IGoalRegionProvider {
 public:
  explicit ConnectedGoalRegionProvider(double search_radius = 2.0);
  std::optional<GoalRegion> Resolve(const GoalRequest& goal,
                                   const GridMap& map) const override;

 private:
  double search_radius_;
};

}  // namespace opt::navigation
#endif  // OPT_NAVIGATION_GOAL_REGIONS_H_
