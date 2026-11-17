#ifndef OPT_NAVIGATION_POSE_SAMPLER_H_
#define OPT_NAVIGATION_POSE_SAMPLER_H_

#include "opt/navigation/goal_adapter.h"

namespace opt::navigation {
// Coarse coverage followed by bounded local refinement. Sampling alone neither
// establishes safety nor proves that a region is unreachable.
class PoseSampler {
 public:
  explicit PoseSampler(AdaptationOptions options = {});
  Pose2d Orient(Pose2d pose, const AdaptationInput& input) const;
  std::vector<Pose2d> SampleRing(const AdaptationInput& input, std::size_t budget) const;
  std::vector<Pose2d> SampleRegion(const AdaptationInput& input, const GoalRegion& region,
                                 std::size_t budget) const;
  std::vector<Pose2d> Refine(const AdaptationInput& input, const Pose2d& seed,
                           double radius, std::size_t budget) const;

 private:
  AdaptationOptions options_;
};
}  // namespace opt::navigation
#endif  // OPT_NAVIGATION_POSE_SAMPLER_H_
