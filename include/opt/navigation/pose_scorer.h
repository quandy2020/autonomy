#ifndef OPT_NAVIGATION_POSE_SCORER_H_
#define OPT_NAVIGATION_POSE_SCORER_H_

#include "opt/navigation/goal_adapter.h"

namespace opt::navigation {
class PoseScorer {
 public:
  PoseScorer(SafetyOptions safety = {}, AdaptationOptions options = {});
  PoseScoreComponents Score(const CandidateEvaluation& candidate,
                            const AdaptationInput& input) const;

 private:
  SafetyOptions safety_;
  AdaptationOptions options_;
};
}  // namespace opt::navigation
#endif  // OPT_NAVIGATION_POSE_SCORER_H_
