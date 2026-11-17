/**
 * @file pose_scorer.cc
 * @brief Internal Opt pose scorer implementation reused by native navigation.
 * @details Keeps the geometry implementation independent of ROS and map ownership.
 */
#include "opt/navigation/pose_scorer.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace opt::navigation {
PoseScorer::PoseScorer(SafetyOptions safety, AdaptationOptions options)
    : safety_(safety), options_(options) {
  for (double value : {options_.preferred_clearance, options_.goal_weight,
                       options_.clearance_weight, options_.path_weight, options_.continuity_weight,
                       options_.heading_weight, options_.yaw_continuity_scale,
                       safety_.robot_radius, safety_.horizontal_margin}) {
    if (!std::isfinite(value) || value < 0) throw std::invalid_argument("Invalid pose score option");
  }
  const double weight = options_.goal_weight + options_.clearance_weight + options_.path_weight +
                        options_.continuity_weight + options_.heading_weight;
  if (options_.preferred_clearance <= 0 || !std::isfinite(weight) || weight <= 0) {
    throw std::invalid_argument("Pose score requires positive scales and weights");
  }
}
PoseScoreComponents PoseScorer::Score(const CandidateEvaluation& candidate,
                                    const AdaptationInput& input) const {
  PoseScoreComponents score;
  if (!candidate.accepted || !candidate.safety.safe || !IsFinite(candidate.pose) ||
      !std::isfinite(candidate.path_length) || candidate.path_length < 0 ||
      std::isnan(candidate.safety.clearance) || !IsFinite(input.goal.original) ||
      !IsFinite(input.robot_pose) || (input.previous && !IsFinite(input.previous->pose))) return score;
  score.goal_proximity = 1 / (1 + Distance(candidate.pose, input.goal.original));
  score.clearance = std::clamp((candidate.safety.clearance - safety_.robot_radius -
                               safety_.horizontal_margin) / options_.preferred_clearance, 0.0, 1.0);
  score.path_efficiency = 1 / (1 + candidate.path_length);
  if (input.previous) {
    score.continuity = 1 / (1 + Distance(candidate.pose, input.previous->pose) +
                              options_.yaw_continuity_scale *
                                  std::abs(NormalizeAngle(candidate.pose.yaw - input.previous->pose.yaw)));
  }
  // Straight-line approach heading is a ranking heuristic, not a dynamics check.
  const double approach = Distance(candidate.pose, input.robot_pose) > 1e-9
                              ? std::atan2(candidate.pose.y - input.robot_pose.y,
                                           candidate.pose.x - input.robot_pose.x)
                              : input.robot_pose.yaw;
  score.heading_alignment = 1 / (1 + std::abs(NormalizeAngle(candidate.pose.yaw - approach)));
  const double weight = options_.goal_weight + options_.clearance_weight + options_.path_weight +
                        options_.continuity_weight + options_.heading_weight;
  score.total = (options_.goal_weight * score.goal_proximity +
                 options_.clearance_weight * score.clearance +
                 options_.path_weight * score.path_efficiency +
                 options_.continuity_weight * score.continuity +
                 options_.heading_weight * score.heading_alignment) / weight;
  return score;
}
}  // namespace opt::navigation
