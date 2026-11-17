/**
 * @file goal_confirmation.cc
 * @brief Internal Opt goal confirmation implementation reused by native navigation.
 * @details Keeps the geometry implementation independent of ROS and map ownership.
 */
#include "opt/navigation/goal_confirmation.h"

#include <algorithm>
#include <stdexcept>

namespace opt::navigation {
namespace {
double DistanceToCell(const Pose2d& pose, const Pose2d& center, double half) {
  return std::hypot(std::max(0.0, std::abs(pose.x - center.x) - half),
                    std::max(0.0, std::abs(pose.y - center.y) - half));
}
}  // namespace

TargetStatus ClassifyTarget(const Pose2d& target, const GridMap& map,
                            const SafetyOptions& options) {
  const auto index = map.WorldToCell(target);
  if (!IsFinite(target) || !index) return TargetStatus::kUnknown;
  const double radius = options.robot_radius + options.horizontal_margin;
  bool unknown = false;
  double min_floor = std::numeric_limits<double>::infinity();
  double max_floor = -std::numeric_limits<double>::infinity();
  double min_ceiling = std::numeric_limits<double>::infinity();
  if (!std::isfinite(radius) || radius <= 0) return TargetStatus::kUnknown;
  const int reach = static_cast<int>(std::min(std::ceil(radius / map.resolution()),
      static_cast<double>(std::max(map.width(), map.height())))) + 1;
  for (int y = index->y - reach; y <= index->y + reach; ++y) {
    for (int x = index->x - reach; x <= index->x + reach; ++x) {
      const Pose2d cell_center{map.origin_x() + (x + .5) * map.resolution(),
                               map.origin_y() + (y + .5) * map.resolution(), 0};
      if (DistanceToCell(target, cell_center, map.resolution() / 2) > radius) continue;
      if (!map.Contains(x, y)) { unknown = true; continue; }
      const auto& cell = map.At(x, y);
      if (cell.state == CellState::kOccupied) return TargetStatus::kBlocked;
      if (cell.state == CellState::kUnknown || !std::isfinite(cell.floor_height) ||
          std::isnan(cell.ceiling_height) || cell.ceiling_height == -std::numeric_limits<double>::infinity()) {
        unknown = true; continue;
      }
      min_floor = std::min(min_floor, cell.floor_height);
      max_floor = std::max(max_floor, cell.floor_height);
      min_ceiling = std::min(min_ceiling, cell.ceiling_height);
      if (min_ceiling - max_floor < options.robot_height + options.vertical_margin ||
          max_floor - min_floor > options.max_floor_step + 1e-12) {
        return TargetStatus::kBlocked;
      }
    }
  }
  return unknown ? TargetStatus::kUnknown : TargetStatus::kFree;
}

ObservationGoalSelector::ObservationGoalSelector(SafetyOptions safety, ObservationOptions options)
    : safety_(safety), options_(options) {
  if (!std::isfinite(options.sensor_range) || options.sensor_range <= 0 ||
      !std::isfinite(options.min_motion) || options.min_motion <= 0 ||
      options.max_candidates < 1 || options.max_sample_cells < 1)
    throw std::invalid_argument("Invalid observation options");
}

bool ObservationGoalSelector::ObservationOpportunity(const Pose2d& pose, const Pose2d& target,
                                                     const GridMap& map) const {
  const double distance = Distance(pose, target);
  if (!std::isfinite(distance) || distance < options_.min_motion) return false;
  const double limit = std::min(distance, options_.sensor_range);
  const double segments = std::ceil(limit / (map.resolution() / 2));
  if (!std::isfinite(segments) || segments > 1000000) return false;
  const int steps = static_cast<int>(segments);
  for (int i = 1; i <= steps; ++i) {
    const double along = limit * i / steps;
    Pose2d sample{pose.x + (target.x - pose.x) * along / distance,
                  pose.y + (target.y - pose.y) * along / distance, 0};
    const auto cell = map.WorldToCell(sample);
    if (!cell || map.At(cell->x, cell->y).state == CellState::kUnknown) return true;
    if (map.At(cell->x, cell->y).state == CellState::kOccupied) return false;
  }
  // Allows reobservation of a stale target in otherwise known geometry.
  return distance <= options_.sensor_range;
}

AdaptationResult ObservationGoalSelector::Select(const AdaptationInput& input,
                                                const std::vector<Pose2d>& suggestions) const {
  AdaptationResult result;
  if (!input.map || !IsFinite(input.robot_pose) || !IsFinite(input.goal.original) ||
      input.map->frame_id() != input.goal.frame_id) {
    result.reason = "invalid observation selection input"; return result;
  }
  auto safety = input.safety;
  if (safety) {
    const auto& configured = safety->options();
    if (&safety->map() != input.map.get() || configured.robot_radius != safety_.robot_radius ||
        configured.horizontal_margin != safety_.horizontal_margin ||
        configured.robot_height != safety_.robot_height ||
        configured.vertical_margin != safety_.vertical_margin ||
        configured.max_floor_step != safety_.max_floor_step) {
      result.reason = "observation safety snapshot mismatch";
      return result;
    }
  } else {
    safety = std::make_shared<GeometrySafety>(input.map, safety_);
  }
  std::unique_ptr<ReachabilityField> reachable;
  std::vector<Pose2d> samples;
  if (!suggestions.empty()) {
    const auto count = std::min(suggestions.size(), options_.max_sample_cells);
    samples.assign(suggestions.begin(), suggestions.begin() + count);
    result.sampling_budget_exhausted = count < suggestions.size();
  }
  else {
    // Uniform coverage instead of allocating/sorting every cell on a large map.
    const auto stride = 1 + (input.map->size() - 1) / options_.max_sample_cells;
    result.sampling_budget_exhausted = stride > 1;
    for (std::size_t index = 0; index < input.map->size(); index += stride) {
      const auto cell = input.map->FromIndex(index);
      if (input.map->At(cell.x, cell.y).state == CellState::kFree)
        samples.push_back(input.map->CellCenter(cell.x, cell.y));
    }
  }
  samples.erase(std::remove_if(samples.begin(), samples.end(), [&](const Pose2d& pose) {
    return !IsFinite(pose) || !ObservationOpportunity(pose, input.goal.original, *input.map);
  }), samples.end());
  std::stable_sort(samples.begin(), samples.end(), [&](const Pose2d& a, const Pose2d& b) {
    return Distance(a, input.goal.original) < Distance(b, input.goal.original);
  });
  if (input.previous && input.previous->mode == GoalMode::kUnknownApproach &&
      ObservationOpportunity(input.previous->pose, input.goal.original, *input.map)) {
    samples.insert(samples.begin(), input.previous->pose);
  }
  for (auto pose : samples) {
    if (result.candidates.size() >= options_.max_candidates) {
      result.sampling_budget_exhausted = true;
      break;
    }
    if (Distance(pose, input.robot_pose) < options_.min_motion ||
        std::any_of(input.excluded.begin(), input.excluded.end(), [&](const Pose2d& visited) {
          return Distance(pose, visited) < options_.min_motion;
        })) continue;
    pose.yaw = std::atan2(input.goal.original.y - pose.y, input.goal.original.x - pose.x);
    CandidateEvaluation candidate;
    candidate.pose = pose; candidate.mode = GoalMode::kUnknownApproach;
    candidate.safety = safety->VerifyPose(pose);
    candidate.path_length = std::numeric_limits<double>::infinity();
    if (candidate.safety.safe) {
      if (!reachable) {
        reachable = std::make_unique<ReachabilityField>(
            safety, input.robot_pose, options_.max_reachability_expansions);
        result.reachability_expansions = reachable->expansions();
        result.reachability_budget_exhausted = reachable->budget_exhausted();
      }
      candidate.path_length = reachable->DistanceTo(pose);
    }
    candidate.accepted = candidate.safety.safe && std::isfinite(candidate.path_length);
    candidate.reason = !candidate.safety.safe ? candidate.safety.reason :
        candidate.accepted ? "safe reachable observation pose"
                           : result.reachability_budget_exhausted
                               ? "observation route unconfirmed within expansion budget"
                               : "observation route blocked";
    if (candidate.accepted) {
      candidate.score = 1 / (1 + Distance(pose, input.goal.original)) +
          .15 / (1 + candidate.path_length);
      if (!result.selected || candidate.score > result.selected->score) {
        result.selected = AdaptedGoal{pose, candidate.mode, "observation", candidate.safety.clearance,
                                      candidate.path_length, candidate.score};
      }
    }
    result.candidates.push_back(candidate);
  }
  // Hold a still-safe previous staging pose while in transit, avoiding retarget jitter.
  if (input.previous && input.previous->mode == GoalMode::kUnknownApproach) {
    for (const auto& c : result.candidates) if (c.accepted && Distance(c.pose, input.previous->pose) < 1e-9) {
      result.selected = AdaptedGoal{c.pose, c.mode, "observation", c.safety.clearance, c.path_length, c.score};
      break;
    }
  }
  result.reason = result.selected ? "selected known-space observation pose" :
      (result.reachability_budget_exhausted || result.sampling_budget_exhausted)
          ? "observation search budget exhausted; target unconfirmed"
          : "no safe reachable observation pose; target unconfirmed";
  return result;
}
}  // namespace opt::navigation
