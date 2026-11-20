/**
 * @file goal_adapter.cc
 * @brief Internal Opt goal adapter implementation reused by native navigation.
 * @details Keeps the geometry implementation independent of ROS and map ownership.
 */
#include "autonomy/common/navigation/goal_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "autonomy/common/navigation/pose_sampler.hpp"
#include "autonomy/common/navigation/pose_scorer.hpp"

namespace autonomy::navigation {

std::optional<GoalRegion> GeometricRegionProvider::Resolve(const GoalRequest& goal,
                                                          const GridMap&) const {
  if (goal.region) return goal.region;
  return GoalRegion{"geometric_goal_neighborhood", goal.original, radius_};
}

GoalAdapter::GoalAdapter(SafetyOptions safety, AdaptationOptions adaptation,
                         std::shared_ptr<const IGoalRegionProvider> regions)
    : safety_options_(safety), options_(adaptation), regions_(std::move(regions)) {
  const double values[] = {options_.ring_min_radius, options_.ring_max_radius,
      options_.fallback_radius, options_.preferred_clearance, options_.goal_weight,
      options_.clearance_weight, options_.path_weight, options_.continuity_weight,
      options_.min_score_improvement, options_.switch_distance, options_.switch_yaw,
      options_.yaw_continuity_scale, options_.heading_weight, options_.region_sample_spacing,
      options_.refinement_radius};
  for (double value : values) {
    if (!std::isfinite(value) || value < 0.0) throw std::invalid_argument("Invalid adaptation option");
  }
  const double weight = options_.goal_weight + options_.clearance_weight + options_.path_weight +
                        options_.continuity_weight + options_.heading_weight;
  if (options_.ring_max_radius < options_.ring_min_radius || options_.ring_count < 1 ||
      options_.angle_count < 4 || options_.max_candidates < 2 ||
      options_.fallback_radius <= 0.0 || options_.preferred_clearance <= 0.0 ||
      !std::isfinite(weight) || weight <= 0.0 ||
      options_.region_sample_spacing <= 0 || options_.refinement_levels < 0 ||
      options_.refinement_levels > 8 || options_.refinement_seeds < 0 ||
      options_.refinement_seeds > 32) {
    throw std::invalid_argument("Invalid candidate or scoring configuration");
  }
  if (!regions_) regions_ = std::make_shared<GeometricRegionProvider>(options_.fallback_radius);
}

AdaptationResult GoalAdapter::Adapt(const AdaptationInput& input) const {
  AdaptationResult result;
  if (input.cancel_checker && input.cancel_checker()) {
    result.reason = "goal adaptation canceled";
    return result;
  }
  if (!input.map || !IsFinite(input.robot_pose) || !IsFinite(input.goal.original) ||
      input.goal.frame_id != input.map->frame_id() || !IsValidGoalOrientation(input.goal)) {
    result.reason = "missing map, invalid pose, or frame mismatch";
    return result;
  }
  if (input.previous && !IsFinite(input.previous->pose)) {
    result.reason = "invalid previous goal";
    return result;
  }
  if (std::any_of(input.excluded.begin(), input.excluded.end(),
                  [](const Pose2d& pose) { return !IsFinite(pose); })) {
    result.reason = "invalid excluded goal pose";
    return result;
  }
  if (input.exact_only && input.region_only) {
    result.reason = "conflicting exact-only and region-only search";
    return result;
  }
  auto safety = input.safety;
  if (safety) {
    const auto& configured = safety->options();
    if (&safety->map() != input.map.get() ||
        configured.robot_radius != safety_options_.robot_radius ||
        configured.horizontal_margin != safety_options_.horizontal_margin ||
        configured.robot_height != safety_options_.robot_height ||
        configured.vertical_margin != safety_options_.vertical_margin ||
        configured.max_floor_step != safety_options_.max_floor_step) {
      result.reason = "safety snapshot does not match map or robot dimensions";
      return result;
    }
  } else {
    safety = std::make_shared<GeometrySafety>(input.map, safety_options_);
  }
  // Reject unsafe candidates before constructing a reachable field. The field
  // is built at most once and serves every accepted candidate in this search.
  std::unique_ptr<ReachabilityField> reachable;
  const PoseSampler sampler(options_);
  const PoseScorer scorer(safety_options_, options_);
  const bool refine = input.enable_refinement && options_.refinement_levels > 0 &&
                      options_.refinement_seeds > 0 && options_.refinement_radius > 0;
  auto mode_allowed = [&](GoalMode mode) {
    return !input.preserve_mode || !input.previous || input.previous->mode == mode;
  };
  auto evaluate = [&](const Pose2d& pose, GoalMode mode,
                      std::size_t limit) -> std::optional<CandidateEvaluation> {
    if (input.cancel_checker && input.cancel_checker()) return std::nullopt;
    for (const auto& known : result.candidates) {
      if (known.mode == mode && Distance(pose, known.pose) < 1e-9 &&
          std::abs(NormalizeAngle(pose.yaw - known.pose.yaw)) < 1e-9) return known;
    }
    if (result.candidates.size() >= limit) {
      result.sampling_budget_exhausted = true;
      return std::nullopt;
    }
    if (!IsFinite(pose)) return std::nullopt;
    CandidateEvaluation candidate;
    candidate.pose = pose;
    candidate.mode = mode;
    candidate.safety = safety->VerifyPose(pose);
    candidate.path_length = std::numeric_limits<double>::infinity();
    if (std::any_of(input.excluded.begin(), input.excluded.end(), [&](const Pose2d& rejected) {
      return Distance(pose, rejected) < std::max(options_.switch_distance, 1e-9);
    })) candidate.reason = "temporarily excluded after planning failure";
    else if (!candidate.safety.safe) candidate.reason = candidate.safety.reason;
    else {
      if (!reachable) {
        reachable = std::make_unique<ReachabilityField>(
            safety, input.robot_pose, options_.max_reachability_expansions, input.cancel_checker);
        result.reachability_budget_exhausted = reachable->budget_exhausted();
        result.reachability_expansions = reachable->expansions();
      }
      candidate.path_length = reachable->DistanceTo(pose);
      candidate.accepted = std::isfinite(candidate.path_length);
      candidate.reason = candidate.accepted ? "safe and reachable"
                           : result.reachability_budget_exhausted
                               ? "reachability not established within expansion budget"
                               : "unreachable";
      if (candidate.accepted) {
        candidate.score_components = scorer.Score(candidate, input);
        candidate.score = candidate.score_components.total;
      }
    }
    result.candidates.push_back(candidate);
    return candidate;
  };
  auto select = [&](const CandidateEvaluation& candidate, const std::string& region_id) {
    if (input.cancel_checker && input.cancel_checker()) {
      result.reason = "goal adaptation canceled";
      return;
    }
    result.selected = AdaptedGoal{candidate.pose, candidate.mode, region_id,
        candidate.safety.clearance, candidate.path_length, candidate.score};
    result.reason = "selected " + std::string(ToString(candidate.mode));
  };

  // A valid original goal always wins; scoring never displaces it.
  if (!input.region_only && (mode_allowed(GoalMode::kExact) || input.exact_only)) {
    const auto exact = evaluate(input.goal.original, GoalMode::kExact, options_.max_candidates);
    if (exact && exact->accepted) { select(*exact, ""); return result; }
    if (exact && exact->safety.safe && result.reachability_budget_exhausted) {
      result.reason = "original goal reachability unconfirmed within expansion budget";
      return result;
    }
  }
  if (input.exact_only) {
    result.reason = result.reachability_budget_exhausted
                        ? "original goal reachability unconfirmed within expansion budget"
                        : "original goal unavailable";
    return result;
  }

  auto choose_best = [&](GoalMode mode, const std::string& region_id,
                         const std::optional<GoalRegion>& region, std::size_t limit) {
    // Refine only safe, reachable seeds. Each level samples a smaller neighborhood;
    // all new poses still pass the same footprint, overhead and route hard checks.
    for (int level = 0; refine && level < options_.refinement_levels &&
                        result.candidates.size() < limit; ++level) {
      std::vector<CandidateEvaluation> seeds;
      for (const auto& candidate : result.candidates) {
        if (candidate.accepted && candidate.mode == mode) seeds.push_back(candidate);
      }
      std::stable_sort(seeds.begin(), seeds.end(), [](const auto& a, const auto& b) {
        return a.score > b.score;
      });
      if (seeds.size() > static_cast<std::size_t>(options_.refinement_seeds))
        seeds.resize(options_.refinement_seeds);
      const double radius = std::ldexp(options_.refinement_radius, -level);
      for (const auto& seed : seeds) {
        for (const auto& pose : sampler.Refine(input, seed.pose, radius,
                                               limit - result.candidates.size())) {
          if (mode == GoalMode::kApproximate &&
              Distance(pose, input.goal.original) > options_.ring_max_radius + 1e-9) continue;
          if (region && Distance(pose, region->center) > region->radius + 1e-9) continue;
          evaluate(pose, mode, limit);
        }
      }
    }
    std::optional<CandidateEvaluation> best;
    std::optional<CandidateEvaluation> previous;
    for (const auto& candidate : result.candidates) {
      if (!candidate.accepted || candidate.mode != mode) continue;
      if (!best || candidate.score > best->score) best = candidate;
      if (input.previous && Distance(candidate.pose, input.previous->pose) < 1e-9 &&
          std::abs(NormalizeAngle(candidate.pose.yaw - input.previous->pose.yaw)) < 1e-9)
        previous = candidate;
    }
    if (!best) return false;
    if (previous && ((Distance(best->pose, previous->pose) < options_.switch_distance &&
                      std::abs(NormalizeAngle(best->pose.yaw - previous->pose.yaw)) < options_.switch_yaw) ||
                     best->score < previous->score + options_.min_score_improvement)) {
      best = previous;
      result.retained_previous = true;
    }
    select(*best, region_id);
    return true;
  };

  auto add_previous = [&](GoalMode mode, std::size_t limit) {
    if (!input.previous || input.previous->mode != mode) return;
    const auto oriented = sampler.Orient(input.previous->pose, input);
    // A small heading change may be held by hysteresis. A large change must
    // respect the current orientation policy rather than retaining an old yaw.
    if (std::abs(NormalizeAngle(oriented.yaw - input.previous->pose.yaw)) <= options_.switch_yaw)
      evaluate(input.previous->pose, mode, limit);
    evaluate(oriented, mode, limit);
  };
  auto has_accepted = [&](GoalMode mode) {
    return std::any_of(result.candidates.begin(), result.candidates.end(), [&](const auto& c) {
      return c.accepted && c.mode == mode;
    });
  };
  auto coarse_limit = [&](std::size_t limit) {
    const auto remaining = limit - result.candidates.size();
    return limit - (refine ? remaining / 3 : 0);
  };

  if (!input.region_only && input.goal.allow_approximate && mode_allowed(GoalMode::kApproximate)) {
    // Reserve budget for the region stage if point-level candidates all fail.
    // Fine sampling and previous-pose checks share the same hard stage limit.
    const bool can_use_region = input.goal.allow_region_fallback &&
                               mode_allowed(GoalMode::kRegionFallback);
    const auto remaining = options_.max_candidates - result.candidates.size();
    const auto limit = result.candidates.size() +
                       (can_use_region ? remaining / 2 : remaining);
    const auto coarse = coarse_limit(limit);
    if (input.previous && input.previous->mode == GoalMode::kApproximate &&
        Distance(input.previous->pose, input.goal.original) <= options_.ring_max_radius + 1e-9) {
      add_previous(GoalMode::kApproximate, coarse);
    }
    for (const auto& pose : sampler.SampleRing(input, coarse - result.candidates.size()))
      evaluate(pose, GoalMode::kApproximate, coarse);
    if (!has_accepted(GoalMode::kApproximate)) {
      // Without an accepted seed, use the unused fine budget for wider coverage.
      for (const auto& pose : sampler.SampleRing(input, limit))
        evaluate(pose, GoalMode::kApproximate, limit);
    }
    if (choose_best(GoalMode::kApproximate, "", std::nullopt, limit)) return result;
  }

  if (input.goal.allow_region_fallback && mode_allowed(GoalMode::kRegionFallback)) {
    const auto region = regions_->Resolve(input.goal, *input.map);
    if (!region || region->id.empty() || !IsFinite(region->center) ||
        !std::isfinite(region->radius) || region->radius <= 0.0) {
      result.reason = "no valid grounded goal region";
      return result;
    }
    const auto limit = options_.max_candidates;
    const auto coarse = coarse_limit(limit);
    if (input.previous && input.previous->mode == GoalMode::kRegionFallback &&
        result.candidates.size() < options_.max_candidates &&
        input.previous->region_id == region->id && Distance(input.previous->pose, region->center) <= region->radius) {
      add_previous(GoalMode::kRegionFallback, coarse);
    }
    for (const auto& pose : sampler.SampleRegion(input, *region,
        coarse - result.candidates.size()))
      evaluate(pose, GoalMode::kRegionFallback, coarse);
    if (!has_accepted(GoalMode::kRegionFallback)) {
      for (const auto& pose : sampler.SampleRegion(input, *region, limit))
        evaluate(pose, GoalMode::kRegionFallback, limit);
    }
    if (choose_best(GoalMode::kRegionFallback, region->id, region, limit)) return result;
  }
  result.sampling_budget_exhausted = result.sampling_budget_exhausted ||
      result.candidates.size() >= options_.max_candidates;
  result.reason = result.reachability_budget_exhausted ?
      "reachability search budget exhausted; no reachable candidate confirmed" :
      result.candidates.size() >= options_.max_candidates ?
      "candidate budget exhausted; no accepted candidate" :
      "no safe reachable candidate found within the search budget and allowed regions";
  return result;
}

}  // namespace autonomy::navigation
