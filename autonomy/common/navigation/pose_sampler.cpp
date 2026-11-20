/**
 * @file pose_sampler.cc
 * @brief Internal Opt pose sampler implementation reused by native navigation.
 * @details Keeps the geometry implementation independent of ROS and map ownership.
 */
#include "autonomy/common/navigation/pose_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace autonomy::navigation {
PoseSampler::PoseSampler(AdaptationOptions options) : options_(options) {
  if (!std::isfinite(options_.ring_min_radius) || options_.ring_min_radius < 0 ||
      !std::isfinite(options_.ring_max_radius) ||
      options_.ring_max_radius < options_.ring_min_radius || options_.ring_count < 1 ||
      options_.angle_count < 4 || !std::isfinite(options_.region_sample_spacing) ||
      options_.region_sample_spacing <= 0) {
    throw std::invalid_argument("Invalid pose sampling options");
  }
}
Pose2d PoseSampler::Orient(Pose2d pose, const AdaptationInput& input) const {
  if (!IsFinite(pose) || !IsFinite(input.robot_pose) || !IsFinite(input.goal.original) ||
      !IsValidGoalOrientation(input.goal)) {
    throw std::invalid_argument("Invalid pose or orientation request");
  }
  const auto bearing = [&](const Pose2d& target) {
    return Distance(pose, target) > 1e-9 ? std::atan2(target.y - pose.y, target.x - pose.x)
                                         : input.goal.original.yaw;
  };
  switch (input.goal.orientation_policy) {
    case GoalOrientationPolicy::kPreserveRequested:
      pose.yaw = input.goal.original.yaw;
      break;
    case GoalOrientationPolicy::kFaceOriginalGoal:
      pose.yaw = bearing(input.goal.original);
      break;
    case GoalOrientationPolicy::kFaceReferencePoint:
      if (!input.goal.orientation_reference || !IsFinite(*input.goal.orientation_reference)) {
        throw std::invalid_argument("Facing a reference point requires a finite reference pose");
      }
      pose.yaw = bearing(*input.goal.orientation_reference);
      break;
    case GoalOrientationPolicy::kAlignApproach:
      pose.yaw = Distance(pose, input.robot_pose) > 1e-9
                     ? std::atan2(pose.y - input.robot_pose.y, pose.x - input.robot_pose.x)
                     : input.robot_pose.yaw;
      break;
    default:
      throw std::invalid_argument("Invalid goal orientation policy");
  }
  pose.yaw = NormalizeAngle(pose.yaw);
  return pose;
}
std::vector<Pose2d> PoseSampler::SampleRing(const AdaptationInput& input,
                                          std::size_t budget) const {
  std::vector<Pose2d> samples;
  // Interleave radii before filling angular detail.
  // A small budget therefore cannot be spent only on the innermost blocked ring.
  const int rings = static_cast<int>(std::min<std::size_t>(options_.ring_count, budget));
  if (rings == 0) return samples;
  const std::size_t angles = std::min<std::size_t>(options_.angle_count, budget / rings);
  for (std::size_t angle = 0; angle < angles; ++angle) {
    const double theta = 2 * kPi * angle / angles;
    for (int ring = 0; ring < rings; ++ring) {
      const double u = rings == 1 ? 1 : static_cast<double>(ring) / (rings - 1);
      const double radius = options_.ring_min_radius +
                            u * (options_.ring_max_radius - options_.ring_min_radius);
      const Pose2d pose{input.goal.original.x + radius * std::cos(theta),
                        input.goal.original.y + radius * std::sin(theta), 0};
      if (IsFinite(pose)) samples.push_back(Orient(pose, input));
    }
  }
  return samples;
}
std::vector<Pose2d> PoseSampler::SampleRegion(const AdaptationInput& input,
                                            const GoalRegion& region,
                                            std::size_t budget) const {
  std::vector<Pose2d> samples;
  if (!input.map || budget == 0 || !IsFinite(region.center) ||
      !std::isfinite(region.radius) || region.radius <= 0) return samples;
  const auto& map = *input.map;
  const auto coordinate = [](double value, double origin, double resolution, int limit) {
    return static_cast<int>(std::clamp(std::floor((value - origin) / resolution),
                                       0.0, static_cast<double>(limit - 1)));
  };
  const int min_x = coordinate(region.center.x - region.radius, map.origin_x(),
                               map.resolution(), map.width());
  const int max_x = coordinate(region.center.x + region.radius, map.origin_x(),
                               map.resolution(), map.width());
  const int min_y = coordinate(region.center.y - region.radius, map.origin_y(),
                               map.resolution(), map.height());
  const int max_y = coordinate(region.center.y + region.radius, map.origin_y(),
                               map.resolution(), map.height());
  const int width = max_x - min_x + 1, height = max_y - min_y + 1;
  const int extent = std::max(width, height);
  int stride = static_cast<int>(std::clamp(std::ceil(options_.region_sample_spacing /
                                                   map.resolution()), 1.0,
                                           static_cast<double>(extent)));
  auto sample_count = [&](int step) {
    return static_cast<std::size_t>((width - 1) / step + 1) * ((height - 1) / step + 1);
  };
  if (sample_count(stride) > budget) {
    int low = stride, high = extent;
    while (low < high) {
      const int middle = low + (high - low) / 2;
      if (sample_count(middle) > budget) low = middle + 1;
      else high = middle;
    }
    stride = low;
  }
  // Cover the bounding box uniformly instead of scanning only its nearest cells.
  // Known occupied cells are skipped; footprint/height checks remain downstream.
  for (std::int64_t y = min_y; y <= max_y; y += stride) {
    for (std::int64_t x = min_x; x <= max_x; x += stride) {
      const auto pose = map.CellCenter(static_cast<int>(x), static_cast<int>(y));
      if (Distance(pose, region.center) > region.radius ||
          map.At(static_cast<int>(x), static_cast<int>(y)).state != CellState::kFree) continue;
      samples.push_back(Orient(pose, input));
    }
  }
  std::stable_sort(samples.begin(), samples.end(), [&](const auto& a, const auto& b) {
    return Distance(a, input.goal.original) < Distance(b, input.goal.original);
  });
  return samples;
}
std::vector<Pose2d> PoseSampler::Refine(const AdaptationInput& input, const Pose2d& seed,
                                      double radius, std::size_t budget) const {
  std::vector<Pose2d> samples;
  if (!IsFinite(seed) || !std::isfinite(radius) || radius <= 0) return samples;
  const auto count = std::min<std::size_t>(options_.angle_count, budget);
  for (std::size_t i = 0; i < count; ++i) {
    const double angle = 2 * kPi * i / count;
    const Pose2d pose{seed.x + radius * std::cos(angle),
                      seed.y + radius * std::sin(angle), 0};
    if (IsFinite(pose)) samples.push_back(Orient(pose, input));
  }
  return samples;
}
}  // namespace autonomy::navigation
