/*
 * Copyright 2026 The Openbot Authors
 *
 * Blended Path (linear segments + circular blends) for Kunz–Stilman.
 */

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "autonomy/manipulation/pipeline/kunz_circular_path_segment.hpp"
#include "autonomy/manipulation/pipeline/kunz_joint_configuration.hpp"
#include "autonomy/manipulation/pipeline/kunz_linear_path_segment.hpp"
#include "autonomy/manipulation/pipeline/kunz_path_segment.hpp"

namespace autonomy {
namespace manipulation {
namespace trajectory {
namespace kunz {

/**
 * @brief Blended path: linear segments + circular blends at waypoints.
 */
class Path {
 public:
  static Path Create(const std::vector<JointConfiguration>& waypoints,
                     double max_deviation) {
    Path path;
    if (waypoints.size() < 2 || max_deviation <= 0.0) {
      return path;
    }
    JointConfiguration start_config = waypoints.front();
    for (std::size_t i = 1; i + 1 < waypoints.size(); ++i) {
      const JointConfiguration& w1 = waypoints[i - 1];
      const JointConfiguration& w2 = waypoints[i];
      const JointConfiguration& w3 = waypoints[i + 1];
      auto blend = std::make_unique<CircularPathSegment>(
          LinearInterpolate(w1, w2, 0.5), w2, LinearInterpolate(w2, w3, 0.5), max_deviation);
      JointConfiguration end_config = blend->GetConfiguration(0.0);
      if (EuclideanNorm(Subtract(end_config, start_config)) > 1e-6) {
        path.segments_.push_back(
            std::make_unique<LinearPathSegment>(start_config, end_config));
      }
      start_config = blend->GetConfiguration(blend->GetLength());
      path.segments_.push_back(std::move(blend));
    }
    path.segments_.push_back(std::make_unique<LinearPathSegment>(
        start_config, waypoints.back()));

    for (auto& seg : path.segments_) {
      seg->position = path.length_;
      for (double sp : seg->GetSwitchingPoints()) {
        path.switching_.emplace_back(path.length_ + sp, false);
      }
      path.length_ += seg->GetLength();
      while (!path.switching_.empty() &&
             path.switching_.back().first >= path.length_) {
        path.switching_.pop_back();
      }
      path.switching_.emplace_back(path.length_, true);
    }
    if (!path.switching_.empty()) {
      path.switching_.pop_back();
    }
    return path;
  }

  double GetLength() const { return length_; }

  PathSegment* GetPathSegment(double* s) const {
    if (segments_.empty()) {
      return nullptr;
    }
    for (std::size_t i = 0; i + 1 < segments_.size(); ++i) {
      if (*s < segments_[i + 1]->position) {
        *s -= segments_[i]->position;
        return segments_[i].get();
      }
    }
    *s -= segments_.back()->position;
    return segments_.back().get();
  }

  JointConfiguration GetConfiguration(double s) const {
    PathSegment* seg = GetPathSegment(&s);
    return seg ? seg->GetConfiguration(s) : JointConfiguration{};
  }
  JointConfiguration GetTangent(double s) const {
    PathSegment* seg = GetPathSegment(&s);
    return seg ? seg->GetTangent(s) : JointConfiguration{};
  }
  JointConfiguration GetCurvature(double s) const {
    PathSegment* seg = GetPathSegment(&s);
    return seg ? seg->GetCurvature(s) : JointConfiguration{};
  }

  const std::vector<std::pair<double, bool>>& SwitchingPoints() const {
    return switching_;
  }

 private:
  double length_ = 0.0;
  std::vector<std::unique_ptr<PathSegment>> segments_;
  std::vector<std::pair<double, bool>> switching_;
};

}  // namespace kunz
}  // namespace trajectory
}  // namespace manipulation
}  // namespace autonomy
