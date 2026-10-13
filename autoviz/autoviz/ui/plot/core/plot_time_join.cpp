/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/core/plot_time_join.hpp"

#include <algorithm>
#include <cmath>

namespace autoviz {
namespace plot {
namespace {

struct ByX {
  bool operator()(const PlotPoint& sample, double x) const {
    return sample.x < x;
  }
  bool operator()(double x, const PlotPoint& sample) const {
    return x < sample.x;
  }
};

}  // namespace

std::optional<double> NearestYByTime(const std::deque<PlotPoint>& samples,
                                     double t, double max_dt) {
  if (samples.empty() || max_dt < 0.0) {
    return std::nullopt;
  }
  const auto it = std::lower_bound(samples.begin(), samples.end(), t, ByX{});
  const PlotPoint* best = nullptr;
  double best_dt = max_dt;
  if (it != samples.end()) {
    const double dt = std::abs(it->x - t);
    if (dt <= best_dt) {
      best_dt = dt;
      best = &(*it);
    }
  }
  if (it != samples.begin()) {
    const auto prev = std::prev(it);
    const double dt = std::abs(prev->x - t);
    if (dt <= best_dt) {
      best = &(*prev);
    }
  }
  if (best == nullptr) {
    return std::nullopt;
  }
  return best->y;
}

std::pair<std::size_t, std::size_t> VisibleIndexRange(
    const std::deque<PlotPoint>& samples, double min_x, double max_x) {
  if (samples.empty() || max_x < min_x) {
    return {0, 0};
  }
  const auto begin_it =
      std::lower_bound(samples.begin(), samples.end(), min_x, ByX{});
  const auto end_it =
      std::upper_bound(samples.begin(), samples.end(), max_x, ByX{});
  return {static_cast<std::size_t>(begin_it - samples.begin()),
          static_cast<std::size_t>(end_it - samples.begin())};
}

bool VisibleYRange(const std::deque<PlotPoint>& samples, double min_x,
                   double max_x, double* min_y, double* max_y) {
  if (min_y == nullptr || max_y == nullptr || samples.empty()) {
    return false;
  }
  // Fast path when X is non-decreasing (timestamp / index streams).
  bool monotonic = true;
  for (std::size_t i = 1; i < samples.size(); ++i) {
    if (samples[i].x + 1e-15 < samples[i - 1].x) {
      monotonic = false;
      break;
    }
  }
  bool any = false;
  double lo = 0.0;
  double hi = 0.0;
  if (monotonic) {
    const auto [begin, end] = VisibleIndexRange(samples, min_x, max_x);
    for (std::size_t i = begin; i < end; ++i) {
      if (!any) {
        lo = hi = samples[i].y;
        any = true;
      } else {
        lo = std::min(lo, samples[i].y);
        hi = std::max(hi, samples[i].y);
      }
    }
  } else {
    for (const PlotPoint& point : samples) {
      if (point.x < min_x || point.x > max_x) {
        continue;
      }
      if (!any) {
        lo = hi = point.y;
        any = true;
      } else {
        lo = std::min(lo, point.y);
        hi = std::max(hi, point.y);
      }
    }
  }
  if (!any) {
    return false;
  }
  *min_y = lo;
  *max_y = hi;
  return true;
}

}  // namespace plot
}  // namespace autoviz
