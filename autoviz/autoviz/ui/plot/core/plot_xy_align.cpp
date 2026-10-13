/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/core/plot_xy_align.hpp"

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

const PlotPoint* NearestTimed(const std::deque<PlotPoint>& samples, double t,
                              double max_dt) {
  if (samples.empty() || max_dt < 0.0) {
    return nullptr;
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
  return best;
}

}  // namespace

void PushTimedSample(std::deque<PlotPoint>& buffer, double timestamp_sec,
                     double value, int max_points) {
  PlotPoint sample;
  sample.x = timestamp_sec;
  sample.y = value;
  sample.t = timestamp_sec;
  buffer.push_back(sample);
  while (static_cast<int>(buffer.size()) > max_points) {
    buffer.pop_front();
  }
}

std::vector<PlotPoint> AlignXyNearest(std::deque<PlotPoint>& x_samples,
                                      std::deque<PlotPoint>& y_samples,
                                      double max_dt_sec) {
  std::vector<PlotPoint> out;
  if (x_samples.empty() || y_samples.empty() || max_dt_sec < 0.0) {
    return out;
  }

  while (!y_samples.empty()) {
    const PlotPoint y = y_samples.front();
    const PlotPoint* best = NearestTimed(x_samples, y.x, max_dt_sec);
    if (best == nullptr) {
      break;
    }
    while (!x_samples.empty() && x_samples.front().x < best->x - 1e-15) {
      x_samples.pop_front();
    }
    PlotPoint point;
    point.x = best->y;
    point.y = y.y;
    point.t = y.x;
    out.push_back(point);
    y_samples.pop_front();
  }
  return out;
}

std::optional<PlotPoint> SampleXyFromTime(const std::deque<PlotPoint>& points,
                                          double time_sec) {
  if (points.empty()) {
    return std::nullopt;
  }
  // points are ordered by append time; for XY, t is sample time.
  const PlotPoint* best = nullptr;
  for (const PlotPoint& point : points) {
    if (point.t <= time_sec) {
      best = &point;
    } else {
      break;
    }
  }
  if (best == nullptr) {
    return std::nullopt;
  }
  return *best;
}

}  // namespace plot
}  // namespace autoviz
