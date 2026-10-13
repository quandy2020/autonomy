/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/core/plot_sample_reducer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>

namespace autoviz {
namespace plot {
namespace {

struct FiniteRun {
  std::size_t begin = 0;
  std::size_t end = 0;

  [[nodiscard]] std::size_t size() const noexcept { return end - begin; }

  [[nodiscard]] std::size_t endpointCost() const noexcept {
    return size() == 1 ? 1 : 2;
  }

  [[nodiscard]] std::size_t interiorSize() const noexcept {
    return size() > 2 ? size() - 2 : 0;
  }
};

[[nodiscard]] bool FinitePoint(const QPointF& point) {
  return std::isfinite(point.x()) && std::isfinite(point.y());
}

void AppendRunEndpoints(PlotSampleReduction& result, const FiniteRun& run,
                        const PlotSampleAccessor& sample_at) {
  result.samples.push_back({sample_at(run.begin), run.begin, true});
  if (run.size() > 1) {
    result.samples.push_back(
        {sample_at(run.end - 1), run.end - 1, false});
  }
  ++result.retained_runs;
}

[[nodiscard]] std::vector<std::size_t> AllocateBuckets(
    const std::vector<FiniteRun>& runs, std::size_t bucket_budget) {
  std::vector<std::size_t> allocation(runs.size(), 0);
  const std::size_t total_interior = std::accumulate(
      runs.begin(), runs.end(), std::size_t{0},
      [](std::size_t total, const FiniteRun& run) {
        return total + run.interiorSize();
      });
  if (bucket_budget == 0 || total_interior == 0) {
    return allocation;
  }

  struct Remainder {
    std::size_t run = 0;
    std::uint64_t numerator = 0;
  };
  std::vector<Remainder> remainders;
  remainders.reserve(runs.size());
  std::size_t assigned = 0;
  for (std::size_t index = 0; index < runs.size(); ++index) {
    const std::size_t interior = runs[index].interiorSize();
    const std::uint64_t numerator =
        static_cast<std::uint64_t>(bucket_budget) *
        static_cast<std::uint64_t>(interior);
    allocation[index] = std::min(
        interior,
        static_cast<std::size_t>(numerator /
                                 static_cast<std::uint64_t>(total_interior)));
    assigned += allocation[index];
    remainders.push_back(
        {index, numerator % static_cast<std::uint64_t>(total_interior)});
  }
  std::stable_sort(remainders.begin(), remainders.end(),
                   [](const Remainder& lhs, const Remainder& rhs) {
                     return lhs.numerator > rhs.numerator;
                   });
  for (const Remainder& remainder : remainders) {
    if (assigned >= bucket_budget) {
      break;
    }
    if (allocation[remainder.run] < runs[remainder.run].interiorSize()) {
      ++allocation[remainder.run];
      ++assigned;
    }
  }
  return allocation;
}

void AppendReducedRun(PlotSampleReduction& result, const FiniteRun& run,
                      std::size_t bucket_count,
                      const PlotSampleAccessor& sample_at) {
  std::vector<std::size_t> selected;
  selected.reserve(2 + bucket_count * 4);
  selected.push_back(run.begin);

  const std::size_t interior_begin = run.begin + 1;
  const std::size_t interior_size = run.interiorSize();
  for (std::size_t bucket = 0; bucket < bucket_count; ++bucket) {
    const std::size_t begin =
        interior_begin +
        static_cast<std::size_t>(
            (static_cast<std::uint64_t>(interior_size) * bucket) /
            static_cast<std::uint64_t>(bucket_count));
    const std::size_t end =
        interior_begin +
        static_cast<std::size_t>(
            (static_cast<std::uint64_t>(interior_size) * (bucket + 1)) /
            static_cast<std::uint64_t>(bucket_count));
    if (begin >= end) {
      continue;
    }
    std::size_t min_x = begin;
    std::size_t max_x = begin;
    std::size_t min_y = begin;
    std::size_t max_y = begin;
    QPointF min_x_point = sample_at(begin);
    QPointF max_x_point = min_x_point;
    QPointF min_y_point = min_x_point;
    QPointF max_y_point = min_x_point;
    for (std::size_t index = begin + 1; index < end; ++index) {
      const QPointF point = sample_at(index);
      if (point.x() < min_x_point.x()) {
        min_x = index;
        min_x_point = point;
      }
      if (point.x() > max_x_point.x()) {
        max_x = index;
        max_x_point = point;
      }
      if (point.y() < min_y_point.y()) {
        min_y = index;
        min_y_point = point;
      }
      if (point.y() > max_y_point.y()) {
        max_y = index;
        max_y_point = point;
      }
    }
    selected.push_back(min_x);
    selected.push_back(max_x);
    selected.push_back(min_y);
    selected.push_back(max_y);
  }
  if (run.size() > 1) {
    selected.push_back(run.end - 1);
  }
  std::sort(selected.begin(), selected.end());
  selected.erase(std::unique(selected.begin(), selected.end()), selected.end());
  for (std::size_t index : selected) {
    result.samples.push_back(
        {sample_at(index), index, index == run.begin});
  }
  ++result.retained_runs;
}

}  // namespace

PlotSampleReduction ReducePlotSamples(std::size_t sample_count,
                                      std::size_t sample_budget,
                                      std::size_t horizontal_bucket_count,
                                      const PlotSampleAccessor& sample_at) {
  PlotSampleReduction result;
  result.input_samples = sample_count;
  if (sample_count == 0 || !sample_at) {
    result.reduced = sample_count > 0;
    return result;
  }

  std::size_t endpoint_cost = 0;
  std::size_t total_interior = 0;
  std::size_t current_run_size = 0;
  const auto finish_run = [&]() {
    if (current_run_size == 0) {
      return;
    }
    ++result.input_runs;
    endpoint_cost += current_run_size == 1 ? 1 : 2;
    total_interior += current_run_size > 2 ? current_run_size - 2 : 0;
    current_run_size = 0;
  };
  for (std::size_t index = 0; index < sample_count; ++index) {
    if (FinitePoint(sample_at(index))) {
      ++result.finite_samples;
      ++current_run_size;
    } else {
      finish_run();
    }
  }
  finish_run();
  if (result.input_runs == 0) {
    result.reduced = sample_count > 0;
    return result;
  }

  if (result.finite_samples <= sample_budget) {
    result.samples.reserve(result.finite_samples);
    bool in_run = false;
    for (std::size_t index = 0; index < sample_count; ++index) {
      const QPointF point = sample_at(index);
      if (FinitePoint(point)) {
        result.samples.push_back({point, index, !in_run});
        in_run = true;
      } else {
        in_run = false;
      }
    }
    result.retained_runs = result.input_runs;
    return result;
  }
  result.reduced = true;

  if (endpoint_cost > sample_budget) {
    result.samples.reserve(sample_budget);
    std::size_t remaining = sample_budget;
    bool budget_exhausted = false;
    std::size_t run_begin = 0;
    std::size_t run_size = 0;
    const auto emit_run = [&]() {
      if (run_size == 0 || budget_exhausted) {
        run_size = 0;
        return;
      }
      const FiniteRun run{run_begin, run_begin + run_size};
      if (run.endpointCost() > remaining) {
        budget_exhausted = true;
        run_size = 0;
        return;
      }
      AppendRunEndpoints(result, run, sample_at);
      remaining -= run.endpointCost();
      run_size = 0;
    };
    for (std::size_t index = 0; index < sample_count && !budget_exhausted;
         ++index) {
      if (FinitePoint(sample_at(index))) {
        if (run_size == 0) {
          run_begin = index;
        }
        ++run_size;
      } else {
        emit_run();
      }
    }
    emit_run();
    result.dropped_runs = result.input_runs - result.retained_runs;
    return result;
  }

  std::vector<FiniteRun> runs;
  runs.reserve(result.input_runs);
  bool in_run = false;
  for (std::size_t index = 0; index < sample_count; ++index) {
    if (FinitePoint(sample_at(index))) {
      if (!in_run) {
        runs.push_back({index, index + 1});
        in_run = true;
      } else {
        runs.back().end = index + 1;
      }
    } else {
      in_run = false;
    }
  }
  const std::size_t extrema_slots = sample_budget - endpoint_cost;
  const std::size_t bucket_budget =
      std::min({horizontal_bucket_count, extrema_slots / 4, total_interior});
  const std::vector<std::size_t> buckets = AllocateBuckets(runs, bucket_budget);
  result.samples.reserve(endpoint_cost + bucket_budget * 4);
  for (std::size_t index = 0; index < runs.size(); ++index) {
    AppendReducedRun(result, runs[index], buckets[index], sample_at);
  }
  return result;
}

}  // namespace plot
}  // namespace autoviz
