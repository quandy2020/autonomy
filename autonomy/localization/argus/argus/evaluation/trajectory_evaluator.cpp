/*
 * Copyright 2026 The Openbot Authors
 */
#include "argus/evaluation/trajectory_evaluator.hpp"

#include <algorithm>
#include <cmath>

#include "argus/common/transform.hpp"

namespace argus {
namespace evaluation {

TrajectoryEvalSummary EvaluateTrajectories(
    const std::vector<TimedPose>& estimate,
    const std::vector<TimedPose>& ground_truth, double max_dt_sec) {
    TrajectoryEvalSummary s;
    if (estimate.empty() || ground_truth.empty()) {
        s.error = "empty trajectory";
        return s;
    }
    double sum_t2 = 0;
    double sum_t = 0;
    double sum_r2 = 0;
    double sum_r = 0;
    double max_t = 0;
    int n = 0;
    std::size_t j = 0;
    for (const TimedPose& e : estimate) {
        while (j + 1 < ground_truth.size() &&
               ground_truth[j + 1].stamp_sec <= e.stamp_sec) {
            ++j;
        }
        std::size_t best = j;
        if (j + 1 < ground_truth.size()) {
            const double d0 = std::abs(ground_truth[j].stamp_sec - e.stamp_sec);
            const double d1 =
                std::abs(ground_truth[j + 1].stamp_sec - e.stamp_sec);
            if (d1 < d0) {
                best = j + 1;
            }
        }
        const double dt = std::abs(ground_truth[best].stamp_sec - e.stamp_sec);
        if (dt > max_dt_sec) {
            continue;
        }
        const transform::Rigid3d err =
            ground_truth[best].T_w_b.inverse() * e.T_w_b;
        const double te = err.translation().norm();
        const double re = transform::GetAngle(err);
        sum_t2 += te * te;
        sum_t += te;
        sum_r2 += re * re;
        sum_r += re;
        max_t = std::max(max_t, te);
        ++n;
    }
    if (n == 0) {
        s.error = "no matched pairs within max_dt";
        return s;
    }
    s.matched_pairs = n;
    s.ate_rmse_m = std::sqrt(sum_t2 / n);
    s.ate_mean_m = sum_t / n;
    s.ate_max_m = max_t;
    s.rot_rmse_rad = std::sqrt(sum_r2 / n);
    s.rot_mean_rad = sum_r / n;
    s.ok = true;
    return s;
}

}  // namespace evaluation
}  // namespace argus
