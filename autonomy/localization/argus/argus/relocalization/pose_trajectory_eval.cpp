/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "argus/relocalization/pose_trajectory_eval.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace argus {
namespace relocalization {
namespace {

transform::Rigid3d FromTum(double tx, double ty, double tz, double qx,
                           double qy, double qz, double qw) {
    Eigen::Quaterniond q(qw, qx, qy, qz);
    if (q.norm() > 1e-12) {
        q.normalize();
    } else {
        q = Eigen::Quaterniond::Identity();
    }
    return transform::Rigid3d(Eigen::Vector3d(tx, ty, tz), q);
}

}  // namespace

bool LoadTumTrajectory(const std::string& path, std::vector<TimedPose>* out) {
    if (out == nullptr) {
        return false;
    }
    out->clear();
    std::ifstream in(path);
    if (!in) {
        return false;
    }
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream ss(line);
        double t = 0;
        double tx = 0;
        double ty = 0;
        double tz = 0;
        double qx = 0;
        double qy = 0;
        double qz = 0;
        double qw = 1;
        if (!(ss >> t >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
            continue;
        }
        TimedPose p;
        p.stamp_sec = t;
        p.T_w_b = FromTum(tx, ty, tz, qx, qy, qz, qw);
        out->push_back(p);
    }
    std::sort(out->begin(), out->end(),
              [](const TimedPose& a, const TimedPose& b) {
                  return a.stamp_sec < b.stamp_sec;
              });
    return !out->empty();
}

TrajectoryEvalSummary EvaluateTrajectories(const std::vector<TimedPose>& estimate,
                                     const std::vector<TimedPose>& ground_truth,
                                     double max_dt_sec) {
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
        const RelocEvalMetrics m = EvaluateAgainstGroundTruth(
            e.T_w_b, ground_truth[best].T_w_b, 1e9, 1e9);
        sum_t2 += m.translation_error_m * m.translation_error_m;
        sum_t += m.translation_error_m;
        sum_r2 += m.rotation_error_rad * m.rotation_error_rad;
        sum_r += m.rotation_error_rad;
        max_t = std::max(max_t, m.translation_error_m);
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

}  // namespace relocalization
}  // namespace argus
