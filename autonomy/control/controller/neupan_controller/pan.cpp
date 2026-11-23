/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/control/controller/neupan_controller/pan.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {
namespace {

Mat2X Downsample(const Mat2X& mat, int m) {
    const int n = static_cast<int>(mat.cols());
    if (m >= n) {
        return mat;
    }
    Mat2X out(2, m);
    for (int i = 0; i < m; ++i) {
        const int idx =
            m == 1 ? 0
                   : static_cast<int>(static_cast<double>(i) * (n - 1) / (m - 1));
        out.col(i) = mat.col(idx);
    }
    return out;
}

}  // namespace

Pan::Pan(const RobotModel& robot, std::shared_ptr<const DistanceEncoder> encoder,
         const NrmpParams& nrmp_params, const Options& opts)
    : robot_(robot),
      encoder_(std::move(encoder)),
      nrmp_(robot, nrmp_params),
      opts_(opts) {}

Pan::DuneStage Pan::RunDune(const Mat3X& nom_s, const Mat2X& points) const {
    if (!encoder_) {
        throw std::runtime_error("PAN: encoder is not loaded");
    }
    const Mat2X obs = Downsample(points, opts_.dune_max_num);
    DuneStage stage;
    stage.min_distance = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= robot_.T; ++i) {
        const Vec2 trans = nom_s.col(i).head<2>();
        const double theta = nom_s(2, i);
        Mat22 R;
        R << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
        const Mat2X point_flow = R.transpose() * (obs.colwise() - trans);

        Mat mu;
        Vec distances;
        std::string error;
        if (!encoder_->InferDuals(point_flow, &mu, &distances, &error)) {
            throw std::runtime_error("PAN DUNE: " + error);
        }
        const Eigen::Index n = point_flow.cols();
        const Mat2X lam = -R * robot_.G.transpose() * mu;
        if (i == 0 && n > 0) {
            stage.min_distance = distances.minCoeff();
        }
        std::vector<Eigen::Index> idx(static_cast<size_t>(n));
        std::iota(idx.begin(), idx.end(), 0);
        std::stable_sort(idx.begin(), idx.end(),
                         [&](Eigen::Index a, Eigen::Index b) {
                             return distances(a) < distances(b);
                         });
        Mat mu_sorted(mu.rows(), n);
        Mat2X lam_sorted(2, n);
        Mat2X pts_sorted(2, n);
        for (Eigen::Index k = 0; k < n; ++k) {
            mu_sorted.col(k) = mu.col(idx[static_cast<size_t>(k)]);
            lam_sorted.col(k) = lam.col(idx[static_cast<size_t>(k)]);
            pts_sorted.col(k) = obs.col(idx[static_cast<size_t>(k)]);
        }
        stage.mu_list.push_back(std::move(mu_sorted));
        stage.lam_list.push_back(std::move(lam_sorted));
        stage.sort_point_list.push_back(std::move(pts_sorted));
    }
    return stage;
}

bool Pan::StopCriteria(const Mat3X& nom_s, const Mat2X& nom_u,
                      const DuneStage* dune_out) {
    if (!prev_) {
        prev_ = Prev{nom_s, nom_u, {}, {}};
        if (dune_out) {
            prev_->mu_list = dune_out->mu_list;
            prev_->lam_list = dune_out->lam_list;
        }
        return false;
    }
    double diff = 0.0;
    if (!dune_out || prev_->mu_list.empty()) {
        diff = (nom_s - prev_->nom_s).squaredNorm() +
               (nom_u - prev_->nom_u).squaredNorm();
    } else {
        const int effect_num = std::min(
            {static_cast<int>(dune_out->mu_list[0].cols()),
             static_cast<int>(prev_->mu_list[0].cols()), nrmp_.params().max_num});
        double mu_sq = 0.0;
        double lam_sq = 0.0;
        for (size_t i = 0; i < dune_out->mu_list.size(); ++i) {
            mu_sq += (dune_out->mu_list[i].leftCols(effect_num) -
                      prev_->mu_list[i].leftCols(effect_num))
                         .squaredNorm();
            lam_sq += (dune_out->lam_list[i].leftCols(effect_num) -
                       prev_->lam_list[i].leftCols(effect_num))
                          .squaredNorm();
        }
        const double n = static_cast<double>(std::max(effect_num, 1));
        diff = mu_sq / (n * n) + lam_sq / (n * n);
    }
    prev_ = Prev{nom_s, nom_u, {}, {}};
    if (dune_out) {
        prev_->mu_list = dune_out->mu_list;
        prev_->lam_list = dune_out->lam_list;
    }
    return diff < opts_.iter_threshold;
}

Pan::Output Pan::Forward(Mat3X nom_s, Mat2X nom_u, const Mat3X& ref_s,
                         const Vec& ref_us, const Mat2X& points,
                         const std::optional<Nrmp::Box>& first) {
    Output out;
    out.min_distance = std::numeric_limits<double>::infinity();
    out.opt_s = nom_s;
    out.opt_u = nom_u;

    for (int i = 0; i < opts_.iter_num; ++i) {
        std::vector<Mat> fa_list;
        std::vector<Vec> fb_list;
        std::optional<DuneStage> dune_out;
        if (points.cols() > 0 && encoder_) {
            dune_out = RunDune(nom_s, points);
            out.min_distance = dune_out->min_distance;
            nrmp_.BuildFaFb(dune_out->mu_list, dune_out->lam_list,
                            dune_out->sort_point_list, &fa_list, &fb_list);
        } else {
            nrmp_.BuildFaFb({}, {}, {}, &fa_list, &fb_list);
        }

        const Nrmp::Result res =
            nrmp_.Solve(nom_s, nom_u, ref_s, ref_us, fa_list, fb_list, first);
        out.solver_status = res.status;
        if (!res.success) {
            break;
        }
        out.solved = true;
        nom_s = res.s;
        nom_u = res.u;
        out.opt_s = res.s;
        out.opt_u = res.u;
        out.opt_d = res.d;
        if (StopCriteria(nom_s, nom_u, dune_out ? &*dune_out : nullptr)) {
            break;
        }
    }
    return out;
}

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
