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

#pragma once

/**
 * @file nrmp.hpp
 * @brief NeuPAN NRMP receding-horizon QP, solved with DAQP.
 */

#include <optional>
#include <vector>

#include "autonomy/control/controller/neupan_controller/robot_model.hpp"
#include "autonomy/control/controller/neupan_controller/types.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

/**
 * @class autonomy::control::controller::neupan_controller::Nrmp
 * @brief Quadratic program for tracking + obstacle dual constraints.
 *
 * Decision z = [s (3*(T+1)) | u (2*T) | d (T) | e (max_num*T)].
 */
class Nrmp
{
public:
    struct Result {
        Mat3X s;
        Mat2X u;
        Vec d;
        double objective{0.0};
        bool success{false};
        int status{-1};
    };

    struct Box {
        Vec2 lo;
        Vec2 hi;
    };

    Nrmp(const RobotModel& robot, const NrmpParams& params);

    Result Solve(const Mat3X& nom_s, const Mat2X& nom_u, const Mat3X& ref_s,
                 const Vec& ref_us, const std::vector<Mat>& fa_list,
                 const std::vector<Vec>& fb_list,
                 const std::optional<Box>& first = std::nullopt) const;

    void BuildFaFb(const std::vector<Mat>& mu_list,
                   const std::vector<Mat2X>& lam_list,
                   const std::vector<Mat2X>& point_list,
                   std::vector<Mat>* fa_list, std::vector<Vec>* fb_list) const;

    const NrmpParams& params() const {
        return params_;
    }

private:
    double StateWeight(int row) const {
        return row == 2 && robot_.kinematics == Kinematics::kOmni ? 0.0
                                                                  : params_.q_s(row);
    }

    RobotModel robot_;
    NrmpParams params_;
    int T_{0};
    int M_{0};
    int nz_{0};
    int nc_{0};
    int off_u_{0};
    int off_d_{0};
    int off_e_{0};
};

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
