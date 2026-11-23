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
 * @file pan.hpp
 * @brief NeuPAN PAN loop: DUNE duals + NRMP.
 */

#include <memory>
#include <optional>
#include <vector>

#include "autonomy/control/controller/neupan_controller/distance_encoder.hpp"
#include "autonomy/control/controller/neupan_controller/nrmp.hpp"
#include "autonomy/control/controller/neupan_controller/robot_model.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

class Pan
{
public:
    struct Options {
        int iter_num{2};
        int dune_max_num{100};
        double iter_threshold{0.1};
    };

    struct Output {
        Mat3X opt_s;
        Mat2X opt_u;
        Vec opt_d;
        double min_distance{0.0};
        bool solved{false};
        int solver_status{-1};
    };

    struct DuneStage {
        std::vector<Mat> mu_list;
        std::vector<Mat2X> lam_list;
        std::vector<Mat2X> sort_point_list;
        double min_distance{0.0};
    };

    Pan(const RobotModel& robot, std::shared_ptr<const DistanceEncoder> encoder,
        const NrmpParams& nrmp_params, const Options& opts);

    Output Forward(Mat3X nom_s, Mat2X nom_u, const Mat3X& ref_s,
                   const Vec& ref_us, const Mat2X& points,
                   const std::optional<Nrmp::Box>& first = std::nullopt);

    void Reset() {
        prev_.reset();
    }

private:
    DuneStage RunDune(const Mat3X& nom_s, const Mat2X& points) const;
    bool StopCriteria(const Mat3X& nom_s, const Mat2X& nom_u,
                      const DuneStage* dune_out);

    RobotModel robot_;
    std::shared_ptr<const DistanceEncoder> encoder_;
    Nrmp nrmp_;
    Options opts_;

    struct Prev {
        Mat3X nom_s;
        Mat2X nom_u;
        std::vector<Mat> mu_list;
        std::vector<Mat2X> lam_list;
    };
    std::optional<Prev> prev_;
};

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
