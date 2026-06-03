/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include <math.h>

#include <algorithm>
#include <complex>
#include <functional>
#include <iterator>
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/common/math/math.hpp"
#include "autonomy/control/controller/teb_controller/grpah/equivalence.hpp"
#include "autonomy/control/controller/teb_controller/config.hpp"
#include "autonomy/control/controller/teb_controller/geometry/obstacle.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

/**
 * @brief Homotopy signature (2D)
 */
class HomotopySignature : public Equivalence
{
public:
    AUTONOMY_SHARED_PTR_DEFINITIONS(HomotopySignature);

    explicit HomotopySignature(const TimedElasticBandConfig& cfg);

    template <typename BidirIter, typename Fun>
    void CalculateHomotopySignature(BidirIter path_start, BidirIter path_end,
                                    Fun fun_cplx_point,
                                    const ObstContainer* obstacles) {
        if (obstacles->empty()) {
            hsignature_ = std::complex<double>(0, 0);
            return;
        }

        if (!(config_->homotopy.homotopy_signature_prescaler > 0.1 &&
              config_->homotopy.homotopy_signature_prescaler <= 1)) {
            AINFO << "Only a preScaler on the interval (0.1,1] is allowed.";
        }

        int m = std::max((int)obstacles->size() - 1, 5);

        int a = (int)std::ceil(double(m) / 2.0);
        int b = m - a;

        std::advance(path_end, -1);

        using cplx = std::complex<long double>;
        cplx start = fun_cplx_point(*path_start);
        cplx end = fun_cplx_point(*path_end);
        cplx delta = end - start;
        cplx normal(-delta.imag(), delta.real());
        cplx map_bottom_left;
        cplx map_top_right;
        if (std::abs(delta) < 3.0) {
            map_bottom_left = start + cplx(0, -3);
            map_top_right = start + cplx(3, 3);
        } else {
            map_bottom_left = start - normal;
            map_top_right = start + delta + normal;
        }

        hsignature_ = 0;

        std::vector<double> imag_proposals(5);

        while (path_start != path_end) {
            cplx z1 = fun_cplx_point(*path_start);
            cplx z2 = fun_cplx_point(*std::next(path_start));

            for (std::size_t l = 0; l < obstacles->size(); ++l) {
                cplx obst_l = obstacles->at(l)->GetCentroidCplx();
                cplx f0 =
                    (long double)config_->homotopy.homotopy_signature_prescaler *
                    (long double)a * (obst_l - map_bottom_left) * (long double)b *
                    (obst_l - map_top_right);

                cplx Al = f0;
                for (std::size_t j = 0; j < obstacles->size(); ++j) {
                    if (j == l)
                        continue;
                    cplx obst_j = obstacles->at(j)->GetCentroidCplx();
                    cplx diff = obst_l - obst_j;
                    if (std::abs(diff) < 0.05)
                        continue;
                    else
                        Al /= diff;
                }
                double diff2 = std::abs(z2 - obst_l);
                double diff1 = std::abs(z1 - obst_l);
                if (diff2 == 0 || diff1 == 0)
                    continue;
                double log_real = std::log(diff2) - std::log(diff1);
                double arg_diff = std::arg(z2 - obst_l) - std::arg(z1 - obst_l);
                imag_proposals.at(0) = arg_diff;
                imag_proposals.at(1) = arg_diff + 2 * M_PI;
                imag_proposals.at(2) = arg_diff - 2 * M_PI;
                imag_proposals.at(3) = arg_diff + 4 * M_PI;
                imag_proposals.at(4) = arg_diff - 4 * M_PI;
                double log_imag =
                    *std::min_element(imag_proposals.begin(),
                                      imag_proposals.end(),
                                      autonomy::common::SmallerThanAbs);
                cplx log_value(log_real, log_imag);
                hsignature_ += Al * log_value;
            }
            ++path_start;
        }
    }

    bool IsEqual(const Equivalence& other) const override;

    bool IsValid() const override;

    bool IsReasonable() const override;

    const std::complex<long double>& value() const;

private:
    const TimedElasticBandConfig* config_;
    std::complex<long double> hsignature_;
};

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
