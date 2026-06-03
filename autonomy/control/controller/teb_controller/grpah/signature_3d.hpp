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

#include <complex>
#include <optional>
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/control/controller/teb_controller/grpah/equivalence.hpp"
#include "autonomy/control/controller/teb_controller/config.hpp"
#include "autonomy/control/controller/teb_controller/geometry/obstacle.hpp"
#include "autonomy/control/controller/teb_controller/timed_elastic_band.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

/**
 * @brief Homotopy signature (3D)
 */
class HomotopySignature3d : public Equivalence
{
public:
    AUTONOMY_SHARED_PTR_DEFINITIONS(HomotopySignature3d);

    explicit HomotopySignature3d(const TimedElasticBandConfig& cfg);

    template <typename BidirIter, typename Fun>
    void CalculateHomotopySignature(
        BidirIter path_start, BidirIter path_end, Fun fun_cplx_point,
        const ObstContainer* obstacles,
        std::optional<TimeDiffSequence::iterator> timediff_start,
        std::optional<TimeDiffSequence::iterator> timediff_end) {
        hsignature3d_.resize(obstacles->size());

        std::advance(path_end, -1);
        constexpr int num_int_steps_per_segment = 10;

        for (std::size_t l = 0; l < obstacles->size(); ++l) {
            double H = 0;
            double transition_time = 0;
            double next_transition_time = 0;
            BidirIter path_iter;
            TimeDiffSequence::iterator timediff_iter;

            const Point& centroid = obstacles->at(l)->GetCentroid();
            Vector3 s1{static_cast<float>(centroid.x),
                       static_cast<float>(centroid.y), 0.0f};
            constexpr double kPredictionHorizon = 120.0;
            Point predicted_centroid;
            obstacles->at(l)->PredictCentroidConstantVelocity(kPredictionHorizon,
                                                              predicted_centroid);
            Vector3 s2{static_cast<float>(predicted_centroid.x),
                       static_cast<float>(predicted_centroid.y),
                       static_cast<float>(kPredictionHorizon)};
            Vector3 ds = s2 - s1;

            const float ds_sq_norm = SquaredNorm(ds);

            for (path_iter = path_start, timediff_iter = timediff_start.value();
                 path_iter != path_end; ++path_iter, ++timediff_iter) {
                std::complex<long double> z1 = fun_cplx_point(*path_iter);
                std::complex<long double> z2 =
                    fun_cplx_point(*std::next(path_iter));

                transition_time = next_transition_time;
                if (!timediff_start || !timediff_end) {
                    next_transition_time +=
                        std::abs(z2 - z1) / config_->robot.max_velocity_x;
                } else {
                    if (std::distance(path_iter, path_end) !=
                        std::distance(timediff_iter, timediff_end.value())) {
                        AERROR << "Size of Poses and timediff vectors does not "
                                  "match. This is a bug.";
                    }
                    next_transition_time += *timediff_iter;
                }

                Vector3 direction_vec{
                    static_cast<float>(z2.real() - z1.real()),
                    static_cast<float>(z2.imag() - z1.imag()),
                    static_cast<float>(next_transition_time - transition_time)};

                if (Norm(direction_vec) < 1e-15f) {
                    continue;
                }

                Vector3 r{static_cast<float>(z1.real()),
                          static_cast<float>(z1.imag()),
                          static_cast<float>(transition_time)};
                Vector3 dl = direction_vec *
                             (1.0f / static_cast<float>(num_int_steps_per_segment));
                Vector3 p1, p2, d, phi;
                for (int i = 0; i < num_int_steps_per_segment; ++i, r += dl) {
                    p1 = s1 - r;
                    p2 = s2 - r;
                    d = Cross(ds, Cross(p1, p2)) * (1.0f / ds_sq_norm);
                    const float p1_norm = Norm(p1);
                    const float p2_norm = Norm(p2);
                    const float d_sq_norm = SquaredNorm(d);
                    phi = ((Cross(d, p2) * (1.0f / p2_norm)) -
                           (Cross(d, p1) * (1.0f / p1_norm))) *
                          (1.0f / d_sq_norm);
                    H += Dot(phi, dl);
                }
            }

            hsignature3d_.at(l) = H / (4.0 * M_PI);
        }
    }

    bool IsEqual(const Equivalence& other) const override;

    bool IsValid() const override;

    bool IsReasonable() const override;

    const std::vector<double>& values() const;

private:
    const TimedElasticBandConfig* config_;
    std::vector<double> hsignature3d_;
};

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
