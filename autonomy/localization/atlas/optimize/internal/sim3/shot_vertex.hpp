/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_INTERNAL_SIM3_SHOT_VERTEX_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_INTERNAL_SIM3_SHOT_VERTEX_HPP_

#include "autonomy/localization/atlas/type.hpp"

#include <g2o/core/base_vertex.h>
#include <g2o/types/sim3/sim3.h>

namespace autonomy::localization::atlas {
namespace optimize {
namespace internal {
namespace sim3 {

class shot_vertex final : public g2o::BaseVertex<7, g2o::Sim3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    shot_vertex();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void setToOriginImpl() override;

    void oplusImpl(const double* update_) override;

    bool fix_scale_;
};

inline shot_vertex::shot_vertex()
    : g2o::BaseVertex<7, g2o::Sim3>() {}

inline bool shot_vertex::read(std::istream& is) {
    Vec7_t g2o_sim3_wc;
    for (int i = 0; i < 7; ++i) {
        is >> g2o_sim3_wc(i);
    }
    setEstimate(g2o::Sim3(g2o_sim3_wc).inverse());
    return true;
}

inline bool shot_vertex::write(std::ostream& os) const {
    g2o::Sim3 g2o_Sim3_wc(estimate().inverse());
    const Vec7_t g2o_sim3_wc = g2o_Sim3_wc.log();
    for (int i = 0; i < 7; ++i) {
        os << g2o_sim3_wc(i) << " ";
    }
    return os.good();
}

inline void shot_vertex::setToOriginImpl() {
    _estimate = g2o::Sim3();
}

inline void shot_vertex::oplusImpl(const double* update_) {
    Eigen::Map<Vec7_t> update(const_cast<double*>(update_));

    if (fix_scale_) {
        update(6) = 0;
    }

    const g2o::Sim3 s(update);
    setEstimate(s * estimate());
}

} // namespace sim3
} // namespace internal
} // namespace optimize
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_INTERNAL_SIM3_SHOT_VERTEX_HPP_
