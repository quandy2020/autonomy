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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_INTERNAL_SE3_SHOT_VERTEX_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_INTERNAL_SE3_SHOT_VERTEX_HPP_

#include "autonomy/localization/atlas/type.hpp"

#include <g2o/core/base_vertex.h>
#include <g2o/types/slam3d/se3quat.h>

namespace autonomy::localization::atlas {
namespace optimize {
namespace internal {
namespace se3 {

class shot_vertex final : public g2o::BaseVertex<6, g2o::SE3Quat> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    shot_vertex();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void setToOriginImpl() override;

    void oplusImpl(const double* update_) override;
};

inline shot_vertex::shot_vertex()
    : g2o::BaseVertex<6, g2o::SE3Quat>() {}

inline bool shot_vertex::shot_vertex::read(std::istream& is) {
    Vec7_t estimate;
    for (unsigned int i = 0; i < 7; ++i) {
        is >> estimate(i);
    }
    g2o::SE3Quat g2o_cam_pose_wc;
    g2o_cam_pose_wc.fromVector(estimate);
    setEstimate(g2o_cam_pose_wc.inverse());
    return true;
}

inline bool shot_vertex::shot_vertex::write(std::ostream& os) const {
    g2o::SE3Quat g2o_cam_pose_wc(estimate().inverse());
    for (unsigned int i = 0; i < 7; ++i) {
        os << g2o_cam_pose_wc[i] << " ";
    }
    return os.good();
}

inline void shot_vertex::setToOriginImpl() {
    _estimate = g2o::SE3Quat();
}

inline void shot_vertex::oplusImpl(const double* update_) {
    Eigen::Map<const Vec6_t> update(update_);
    setEstimate(g2o::SE3Quat::exp(update) * estimate());
}

} // namespace se3
} // namespace internal
} // namespace optimize
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_INTERNAL_SE3_SHOT_VERTEX_HPP_
