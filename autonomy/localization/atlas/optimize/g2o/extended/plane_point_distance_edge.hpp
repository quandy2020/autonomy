#pragma once

#include "autonomy/localization/atlas/optimize/internal/landmark_vertex.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <g2o/core/base_unary_edge.h>

namespace autonomy::localization::atlas::optimize::plp_g2o::extended {

/** Point-to-plane distance edge with fixed plane parameters (Structure-PLP-SLAM). */
class point_plane_distance_edge final : public ::g2o::BaseUnaryEdge<1, Vec4_t, internal::landmark_vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void computeError() override {
        const auto* vt_pt = static_cast<const internal::landmark_vertex*>(_vertices[0]);
        const Vec3_t pos_w = vt_pt->estimate();
        const Vec4_t plane_function(_measurement);
        _error[0] = (pos_w.dot(plane_function.head<3>()) + plane_function(3)) / plane_function.head<3>().norm();
    }

    bool read(std::istream&) override { return true; }
    bool write(std::ostream&) const override { return true; }
};

}  // namespace autonomy::localization::atlas::optimize::plp_g2o::extended
