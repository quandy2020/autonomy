#pragma once

#include "autonomy/localization/atlas/data/landmark.hpp"
#include "autonomy/localization/atlas/optimize/g2o/extended/plane_point_distance_edge.hpp"
#include "autonomy/localization/atlas/optimize/internal/landmark_vertex.hpp"

#include <g2o/core/robust_kernel_impl.h>
#include <memory>

namespace autonomy::localization::atlas::optimize::plp_g2o::extended {

class point2plane_edge_wrapper {
public:
    point2plane_edge_wrapper() = delete;

    point2plane_edge_wrapper(const std::shared_ptr<data::landmark>& lm,
                             internal::landmark_vertex* lm_vtx,
                             const Vec4_t& plane_function,
                             const bool use_huber_loss = true)
        : lm_(lm) {
        auto* edge = new point_plane_distance_edge();
        edge->setMeasurement(plane_function);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        edge->setVertex(0, lm_vtx);
        edge_ = edge;

        if (use_huber_loss) {
            auto* huber_kernel = new ::g2o::RobustKernelHuber();
            huber_kernel->setDelta(1.0);
            edge_->setRobustKernel(huber_kernel);
        }
    }

    bool is_inlier() const { return edge_->level() == 0; }
    bool is_outlier() const { return edge_->level() != 0; }
    void set_as_inlier() const { edge_->setLevel(0); }
    void set_as_outlier() const { edge_->setLevel(1); }

    ::g2o::OptimizableGraph::Edge* edge_ = nullptr;
    std::shared_ptr<data::landmark> lm_;
};

}  // namespace autonomy::localization::atlas::optimize::plp_g2o::extended
