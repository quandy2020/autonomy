#pragma once

#include "autonomy/localization/atlas/camera/perspective.hpp"
#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/data/landmark_line.hpp"
#include "autonomy/localization/atlas/optimize/g2o/landmark_vertex_line3d.hpp"
#include "autonomy/localization/atlas/optimize/g2o/se3/reproj_edge_line3d_orthonormal.hpp"
#include "autonomy/localization/atlas/optimize/internal/se3/shot_vertex.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <g2o/core/robust_kernel_impl.h>
#include <opencv2/core/types.hpp>

namespace autonomy::localization::atlas::optimize::plp_g2o::se3 {

class line_reproj_edge_wrapper {
public:
    line_reproj_edge_wrapper() = delete;

    line_reproj_edge_wrapper(const std::shared_ptr<data::keyframe>& shot,
                             internal::se3::shot_vertex* shot_vtx,
                             VertexLine3D* lm_vtx,
                             unsigned int idx,
                             const cv::Point2f& sp,
                             const cv::Point2f& ep,
                             float inv_sigma_sq,
                             float sqrt_chi_sq,
                             bool use_huber_loss = true)
        : shot_(shot), lm_line_(shot->get_landmark_line(idx)), idx_(idx) {
        auto* edge = new reproj_edge_line3d();
        edge->setMeasurement(Vec4_t(sp.x, sp.y, ep.x, ep.y));
        edge->setInformation(Mat22_t::Identity() * inv_sigma_sq);

        if (shot->camera_->model_type_ == camera::model_type_t::Perspective) {
            const auto* c = static_cast<const camera::perspective*>(shot->camera_);
            edge->_fx = c->fx_;
            edge->_fy = c->fy_;
            edge->_cx = c->cx_;
            edge->_cy = c->cy_;
            edge->_K << c->fy_, 0.0, 0.0,
                0.0, c->fx_, 0.0,
                -c->fy_ * c->cx_, -c->fx_ * c->cy_, c->fx_ * c->fy_;
            edge->_cam_matrix = c->eigen_cam_matrix_;
        }

        edge->setVertex(0, shot_vtx);
        edge->setVertex(1, lm_vtx);
        edge_ = edge;

        if (use_huber_loss) {
            auto* huber_kernel = new ::g2o::RobustKernelHuber();
            huber_kernel->setDelta(sqrt_chi_sq);
            edge_->setRobustKernel(huber_kernel);
        }
    }

    bool depth_is_positive_via_endpoints_trimming() const {
        return static_cast<reproj_edge_line3d*>(edge_)->depth_is_positive_via_endpoints_trimming();
    }

    void set_as_outlier() const { edge_->setLevel(1); }
    void set_as_inlier() const { edge_->setLevel(0); }

    ::g2o::OptimizableGraph::Edge* edge_ = nullptr;
    std::shared_ptr<data::keyframe> shot_;
    std::shared_ptr<data::landmark_line> lm_line_;
    unsigned int idx_ = 0;
};

}  // namespace autonomy::localization::atlas::optimize::plp_g2o::se3
