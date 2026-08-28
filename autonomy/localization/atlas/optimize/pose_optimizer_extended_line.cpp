/*
 * Point + line pose-only BA (Structure-PLP-SLAM).
 */
#include "autonomy/localization/atlas/data/frame.hpp"
#include "autonomy/localization/atlas/data/landmark.hpp"
#include "autonomy/localization/atlas/data/landmark_line.hpp"
#include "autonomy/localization/atlas/optimize/pose_optimizer_extended_line.hpp"
#include "autonomy/localization/atlas/optimize/internal/se3/pose_opt_edge_wrapper.hpp"
#include "autonomy/localization/atlas/optimize/internal/se3/shot_vertex.hpp"
#include "autonomy/localization/atlas/camera/perspective.hpp"
#include "autonomy/localization/atlas/type.hpp"
#include "autonomy/localization/atlas/util/converter.hpp"

#include <utility>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include "autonomy/localization/atlas/optimize/g2o/se3/pose_opt_edge_line3d_orthonormal.hpp"

namespace autonomy::localization::atlas::optimize {

namespace {

using line_pose_edge_wrap = std::pair<::g2o::OptimizableGraph::Edge*, unsigned int>;

line_pose_edge_wrap make_line_pose_edge(internal::se3::shot_vertex* vtx,
                                            const camera::base* camera,
                                            const Vec6_t& pluecker,
                                            unsigned int idx,
                                            const cv::Point2f& sp,
                                            const cv::Point2f& ep,
                                            float inv_sigma_sq,
                                            float sqrt_chi_sq) {
    auto* g2o_edge = new optimize::plp_g2o::se3::pose_opt_edge_line3d();
    g2o_edge->setMeasurement(Vec4_t(sp.x, sp.y, ep.x, ep.y));
    g2o_edge->setInformation(Mat22_t::Identity() * inv_sigma_sq);
    if (camera->model_type_ == camera::model_type_t::Perspective) {
        const auto* c = static_cast<const camera::perspective*>(camera);
        g2o_edge->_fx = c->fx_;
        g2o_edge->_fy = c->fy_;
        g2o_edge->_cx = c->cx_;
        g2o_edge->_cy = c->cy_;
        g2o_edge->_K << c->fy_, 0.0, 0.0,
            0.0, c->fx_, 0.0,
            -c->fy_ * c->cx_, -c->fx_ * c->cy_, c->fx_ * c->fy_;
    }
    g2o_edge->_pos_w = pluecker;
    g2o_edge->setVertex(0, vtx);
    auto* huber = new ::g2o::RobustKernelHuber();
    huber->setDelta(sqrt_chi_sq);
    g2o_edge->setRobustKernel(huber);
    return {g2o_edge, idx};
}

}  // namespace

pose_optimizer_extended_line::pose_optimizer_extended_line(unsigned int num_trials, unsigned int num_each_iter)
    : num_trials_(num_trials), num_each_iter_(num_each_iter) {}

unsigned int pose_optimizer_extended_line::optimize(data::frame& frm, Mat44_t& optimized_pose,
                                                    std::vector<bool>& outlier_flags,
                                                    std::vector<bool>& outlier_flags_line) const {
    using pose_opt_edge_wrapper = internal::se3::pose_opt_edge_wrapper;

    auto linear_solver = autonomy::localization::atlas::make_unique<::g2o::LinearSolverEigen<::g2o::BlockSolver_6_3::PoseMatrixType>>();
    auto block_solver = autonomy::localization::atlas::make_unique<::g2o::BlockSolver_6_3>(std::move(linear_solver));
    auto algorithm = new ::g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    ::g2o::SparseOptimizer g2o_optimizer;
    g2o_optimizer.setAlgorithm(algorithm);
    g2o_optimizer.setVerbose(false);

    auto* frm_vtx = new internal::se3::shot_vertex();
    frm_vtx->setId(0);
    frm_vtx->setEstimate(util::converter::to_g2o_SE3(frm.get_pose_cw()));
    frm_vtx->setFixed(false);
    g2o_optimizer.addVertex(frm_vtx);

    const unsigned int num_keypts = frm.frm_obs_.undist_keypts_.size();
    outlier_flags.resize(num_keypts, false);

    std::vector<pose_opt_edge_wrapper> point_edges;
    point_edges.reserve(num_keypts);

    constexpr float chi_sq_2D = 5.99146f;
    const float sqrt_chi_sq_2D = std::sqrt(chi_sq_2D);
    constexpr float chi_sq_3D = 7.81473f;
    const float sqrt_chi_sq_3D = std::sqrt(chi_sq_3D);

    unsigned int num_init_obs = 0;
    for (unsigned int idx = 0; idx < num_keypts; ++idx) {
        const auto& lm = frm.get_landmark(idx);
        if (!lm || lm->will_be_erased()) {
            continue;
        }
        ++num_init_obs;
        const auto& undist_keypt = frm.frm_obs_.undist_keypts_.at(idx);
        const float x_right = frm.frm_obs_.stereo_x_right_.empty() ? -1.f : frm.frm_obs_.stereo_x_right_.at(idx);
        const float inv_sigma_sq = frm.orb_params_->inv_level_sigma_sq_.at(undist_keypt.octave);
        const float sqrt_chi_sq = (frm.camera_->setup_type_ == camera::setup_type_t::Monocular) ? sqrt_chi_sq_2D : sqrt_chi_sq_3D;
        point_edges.emplace_back(frm.camera_, frm_vtx, lm->get_pos_in_world(),
                                 idx, undist_keypt.pt.x, undist_keypt.pt.y, x_right,
                                 inv_sigma_sq, sqrt_chi_sq);
        g2o_optimizer.addEdge(point_edges.back().edge_);
    }

    const unsigned int num_keylines = static_cast<unsigned int>(frm.line_obs_.size());
    outlier_flags_line.resize(num_keylines, false);
    std::vector<line_pose_edge_wrap> line_edges;
    line_edges.reserve(num_keylines);

    for (unsigned int idx = 0; idx < num_keylines; ++idx) {
        const auto& lm_line = frm.get_landmark_line(idx);
        if (!lm_line || lm_line->will_be_erased()) {
            continue;
        }
        const auto& keyline = frm.line_obs_.keylines.at(idx);
        const float inv_sigma_sq = frm.inv_level_sigma_sq_lsd_.at(static_cast<size_t>(keyline.octave));
        auto line_wrap = make_line_pose_edge(frm_vtx, frm.camera_, lm_line->get_pluecker_coord(), idx,
                                        keyline.getStartPoint(), keyline.getEndPoint(),
                                        inv_sigma_sq, sqrt_chi_sq_2D);
        line_edges.push_back(line_wrap);
        g2o_optimizer.addEdge(line_wrap.first);
    }

    if (num_init_obs < 5) {
        delete frm_vtx;
        return 0;
    }

    unsigned int num_bad_obs = 0;
    for (unsigned int trial = 0; trial < num_trials_; ++trial) {
        g2o_optimizer.initializeOptimization();
        g2o_optimizer.optimize(static_cast<int>(num_each_iter_));
        num_bad_obs = 0;

        for (auto& wrap : point_edges) {
            auto* edge = wrap.edge_;
            if (outlier_flags.at(wrap.idx_)) {
                edge->computeError();
            }
            const bool is_mono = wrap.is_monocular_;
            const float chi_sq = is_mono ? chi_sq_2D : chi_sq_3D;
            if (chi_sq < edge->chi2()) {
                outlier_flags.at(wrap.idx_) = true;
                wrap.set_as_outlier();
                ++num_bad_obs;
            } else {
                outlier_flags.at(wrap.idx_) = false;
                wrap.set_as_inlier();
            }
            if (trial == num_trials_ - 2) {
                edge->setRobustKernel(nullptr);
            }
        }

        if (num_init_obs - num_bad_obs < 5) {
            break;
        }

        for (auto& line_wrap : line_edges) {
            auto* line_edge = line_wrap.first;
            if (outlier_flags_line.at(line_wrap.second)) {
                line_edge->computeError();
            }
            if (chi_sq_2D < line_edge->chi2()) {
                outlier_flags_line.at(line_wrap.second) = true;
                line_edge->setLevel(1);
                line_edge->setRobustKernel(nullptr);
            } else {
                outlier_flags_line.at(line_wrap.second) = false;
                line_edge->setLevel(0);
            }
            if (trial == num_trials_ - 2) {
                line_edge->setRobustKernel(nullptr);
            }
        }
    }

    optimized_pose = util::converter::to_eigen_mat(frm_vtx->estimate());
    return num_init_obs - num_bad_obs;
}

}  // namespace autonomy::localization::atlas::optimize
