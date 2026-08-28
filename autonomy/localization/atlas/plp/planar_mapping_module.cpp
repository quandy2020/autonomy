#include "autonomy/localization/atlas/plp/planar_mapping_module.hpp"

#include "autonomy/localization/atlas/camera/perspective.hpp"
#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/data/landmark.hpp"
#include "autonomy/localization/atlas/data/landmark_plane.hpp"
#include "autonomy/localization/atlas/data/map_database.hpp"
#include "autonomy/localization/atlas/solve/GCRANSAC/GCRANSAC.h"
#include "autonomy/localization/atlas/solve/GCRANSAC/flann_neighborhood_graph.h"
#include "autonomy/localization/atlas/solve/GCRANSAC/preemption_sprt.h"
#include "autonomy/localization/atlas/solve/GCRANSAC/types.h"
#include "autonomy/localization/atlas/solve/GCRANSAC/uniform_sampler.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <random>

#include <Eigen/SVD>
#include <opencv2/core.hpp>

#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas::plp {

namespace {

size_t count_plane_points(const std::shared_ptr<data::landmark_plane>& plane) {
    size_t count = plane->get_fit_samples().size();
    for (const auto& lm : plane->get_landmarks()) {
        if (lm && !lm->will_be_erased()) {
            ++count;
        }
    }
    return count;
}

std::vector<Vec3_t> collect_plane_points(const std::shared_ptr<data::landmark_plane>& plane) {
    std::vector<Vec3_t> points;
    for (const auto& lm : plane->get_landmarks()) {
        if (lm && !lm->will_be_erased()) {
            points.push_back(lm->get_pos_in_world());
        }
    }
    for (const auto& p : plane->get_fit_samples()) {
        points.push_back(p);
    }
    return points;
}

double fit_plane_svd(const std::vector<Vec3_t>& points, const std::vector<int>& indexes, double& a, double& b,
                     double& c, double& d) {
    if (indexes.empty()) {
        return std::numeric_limits<double>::max();
    }
    Eigen::Matrix<double, 3, Eigen::Dynamic> X(3, static_cast<int>(indexes.size()));
    for (size_t i = 0; i < indexes.size(); ++i) {
        X.col(static_cast<int>(i)) = points.at(static_cast<size_t>(indexes[i]));
    }
    const Vec3_t centroid = X.rowwise().mean();
    const Eigen::Matrix<double, 3, Eigen::Dynamic> centered = X.colwise() - centroid;
    const Eigen::JacobiSVD<Eigen::Matrix<double, 3, Eigen::Dynamic>> svd(centered, Eigen::ComputeFullU);
    const Vec3_t normal = svd.matrixU().col(2).normalized();
    d = -normal.dot(centroid);
    a = normal(0);
    b = normal(1);
    c = normal(2);
    const Eigen::VectorXd residual = (X.transpose() * normal).array() + d;
    return std::abs(residual.norm() / static_cast<double>(indexes.size()));
}

void add_rgbd_depth_samples(const std::shared_ptr<data::keyframe>& keyfrm, const long label,
                            std::shared_ptr<data::landmark_plane>& plane) {
    const cv::Mat& depth = keyfrm->depth_map_;
    const cv::Mat& seg = keyfrm->get_segmentation_mask();
    const auto* cam = dynamic_cast<const camera::perspective*>(keyfrm->camera_);
    if (depth.empty() || seg.empty() || depth.type() != CV_32FC1 || cam == nullptr || !plane) {
        return;
    }

    const Mat33_t rot_wc = keyfrm->get_rot_cw().transpose();
    const Vec3_t trans_wc = keyfrm->get_trans_wc();
    constexpr int kStride = 6;
    constexpr float kMinDepth = 0.2f;
    constexpr float kMaxDepth = 8.f;
    constexpr unsigned int kMaxSamples = 128;

    for (int v = 0; v < depth.rows; v += kStride) {
        const float* row = depth.ptr<float>(v);
        for (int u = 0; u < depth.cols; u += kStride) {
            if (plane->get_fit_samples().size() >= kMaxSamples) {
                return;
            }
            const float z = row[u];
            if (!std::isfinite(z) || z < kMinDepth || z > kMaxDepth) {
                continue;
            }
            if (u >= seg.cols || v >= seg.rows) {
                continue;
            }
            const auto color = seg.at<cv::Vec3b>(v, u);
            const uint32_t id = static_cast<uint32_t>(color[0]) | (static_cast<uint32_t>(color[1]) << 8) |
                                (static_cast<uint32_t>(color[2]) << 16);
            if (id == 0 || static_cast<long>(id) != label) {
                continue;
            }

            const float x = (static_cast<float>(u) - static_cast<float>(cam->cx_)) * z / static_cast<float>(cam->fx_);
            const float y = (static_cast<float>(v) - static_cast<float>(cam->cy_)) * z / static_cast<float>(cam->fy_);
            const Vec3_t pos_w = rot_wc * Vec3_t(x, y, z) + trans_wc;
            plane->add_fit_sample(pos_w);
        }
    }
}

}  // namespace

planar_mapping_module::planar_mapping_module(data::map_database* map_db, const bool is_monocular, const Options& options)
    : map_db_(map_db), is_monocular_(is_monocular), options_(options) {}

long planar_mapping_module::color_hash(const cv::Vec3b& color) {
    const uint32_t id = static_cast<uint32_t>(color[0]) | (static_cast<uint32_t>(color[1]) << 8) |
                        (static_cast<uint32_t>(color[2]) << 16);
    return id == 0 ? 0 : static_cast<long>(id);
}

void planar_mapping_module::estimate_map_scale(const std::shared_ptr<data::keyframe>& keyfrm) {
    const double median_depth = keyfrm->compute_median_depth(true);
    map_scale_ = (median_depth > 1e-6) ? (1.0 / median_depth) : 1.0;
    planar_distance_thresh_ = options_.plane_distance_correction * map_scale_;
    final_error_thresh_ = options_.final_error_correction * map_scale_;
}

void planar_mapping_module::estimate_map_scale() {
    const auto lms = map_db_->get_all_landmarks();
    if (lms.empty()) {
        map_scale_ = 1.0;
        return;
    }
    double sum = 0.0;
    unsigned int count = 0;
    for (const auto& lm : lms) {
        if (!lm || lm->will_be_erased()) {
            continue;
        }
        sum += lm->get_pos_in_world().norm();
        ++count;
    }
    map_scale_ = (count > 0) ? (sum / count) : 1.0;
    planar_distance_thresh_ = options_.plane_distance_correction * map_scale_;
    final_error_thresh_ = options_.final_error_correction * map_scale_;
}

bool planar_mapping_module::process_new_keyframe(const std::shared_ptr<data::keyframe>& keyfrm) {
    if (!keyfrm || keyfrm->will_be_erased()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mtx_plane_);

    if (is_monocular_) {
        estimate_map_scale(keyfrm);
    } else if (map_db_->get_num_keyframes() < 3) {
        estimate_map_scale();
    }

    std::unordered_map<long, std::shared_ptr<data::landmark_plane>> color_to_planes;
    if (!create_color_to_plane(keyfrm, color_to_planes)) {
        return false;
    }
    return create_new_planes(color_to_planes);
}

bool planar_mapping_module::create_color_to_plane(
    const std::shared_ptr<data::keyframe>& keyfrm,
    std::unordered_map<long, std::shared_ptr<data::landmark_plane>>& color_to_planes) {
    const cv::Mat& segmentation_mask = keyfrm->get_segmentation_mask();
    const auto lms = keyfrm->get_valid_landmarks();
    if (segmentation_mask.empty() || segmentation_mask.type() != CV_8UC3) {
        return false;
    }
    if (lms.empty() && (is_monocular_ || keyfrm->depth_map_.empty())) {
        return false;
    }

    for (const auto& lm : lms) {
        if (!lm || lm->will_be_erased() || lm->get_owning_plane()) {
            continue;
        }
        const int kpt_id = lm->get_index_in_keyframe(keyfrm);
        if (kpt_id < 0) {
            continue;
        }
        const auto& kpt = keyfrm->frm_obs_.undist_keypts_.at(static_cast<size_t>(kpt_id));
        const int x = static_cast<int>(kpt.pt.x);
        const int y = static_cast<int>(kpt.pt.y);
        if (x < 0 || y < 0 || x >= segmentation_mask.cols || y >= segmentation_mask.rows) {
            continue;
        }

        const auto center_color = segmentation_mask.at<cv::Vec3b>(y, x);
        const long pseudo_hash = color_hash(center_color);
        if (pseudo_hash == 0) {
            continue;
        }

        if (!options_.check_seg_3x3_window) {
            if (!color_to_planes.count(pseudo_hash)) {
                color_to_planes[pseudo_hash] = std::make_shared<data::landmark_plane>(keyfrm, map_db_);
                color_to_planes[pseudo_hash]->set_seg_label(pseudo_hash);
            }
            color_to_planes[pseudo_hash]->add_landmark(lm);
            continue;
        }

        bool consistent = true;
        for (int dy = -1; dy <= 1 && consistent; ++dy) {
            for (int dx = -1; dx <= 1 && consistent; ++dx) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                const int nx = x + dx;
                const int ny = y + dy;
                if (nx < 0 || ny < 0 || nx >= segmentation_mask.cols || ny >= segmentation_mask.rows) {
                    continue;
                }
                const long neighbor_hash = color_hash(segmentation_mask.at<cv::Vec3b>(ny, nx));
                if (neighbor_hash == 0 || neighbor_hash != pseudo_hash) {
                    consistent = false;
                }
            }
        }
        if (!consistent) {
            continue;
        }

        if (!color_to_planes.count(pseudo_hash)) {
            color_to_planes[pseudo_hash] = std::make_shared<data::landmark_plane>(keyfrm, map_db_);
            color_to_planes[pseudo_hash]->set_seg_label(pseudo_hash);
        }
        color_to_planes[pseudo_hash]->add_landmark(lm);
    }

    if (!is_monocular_) {
        for (auto& pair : color_to_planes) {
            if (pair.second && pair.second->num_landmarks() >= 2) {
                add_rgbd_depth_samples(keyfrm, pair.first, pair.second);
            }
        }
    }

    return !color_to_planes.empty();
}

double planar_mapping_module::estimate_plane_svd(const std::vector<std::shared_ptr<data::landmark>>& landmarks,
                                                 const std::vector<int>& indexes,
                                                 double& a, double& b, double& c, double& d) const {
    if (indexes.empty()) {
        return std::numeric_limits<double>::max();
    }
    Eigen::Matrix<double, 3, Eigen::Dynamic> X(3, static_cast<int>(indexes.size()));
    for (size_t i = 0; i < indexes.size(); ++i) {
        X.col(static_cast<int>(i)) = landmarks.at(static_cast<size_t>(indexes[i]))->get_pos_in_world();
    }

    const Vec3_t centroid = X.rowwise().mean();
    const Eigen::Matrix<double, 3, Eigen::Dynamic> centered = X.colwise() - centroid;

    const Eigen::JacobiSVD<Eigen::Matrix<double, 3, Eigen::Dynamic>> svd(centered, Eigen::ComputeFullU);
    const Vec3_t normal = svd.matrixU().col(2).normalized();
    d = -normal.dot(centroid);
    a = normal(0);
    b = normal(1);
    c = normal(2);

    const Eigen::VectorXd residual = (X.transpose() * normal).array() + d;
    return std::abs(residual.norm() / static_cast<double>(indexes.size()));
}

bool planar_mapping_module::estimate_plane_sequential_ransac(const std::shared_ptr<data::landmark_plane>& plane) {
    const auto points = collect_plane_points(plane);
    if (points.size() < static_cast<size_t>(options_.points_per_ransac)) {
        return false;
    }

    double best_error = std::numeric_limits<double>::max();
    std::vector<int> best_inliers;
    double a_best = 0, b_best = 0, c_best = 0, d_best = 0;
    bool best_found = false;

    std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<int> dist(0, static_cast<int>(points.size()) - 1);

    for (unsigned int iter = 0; iter < static_cast<unsigned int>(options_.ransac_iterations); ++iter) {
        std::vector<int> sample;
        while (sample.size() < static_cast<size_t>(options_.points_per_ransac)) {
            sample.push_back(dist(rng));
        }

        double a = 0, b = 0, c = 0, d = 0;
        const double residual = fit_plane_svd(points, sample, a, b, c, d);
        plane->set_equation(a, b, c, d);

        std::vector<int> inliers;
        for (size_t j = 0; j < points.size(); ++j) {
            if (plane->calculate_distance(points[j]) < planar_distance_thresh_) {
                inliers.push_back(static_cast<int>(j));
            }
        }

        const double inlier_ratio = static_cast<double>(inliers.size()) / static_cast<double>(points.size());
        if (inlier_ratio > options_.inliers_ratio_thr &&
            inliers.size() >= static_cast<size_t>(options_.points_per_ransac)) {
            const double error = fit_plane_svd(points, inliers, a, b, c, d);
            if (error < best_error) {
                best_error = error;
                a_best = a;
                b_best = b;
                c_best = c;
                d_best = d;
                best_inliers = std::move(inliers);
                best_found = true;
                if (error < final_error_thresh_) {
                    break;
                }
            }
        } else if (residual < best_error) {
            best_error = residual;
            a_best = a;
            b_best = b;
            c_best = c;
            d_best = d;
        }
    }

    if (!best_found || best_error > final_error_thresh_) {
        return false;
    }

    plane->set_equation(a_best, b_best, c_best, d_best);
    plane->set_best_error(best_error);

    const auto lms = plane->get_landmarks();
    std::vector<std::shared_ptr<data::landmark>> inlier_lms;
    inlier_lms.reserve(lms.size());
    for (const auto& lm : lms) {
        if (!lm || lm->will_be_erased()) {
            continue;
        }
        if (plane->calculate_distance(lm->get_pos_in_world()) < planar_distance_thresh_) {
            inlier_lms.push_back(lm);
        }
    }

    plane->remove_landmarks_ownership();
    plane->set_landmarks(inlier_lms);
    plane->clear_fit_samples();
    plane->set_landmarks_ownership();
    return true;
}

bool planar_mapping_module::create_new_planes(
    std::unordered_map<long, std::shared_ptr<data::landmark_plane>>& color_to_planes) {
    bool has_new_planes = false;
    for (auto& pair : color_to_planes) {
        auto& plane = pair.second;
        if (!plane || count_plane_points(plane) < static_cast<size_t>(options_.min_points_before_ransac)) {
            plane->remove_landmarks_ownership();
            continue;
        }
        const bool estimated = options_.use_graph_cut_ransac ? estimate_plane_sequential_graph_cut_ransac(plane)
                                                             : estimate_plane_sequential_ransac(plane);
        if (estimated && plane->num_landmarks() >= 4) {
            plane->set_valid();
            plane->id_ = map_db_->next_landmark_plane_id_++;
            map_db_->add_landmark_plane(plane);
            has_new_planes = true;
        } else {
            plane->remove_landmarks_ownership();
        }
    }
    color_to_planes.clear();
    return has_new_planes;
}

void planar_mapping_module::refinement() {
    std::lock_guard<std::mutex> lock(mtx_plane_);

    if (map_db_->get_num_landmark_planes() < 2) {
        return;
    }

    offset_delta_threshold_ = options_.offset_delta_factor * planar_distance_thresh_;
    merge_planes();
    refine_planes();
    if (options_.refine_landmark_positions) {
        refine_points();
    }
}

bool planar_mapping_module::merge_planes() {
    auto all_planes = map_db_->get_all_landmark_planes();
    if (all_planes.size() < 2) {
        return false;
    }

    std::vector<std::shared_ptr<data::landmark_plane>> planes_will_be_removed;
    bool was_merged = false;

    for (size_t i = 0; i < all_planes.size(); ++i) {
        if (!all_planes[i] || !all_planes[i]->is_valid()) {
            continue;
        }

        for (size_t j = i + 1; j < all_planes.size(); ++j) {
            if (!all_planes[j] || !all_planes[j]->is_valid()) {
                continue;
            }

            const Vec3_t parent_normal = all_planes[i]->get_normal();
            const Vec3_t candidate_normal = all_planes[j]->get_normal();
            const double parent_normalizer = 1.0 / all_planes[i]->get_normal_norm();
            const double candidate_normalizer = 1.0 / all_planes[j]->get_normal_norm();
            const double parent_offset = all_planes[i]->get_offset();
            const double candidate_offset = all_planes[j]->get_offset();

            const double offset_delta = parent_offset * parent_normalizer - candidate_offset * candidate_normalizer;
            const double normalized_dot_product =
                parent_normal.dot(candidate_normal) * (parent_normalizer * candidate_normalizer);

            if (std::fabs(offset_delta) > offset_delta_threshold_ &&
                std::fabs(normalized_dot_product) < options_.dot_product_threshold) {
                continue;
            }
            if (std::fabs(offset_delta) < offset_delta_threshold_ &&
                std::fabs(normalized_dot_product) > options_.dot_product_threshold) {
                if (all_planes[i]->num_landmarks() > all_planes[j]->num_landmarks()) {
                    all_planes[i]->merge(all_planes[j]);
                    planes_will_be_removed.push_back(all_planes[j]);
                    update_plane_via_ransac(all_planes[i]);
                    all_planes[i]->set_need_refinement();
                } else {
                    all_planes[j]->merge(all_planes[i]);
                    planes_will_be_removed.push_back(all_planes[i]);
                    update_plane_via_ransac(all_planes[j]);
                    all_planes[j]->set_need_refinement();
                }
                was_merged = true;
            }
        }
    }

    if (was_merged) {
        for (const auto& plane : planes_will_be_removed) {
            if (plane) {
                map_db_->erase_landmark_plane(plane);
            }
        }
    }
    return was_merged;
}

bool planar_mapping_module::refine_planes() {
    bool was_changed = false;
    std::vector<std::shared_ptr<data::landmark_plane>> planes_will_be_erased;

    for (const auto& plane : map_db_->get_all_landmark_planes()) {
        if (!plane) {
            continue;
        }
        if (!plane->is_valid()) {
            planes_will_be_erased.push_back(plane);
            continue;
        }
        if (plane->need_refinement()) {
            if (update_plane_via_ransac(plane)) {
                plane->set_refinement_is_done();
                was_changed = true;
            }
        } else if (plane->num_landmarks() < static_cast<unsigned int>(2 * options_.points_per_ransac)) {
            planes_will_be_erased.push_back(plane);
        }
    }

    for (const auto& plane : planes_will_be_erased) {
        if (!plane) {
            continue;
        }
        plane->remove_landmarks_ownership();
        map_db_->erase_landmark_plane(plane);
    }
    return was_changed;
}

bool planar_mapping_module::refine_points() {
    bool was_changed = false;

    for (const auto& plane : map_db_->get_all_landmark_planes()) {
        if (!plane || !plane->is_valid() || plane->need_refinement() ||
            plane->num_landmarks() < static_cast<unsigned int>(options_.points_per_ransac)) {
            continue;
        }

        const auto lms = plane->get_landmarks();
        for (const auto& lm : lms) {
            if (!lm || lm->will_be_erased() || !lm->get_owning_plane()) {
                continue;
            }
            Vec3_t pos_w = lm->get_pos_in_world();
            const double dist_old = plane->calculate_distance(pos_w);
            if (dist_old <= 0.0) {
                continue;
            }
            const Vec3_t n = plane->get_normal().normalized();
            pos_w -= n * dist_old;
            const double dist_new = plane->calculate_distance(pos_w);
            if (dist_new < planar_distance_thresh_ && dist_new < dist_old) {
                lm->set_pos_in_world(pos_w);
                was_changed = true;
            }
        }
    }
    return was_changed;
}

bool planar_mapping_module::update_plane_via_ransac(const std::shared_ptr<data::landmark_plane>& plane) {
    const auto lms = plane->get_landmarks();
    if (lms.size() < static_cast<size_t>(options_.points_per_ransac)) {
        plane->set_invalid();
        return false;
    }

    size_t valid_lm_count = 0;
    for (const auto& lm : lms) {
        if (lm && !lm->will_be_erased() && lm->get_owning_plane()) {
            ++valid_lm_count;
        }
    }
    if (valid_lm_count < static_cast<size_t>(options_.points_per_ransac)) {
        plane->set_invalid();
        return false;
    }

    double best_error = plane->get_best_error();
    std::vector<int> best_inliers;
    double a_best = 0, b_best = 0, c_best = 0, d_best = 0;
    bool best_found = false;

    std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<int> dist(0, static_cast<int>(lms.size()) - 1);

    for (unsigned int iter = 0; iter < static_cast<unsigned int>(options_.ransac_iterations); ++iter) {
        std::vector<int> indexes;
        const size_t target = std::max(static_cast<size_t>(options_.points_per_ransac),
                                       static_cast<size_t>(lms.size() * 0.8));
        while (indexes.size() < target) {
            const int index = dist(rng);
            if (!lms.at(static_cast<size_t>(index))->get_owning_plane() || lms.at(static_cast<size_t>(index))->will_be_erased()) {
                continue;
            }
            indexes.push_back(index);
        }
        if (indexes.empty()) {
            break;
        }

        double a = 0, b = 0, c = 0, d = 0;
        const double residual = estimate_plane_svd(lms, indexes, a, b, c, d);
        if (residual < best_error) {
            best_error = residual;
        }
        a_best = a;
        b_best = b;
        c_best = c;
        d_best = d;
        plane->set_equation(a_best, b_best, c_best, d_best);
        plane->set_best_error(residual);

        std::vector<int> inliers;
        for (size_t j = 0; j < lms.size(); ++j) {
            if (lms[j]->will_be_erased()) {
                continue;
            }
            if (plane->calculate_distance(lms[j]->get_pos_in_world()) < planar_distance_thresh_) {
                inliers.push_back(static_cast<int>(j));
            }
        }

        if (inliers.size() >= static_cast<size_t>(options_.points_per_ransac)) {
            const double error = estimate_plane_svd(lms, inliers, a, b, c, d);
            if (error < best_error) {
                best_error = error;
                a_best = a;
                b_best = b;
                c_best = c;
                d_best = d;
                best_inliers = std::move(inliers);
                best_found = true;
            }
        }
    }

    if (!best_found || best_error > final_error_thresh_) {
        plane->set_need_refinement();
        return false;
    }

    plane->set_equation(a_best, b_best, c_best, d_best);
    plane->set_best_error(best_error);

    std::vector<std::shared_ptr<data::landmark>> inlier_lms;
    inlier_lms.reserve(best_inliers.size());
    for (const int idx : best_inliers) {
        const auto& lm = lms.at(static_cast<size_t>(idx));
        if (!lm->will_be_erased() && plane->calculate_distance(lm->get_pos_in_world()) < planar_distance_thresh_) {
            inlier_lms.push_back(lm);
        }
    }
    if (inlier_lms.size() < static_cast<size_t>(options_.points_per_ransac)) {
        plane->set_invalid();
        return false;
    }

    plane->remove_landmarks_ownership();
    plane->set_landmarks(inlier_lms);
    plane->set_landmarks_ownership();
    return true;
}

bool planar_mapping_module::estimate_plane_sequential_graph_cut_ransac(
    const std::shared_ptr<data::landmark_plane>& plane) {
    const auto lms = plane->get_landmarks();
    const auto& fit_samples = plane->get_fit_samples();
    const size_t total_pts = lms.size() + fit_samples.size();
    if (total_pts < static_cast<size_t>(options_.points_per_ransac)) {
        return false;
    }

    cv::Mat points(0, 3, CV_64F);
    cv::Mat point(1, 3, CV_64F);
    std::vector<std::shared_ptr<data::landmark>> valid_lms;
    valid_lms.reserve(lms.size());
    for (const auto& lm : lms) {
        if (!lm || lm->will_be_erased()) {
            continue;
        }
        const auto pos_w = lm->get_pos_in_world();
        point.at<double>(0) = pos_w(0);
        point.at<double>(1) = pos_w(1);
        point.at<double>(2) = pos_w(2);
        points.push_back(point);
        valid_lms.push_back(lm);
    }
    for (const auto& pos_w : fit_samples) {
        point.at<double>(0) = pos_w(0);
        point.at<double>(1) = pos_w(1);
        point.at<double>(2) = pos_w(2);
        points.push_back(point);
    }
    if (points.rows < static_cast<int>(options_.points_per_ransac)) {
        return false;
    }

    double inlier_outlier_threshold = planar_distance_thresh_;
    double sphere_radius = options_.gc_sphere_radius;
    if (options_.gc_adaptive_number != 0) {
        inlier_outlier_threshold = planar_distance_thresh_;
        sphere_radius = options_.gc_adaptive_number * final_error_thresh_;
    }

    gcransac::utils::Default3DPlaneEstimator estimator;
    gcransac::Plane3D model;
    gcransac::neighborhood::FlannNeighborhoodGraph neighborhood(&points, sphere_radius);
    gcransac::sampler::UniformSampler main_sampler(&points);
    gcransac::sampler::UniformSampler local_optimization_sampler(&points);

    if (!main_sampler.isInitialized() || !local_optimization_sampler.isInitialized()) {
        AWARN << "planar_mapping_module: GC-RANSAC sampler init failed";
        return false;
    }

    gcransac::preemption::SPRTPreemptiveVerfication<gcransac::utils::Default3DPlaneEstimator> preemptive_verification(
        points, estimator, options_.gc_minimum_inlier_ratio_for_sprt);

    gcransac::GCRANSAC<gcransac::utils::Default3DPlaneEstimator,
                       gcransac::neighborhood::FlannNeighborhoodGraph,
                       gcransac::MSACScoringFunction<gcransac::utils::Default3DPlaneEstimator>,
                       gcransac::preemption::SPRTPreemptiveVerfication<gcransac::utils::Default3DPlaneEstimator>>
        gcransac;
    gcransac.setFPS(options_.gc_fps);
    gcransac.settings.threshold = inlier_outlier_threshold;
    gcransac.settings.spatial_coherence_weight = options_.spatial_coherence_weight;
    gcransac.settings.confidence = options_.gc_confidence;
    gcransac.settings.max_iteration_number = 5000;
    gcransac.settings.min_iteration_number = 20;

    gcransac.run(points, estimator, &main_sampler, &local_optimization_sampler, &neighborhood, model,
                 preemptive_verification);

    const auto& statistics = gcransac.getRansacStatistics();
    plane->set_equation(model.descriptor(0, 0), model.descriptor(0, 1), model.descriptor(0, 2), model.descriptor(0, 3));

    const double inlier_ratio =
        static_cast<double>(statistics.inliers.size()) / static_cast<double>(points.rows);
    if (inlier_ratio < options_.inliers_ratio_thr &&
        statistics.inliers.size() < static_cast<size_t>(options_.points_per_ransac)) {
        return false;
    }

    std::vector<int> inliers_list;
    inliers_list.reserve(statistics.inliers.size());
    std::vector<std::shared_ptr<data::landmark>> plane_best_inlier_map_points;
    plane_best_inlier_map_points.reserve(statistics.inliers.size());
    std::vector<Vec3_t> all_points;
    all_points.reserve(static_cast<size_t>(points.rows));
    for (int r = 0; r < points.rows; ++r) {
        all_points.emplace_back(points.at<double>(r, 0), points.at<double>(r, 1), points.at<double>(r, 2));
    }
    for (const auto inlier_idx : statistics.inliers) {
        if (inlier_idx < 0 || inlier_idx >= points.rows) {
            continue;
        }
        inliers_list.push_back(static_cast<int>(inlier_idx));
        if (static_cast<size_t>(inlier_idx) < valid_lms.size()) {
            plane_best_inlier_map_points.push_back(valid_lms.at(static_cast<size_t>(inlier_idx)));
        }
    }
    if (inliers_list.size() < static_cast<size_t>(options_.points_per_ransac)) {
        return false;
    }

    double a = 0, b = 0, c = 0, d = 0;
    const double error = fit_plane_svd(all_points, inliers_list, a, b, c, d);
    if (!std::isfinite(error) || error > final_error_thresh_) {
        return false;
    }
    plane->set_equation(a, b, c, d);
    plane->set_best_error(error);

    plane->remove_landmarks_ownership();
    plane->set_landmarks(plane_best_inlier_map_points);
    plane->clear_fit_samples();
    plane->set_landmarks_ownership();
    return true;
}

}  // namespace autonomy::localization::atlas::plp
