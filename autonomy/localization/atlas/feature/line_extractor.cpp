/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/localization/atlas/feature/line_extractor.hpp"

#include "autonomy/localization/atlas/camera/perspective.hpp"
#include "autonomy/localization/atlas/feature/line_descriptor/line_descriptor_custom.hpp"

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

namespace autonomy::localization::atlas::feature {

line_extractor::line_extractor(camera::base* camera, plp::Options options)
    : camera_(camera), options_(std::move(options)) {}

void line_extractor::ensure_undistort_maps(const cv::Mat& img) const {
    if (maps_ready_) {
        return;
    }
    auto* cam = dynamic_cast<camera::perspective*>(camera_);
    if (cam == nullptr) {
        maps_ready_ = true;
        return;
    }
    const float fx = static_cast<float>(cam->fx_);
    const float fy = static_cast<float>(cam->fy_);
    const float cx = static_cast<float>(cam->cx_);
    const float cy = static_cast<float>(cam->cy_);
    cv::Mat map_x = cv::Mat::zeros(img.rows, img.cols, CV_32F);
    cv::Mat map_y = cv::Mat::zeros(img.rows, img.cols, CV_32F);
    Eigen::Matrix3d k_rect;
    k_rect << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    const Eigen::Matrix3d k_inv = k_rect.inverse();
    for (int v = 0; v < img.rows; ++v) {
        for (int u = 0; u < img.cols; ++u) {
            const Eigen::Vector3d xo(u, v, 1.0);
            const Eigen::Vector3d uo = k_inv * xo;
            const double z_inv = 1.0 / uo(2);
            map_x.at<float>(v, u) = static_cast<float>(fx * uo(0) * z_inv + cx);
            map_y.at<float>(v, u) = static_cast<float>(fy * uo(1) * z_inv + cy);
        }
    }
    cv::convertMaps(map_x, map_y, undist_map1_, undist_map2_, CV_32FC1, false);
    cv::eigen2cv(k_rect, k_rect_);
    maps_ready_ = true;
}

void line_extractor::extract(const cv::Mat& gray,
                             data::line_frame_observation* out) const {
    if (out == nullptr || gray.empty()) {
        return;
    }
    out->keylines.clear();
    out->lbd_descriptors.release();
    out->line_functions.clear();

    cv::Mat work = gray;
    const auto* cam = dynamic_cast<camera::perspective*>(camera_);
    const bool needs_undistort =
        cam != nullptr &&
        (std::abs(cam->k1_) > 1e-8 || std::abs(cam->k2_) > 1e-8 || std::abs(cam->p1_) > 1e-8 ||
         std::abs(cam->p2_) > 1e-8 || std::abs(cam->k3_) > 1e-8);
    if (needs_undistort) {
        ensure_undistort_maps(gray);
        if (!undist_map1_.empty()) {
            cv::remap(gray, work, undist_map1_, undist_map2_, cv::INTER_LINEAR);
        }
    }
    if (options_.equalize_histogram) {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(work, work);
    }

    cv::Ptr<cv::line_descriptor::LSDDetectorC> lsd =
        cv::line_descriptor::LSDDetectorC::createLSDDetectorC();
    cv::line_descriptor::LSDDetectorC::LSDOptions opts;
    opts.refine = 1;
    opts.scale = 0.5f;
    opts.sigma_scale = 0.6f;
    opts.quant = 2.0f;
    opts.ang_th = 22.5f;
    opts.log_eps = 1.0f;
    opts.density_th = 0.6f;
    opts.n_bins = 1024;
    opts.min_length = options_.min_line_length_ratio *
                      static_cast<float>(std::min(work.cols, work.rows));

    std::vector<cv::line_descriptor::KeyLine> lsd_all;
    lsd->detect(work, lsd_all, scale_factor_, num_levels_, opts);

    cv::Ptr<cv::line_descriptor::BinaryDescriptor> binary =
        cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
    cv::Mat lbd_all;
    if (!lsd_all.empty()) {
        binary->compute(work, lsd_all, lbd_all);
    }

    std::vector<cv::line_descriptor::KeyLine> keylsd;
    cv::Mat keylbd;
    for (std::size_t i = 0; i < lsd_all.size(); ++i) {
        if (lsd_all[i].octave == 0 &&
            lsd_all[i].lineLength >= options_.min_line_pixels) {
            keylsd.push_back(lsd_all[i]);
            if (!lbd_all.empty()) {
                keylbd.push_back(lbd_all.row(static_cast<int>(i)));
            }
        }
    }
    out->keylines = std::move(keylsd);
    out->lbd_descriptors = std::move(keylbd);
    out->line_functions.reserve(out->keylines.size());
    for (const auto& kl : out->keylines) {
        Eigen::Vector3d sp(kl.startPointX, kl.startPointY, 1.0);
        Eigen::Vector3d ep(kl.endPointX, kl.endPointY, 1.0);
        Vec3_t fn = sp.cross(ep);
        const double n = std::sqrt(fn(0) * fn(0) + fn(1) * fn(1));
        if (n > 1e-8) {
            fn /= n;
        }
        out->line_functions.push_back(fn);
    }
}

}  // namespace autonomy::localization::atlas::feature
