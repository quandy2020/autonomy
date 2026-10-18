/*
 * Copyright 2026 The Openbot Authors
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

/**
 * @file semantic_dynamic_filter.hpp
 * @brief Image semantic mask → LiDAR keep flags (optional ONNX Runtime).
 *
 * Backends:
 *   - onnx: run segmentation ONNX (ARGUS_HAS_ONNXRUNTIME); else pass-through
 *   - mask_file: load HxW uint8 keep map (0 = dynamic) for offline / KITTI
 *
 * FilterVisual updates the keep mask; FilterLidar projects body/LiDAR points
 * with Rcl/Pcl + pinhole intrinsics and samples that mask.
 */

#ifndef ARGUS_FRONTEND_SEMANTIC_DYNAMIC_FILTER_HPP_
#define ARGUS_FRONTEND_SEMANTIC_DYNAMIC_FILTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "argus/frontend/dynamic_filter_base.hpp"

namespace argus {
namespace frontend {

enum class SemanticBackend {
    kOnnx = 0,
    kMaskFile = 1,
};

inline SemanticBackend ParseSemanticBackend(const std::string& name) {
    if (name == "mask_file" || name == "mask") {
        return SemanticBackend::kMaskFile;
    }
    return SemanticBackend::kOnnx;
}

struct SemanticDynamicFilterConfig {
    std::string model_path;
    /** @brief Optional HxW uint8 PNG/PGM; 0 = reject (dynamic). */
    std::string mask_path;
    SemanticBackend backend = SemanticBackend::kOnnx;
    double mask_threshold = 0.5;
    int input_width = 640;
    int input_height = 640;
    /** @brief Class ids treated as dynamic (COCO person=0; Cityscapes 11–18).
     */
    std::vector<int> dynamic_class_ids = {0};
    std::vector<float> mean = {0.485f, 0.456f, 0.406f};
    std::vector<float> std = {0.229f, 0.224f, 0.225f};
    /** @brief Pinhole for LiDAR→image (set from camera calib). */
    double fx = 718.856;
    double fy = 718.856;
    double cx = 607.1928;
    double cy = 185.2157;
    int image_width = 1242;
    int image_height = 375;
    /** @brief LiDAR → camera, row-major R (9) + t (3). */
    Eigen::Matrix3d R_cl = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_cl = Eigen::Vector3d::Zero();
};

/**
 * @brief Semantic dynamic filter.
 *
 * Without ONNX Runtime / model, `backend=onnx` is pass-through
 * (`model_loaded()==false`). `backend=mask_file` or `SetKeepMask` still
 * enables LiDAR projection gating for KITTI / unit tests.
 */
class SemanticDynamicFilter : public DynamicFilterBase
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(SemanticDynamicFilter);

    /**
     * @brief Construct and optionally load ONNX / mask file.
     * @param config Backend, paths, intrinsics, and extrinsics.
     */
    explicit SemanticDynamicFilter(SemanticDynamicFilterConfig config = {});
    ~SemanticDynamicFilter() override;

    SemanticDynamicFilter(const SemanticDynamicFilter&) = delete;
    SemanticDynamicFilter& operator=(const SemanticDynamicFilter&) = delete;

    /**
     * @brief Project LiDAR into the current keep mask.
     * @param cloud Body-frame points.
     * @param state Unused.
     * @return Per-point keep mask.
     */
    Mask FilterLidar(const sensor::PointCloudXYZI& cloud,
                     const StatesGroup& state) override;

    /**
     * @brief Run ONNX or refresh mask_file; updates internal keep map.
     * @param image BGR camera frame.
     * @param state Unused.
     * @return Per-pixel keep mask.
     */
    Mask FilterVisual(const cv::Mat& image, const StatesGroup& state) override;

    /** @return True if an ONNX session is ready. */
    bool model_loaded() const {
        return model_loaded_;
    }
    /** @return True if a keep mask is currently held. */
    bool has_keep_mask() const {
        return !keep_mask_.empty();
    }
    /** @return Current HxW CV_8UC1 keep map (1 = static). */
    const cv::Mat& keep_mask() const {
        return keep_mask_;
    }

    /**
     * @brief Set LiDAR→camera extrinsic used by FilterLidar.
     * @param R_cl Rotation LiDAR→camera.
     * @param t_cl Translation LiDAR→camera (m).
     */
    void SetLidarCameraExtrinsic(const Eigen::Matrix3d& R_cl,
                                 const Eigen::Vector3d& t_cl);

    /**
     * @brief Set pinhole intrinsics for projection.
     * @param fx Focal length x (px).
     * @param fy Focal length y (px).
     * @param cx Principal point x (px).
     * @param cy Principal point y (px).
     * @param width Image width (px).
     * @param height Image height (px).
     */
    void SetCameraIntrinsics(double fx, double fy, double cx, double cy,
                             int width, int height);

    /**
     * @brief Inject HxW CV_8UC1 keep map (1=static). Used by tests / mask_file.
     * @param keep_mask Grayscale keep map; cloned internally.
     */
    void SetKeepMask(const cv::Mat& keep_mask);

    /**
     * @brief Reload mask from disk (`backend=mask_file`).
     * @param path Override path; empty uses `config.mask_path`.
     * @return True if a non-empty mask was loaded.
     */
    bool LoadMaskFile(const std::string& path = {});

private:
    bool InitOnnx();
    bool RunOnnx(const cv::Mat& bgr, cv::Mat* keep_out);
    Mask ProjectCloud(const sensor::PointCloudXYZI& cloud) const;

    SemanticDynamicFilterConfig config_;
    bool model_loaded_ = false;
    cv::Mat keep_mask_;  // CV_8UC1, 1=keep

    struct OnnxSession;
    std::unique_ptr<OnnxSession> onnx_;
};

}  // namespace frontend
}  // namespace argus

#endif  // ARGUS_FRONTEND_SEMANTIC_DYNAMIC_FILTER_HPP_
