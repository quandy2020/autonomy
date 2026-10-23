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
 * @file visual_feature.hpp
 * @brief Salient image patch feature (FAST-LIVO2 Feature).
 */

#ifndef ARGUS_MAP_VISUAL_FEATURE_HPP_
#define ARGUS_MAP_VISUAL_FEATURE_HPP_

#include <opencv2/core.hpp>

#include <Eigen/Core>

#include "argus/common/math/se3.hpp"

namespace argus {
namespace map {

class VisualPoint;

/** @brief Salient image patch tracked across frames. */
struct Feature {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum FeatureType { CORNER, EDGELET };

    int id_ = 0;                /**< @brief Feature identifier. */
    FeatureType type_ = CORNER; /**< @brief Corner or edgelet type. */
    cv::Mat image_;               /**< @brief Optional source image ROI. */
    Eigen::Vector2d px_ =
        Eigen::Vector2d::Zero(); /**< @brief Pixel location. */
    Eigen::Vector3d f_ =
        Eigen::Vector3d::Zero();   /**< @brief Unit bearing in camera frame. */
    int level_ = 0;                /**< @brief Pyramid level. */
    VisualPoint* point_ = nullptr; /**< @brief Back-pointer to map point. */
    Eigen::Vector2d grad_ =
        Eigen::Vector2d::Zero(); /**< @brief Image gradient. */
    math::SE3 T_f_w_;            /**< @brief Camera pose when observed. */
    float* patch_ = nullptr;     /**< @brief Owned patch buffer. */
    float score_ = 0;            /**< @brief Matching score. */
    float mean_ = 0;             /**< @brief Patch mean intensity. */
    double inv_expo_time_ = 0;   /**< @brief Inverse exposure time. */

    /**
     * @brief Construct a patch feature attached to a map point.
     * @param point Parent visual point.
     * @param patch Owned patch buffer (deleted in destructor).
     * @param px Pixel coordinates.
     * @param f Unit bearing in camera frame.
     * @param T_f_w Frame pose at observation time.
     * @param level Pyramid level.
     */
    Feature(VisualPoint* point, float* patch, const Eigen::Vector2d& px,
            const Eigen::Vector3d& f, const math::SE3& T_f_w, int level)
        : type_(CORNER),
          px_(px),
          f_(f),
          level_(level),
          point_(point),
          T_f_w_(T_f_w),
          patch_(patch),
          score_(0.f),
          mean_(0.f) {}

    /** @return Camera position in world frame for this observation. */
    Eigen::Vector3d pos() const {
        return T_f_w_.inverse().translation();
    }

    ~Feature() {
        delete[] patch_;
    }

    Feature(const Feature&) = delete;
    Feature& operator=(const Feature&) = delete;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_VISUAL_FEATURE_HPP_
