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
 * @file frame.hpp
 * @brief Visual frame with pose and patch features (FAST-LIVO2 Frame).
 */

#ifndef ARGUS_ODOMETRY_FRAME_HPP_
#define ARGUS_ODOMETRY_FRAME_HPP_

#include <list>
#include <memory>
#include <vector>

#include <opencv2/core.hpp>

#include "argus/common/camera/abstract_camera.hpp"
#include "argus/common/math/se3.hpp"
#include "argus/map/visual_point.hpp"

namespace argus {
namespace odometry {

using Features = std::list<map::Feature*>;
using ImagePyramid = std::vector<cv::Mat>;

/**
 * @brief Camera frame holding image, pose, and sparse features.
 */
class Frame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Frame(const Frame&) = delete;
    Frame& operator=(const Frame&) = delete;

    static int frame_counter_; /**< @brief Global id source for new frames. */
    int id_ = 0;               /**< @brief Unique frame id. */
    camera::AbstractCamera* cam_ =
        nullptr;            /**< @brief Projection model (not owned). */
    math::SE3 T_f_w_;       /**< @brief World to frame (camera) transform. */
    math::SE3 T_f_w_prior_; /**< @brief Prior pose before optimization. */
    cv::Mat image_;           /**< @brief Grayscale or RGB image buffer. */
    Features features_;          /**< @brief Observed map features in this frame. */

    /**
     * @brief Construct a frame and build the image pyramid.
     * @param cam Camera model used for projection (must outlive frame).
     * @param img Input image (copied internally).
     */
    Frame(camera::AbstractCamera* cam, const cv::Mat& img);

    /** @brief Release feature references owned by this frame. */
    ~Frame();

    /**
     * @brief Rebuild pyramid and reset features from a new image.
     * @param img Replacement image.
     */
    void InitFrame(const cv::Mat& img);

    /**
     * @brief Reuse this frame object for a new image (new id, cleared features).
     * @param cam Camera model (must outlive frame).
     * @param img Grayscale image matching camera size.
     */
    void Reset(camera::AbstractCamera* cam, const cv::Mat& img);

    /**
     * @brief Number of feature observations.
     * @return Feature list size.
     */
    inline std::size_t NumObservations() const {
        return features_.size();
    }

    /**
     * @brief Project a world point to pixel coordinates.
     * @param xyz_w Point in world frame.
     * @return Pixel coordinates.
     */
    inline Eigen::Vector2d w2c(const Eigen::Vector3d& xyz_w) const {
        return cam_->world2cam(T_f_w_ * xyz_w);
    }

    /**
     * @brief Project using prior pose `T_f_w_prior_`.
     * @param xyz_w Point in world frame.
     * @return Pixel coordinates.
     */
    inline Eigen::Vector2d w2c_prior(const Eigen::Vector3d& xyz_w) const {
        return cam_->world2cam(T_f_w_prior_ * xyz_w);
    }

    /**
     * @brief Back-project a pixel to a unit bearing in frame coordinates.
     * @param px Pixel coordinates.
     * @return Unit direction in frame frame.
     */
    inline Eigen::Vector3d c2f(const Eigen::Vector2d& px) const {
        return cam_->cam2world(px[0], px[1]);
    }

    /**
     * @brief Back-project pixel `(x, y)` to a unit bearing.
     * @param x Horizontal pixel coordinate.
     * @param y Vertical pixel coordinate.
     * @return Unit direction in frame frame.
     */
    inline Eigen::Vector3d c2f(const double x, const double y) const {
        return cam_->cam2world(x, y);
    }

    /**
     * @brief Transform world point into frame (camera) coordinates.
     * @param xyz_w Point in world frame.
     * @return Point in frame frame.
     */
    inline Eigen::Vector3d w2f(const Eigen::Vector3d& xyz_w) const {
        return T_f_w_ * xyz_w;
    }

    /**
     * @brief Transform frame point into world coordinates.
     * @param f Point in frame frame.
     * @return Point in world frame.
     */
    inline Eigen::Vector3d f2w(const Eigen::Vector3d& f) const {
        return T_f_w_.inverse() * f;
    }

    /**
     * @brief Project a frame-frame 3D point to pixels.
     * @param f Point in frame (camera) coordinates.
     * @return Pixel coordinates.
     */
    inline Eigen::Vector2d f2c(const Eigen::Vector3d& f) const {
        return cam_->world2cam(f);
    }

    /**
     * @brief Camera center in world frame.
     * @return Translation of `T_w_f`.
     */
    inline Eigen::Vector3d pos() const {
        return T_f_w_.inverse().translation();
    }
};

using FramePtr = std::unique_ptr<Frame>;

namespace frame_utils {

/**
 * @brief Build a Gaussian image pyramid from level 0.
 * @param img_level_0 Full-resolution grayscale image.
 * @param n_levels Number of pyramid levels (including level 0).
 * @param[out] pyr Output pyramid; resized to `n_levels`.
 */
void CreateImgPyramid(const cv::Mat& img_level_0, int n_levels, ImagePyramid& pyr);

/** @brief Legacy alias for `CreateImgPyramid`. */
inline void createImgPyramid(const cv::Mat& img_level_0, int n_levels,
                             ImagePyramid& pyr) {
    CreateImgPyramid(img_level_0, n_levels, pyr);
}
}  // namespace frame_utils

}  // namespace odometry
}  // namespace argus

#endif  // ARGUS_ODOMETRY_FRAME_HPP_
