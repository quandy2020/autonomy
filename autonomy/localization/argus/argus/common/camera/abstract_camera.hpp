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
 * @file abstract_camera.hpp
 * @brief Abstract camera model (ROS-free port of rpg_vikit AbstractCamera).
 *
 * @see https://github.com/xuankuzcr/rpg_vikit
 */

#ifndef ARGUS_COMMON_CAMERA_ABSTRACT_CAMERA_HPP_
#define ARGUS_COMMON_CAMERA_ABSTRACT_CAMERA_HPP_

#include "Eigen/Core"

#include "argus/common/macros.hpp"

namespace argus {
namespace camera {

/**
 * @brief Base class for geometric camera models used by Argus / LIVO.
 *
 * `cam2world` returns a **unit bearing** in the camera frame.
 * `world2cam(Vector3d)` projects a 3D camera-frame point to pixels.
 * `world2cam(Vector2d)` projects a point on the unit plane (z=1).
 */
class AbstractCamera
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(AbstractCamera);

    AbstractCamera() = default;

    /**
     * @brief Construct with image geometry and optional pyramid scale.
     * @param width Image width (pixels).
     * @param height Image height (pixels).
     * @param scale Resize factor applied to size and intrinsics.
     */
    AbstractCamera(int width, int height, double scale)
        : width_(width), height_(height), scale_(scale) {}

    virtual ~AbstractCamera() = default;

    /** @param x Pixel u coordinate. @param y Pixel v coordinate. @return Unit
     * bearing in camera frame. */
    virtual Eigen::Vector3d cam2world(const double& x,
                                      const double& y) const = 0;
    /** @param px Pixel coordinates. @return Unit bearing in camera frame. */
    virtual Eigen::Vector3d cam2world(const Eigen::Vector2d& px) const = 0;
    /** @param xyz_c 3D point in camera frame. @return Pixel coordinates. */
    virtual Eigen::Vector2d world2cam(const Eigen::Vector3d& xyz_c) const = 0;
    /** @param uv Point on unit plane (z = 1). @return Pixel coordinates. */
    virtual Eigen::Vector2d world2cam(const Eigen::Vector2d& uv) const = 0;

    /** @return Visual reprojection error scale (model-specific). */
    virtual double errorMultiplier2() const = 0;
    /** @return Visual reprojection error scale for 2D residuals. */
    virtual double errorMultiplier() const = 0;

    /** @return Focal length u (pixels). */
    virtual double fx() const = 0;
    /** @return Focal length v (pixels). */
    virtual double fy() const = 0;
    /** @return Principal point u (pixels). */
    virtual double cx() const = 0;
    /** @return Principal point v (pixels). */
    virtual double cy() const = 0;

    /** @return Image width (pixels). */
    int width() const {
        return width_;
    }
    /** @return Image height (pixels). */
    int height() const {
        return height_;
    }
    /** @return Pyramid / resize scale factor. */
    double scale() const {
        return scale_;
    }

    /**
     * @brief True if pixel lies inside the image with optional border margin.
     * @param obs Integer pixel coordinates.
     * @param boundary Margin in pixels from each edge (default 0).
     * @return Whether the observation is in bounds.
     */
    bool isInFrame(const Eigen::Vector2i& obs, int boundary = 0) const {
        return obs[0] >= boundary && obs[0] < width() - boundary &&
               obs[1] >= boundary && obs[1] < height() - boundary;
    }

    /**
     * @brief In-bounds test at a pyramid level (dimensions divided by 2^level).
     * @param obs Integer pixel coordinates at the given level.
     * @param boundary Margin in pixels.
     * @param level Pyramid level (0 = full resolution).
     * @return Whether the observation is in bounds.
     */
    bool isInFrame(const Eigen::Vector2i& obs, int boundary, int level) const {
        return obs[0] >= boundary &&
               obs[0] < width() / (1 << level) - boundary &&
               obs[1] >= boundary &&
               obs[1] < height() / (1 << level) - boundary;
    }

protected:
    int width_ = 0;
    int height_ = 0;
    double scale_ = 1.0;
};

/**
 * @brief Project 3D camera point onto the unit plane (x/z, y/z).
 * @param v 3D point with non-zero z.
 * @return 2D coordinates on z = 1 plane.
 */
inline Eigen::Vector2d Project2d(const Eigen::Vector3d& v) {
    return v.head<2>() / v[2];
}

/**
 * @brief Lift a unit-plane point to homogeneous 3D (x, y, 1).
 * @param v 2D coordinates on z = 1 plane.
 * @return Homogeneous 3D direction.
 */
inline Eigen::Vector3d Unproject2d(const Eigen::Vector2d& v) {
    return Eigen::Vector3d(v[0], v[1], 1.0);
}

}  // namespace camera
}  // namespace argus

#endif  // ARGUS_COMMON_CAMERA_ABSTRACT_CAMERA_HPP_
