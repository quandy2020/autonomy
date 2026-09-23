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
 *
 * GeometricCamera interface (ORB-SLAM3 GeometricCamera + Kalibr/Basalt models).
 */

/**
 * @file
 * @brief Abstract geometric camera projection interface `GeometricCamera`.
 *
 * Covers common Kalibr / OpenCV / Basalt / ORB-SLAM3 models; concrete types live
 * in sensor/camera headers and are built by name via camera::CameraFactory.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SENSOR_GEOMETRIC_CAMERA_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SENSOR_GEOMETRIC_CAMERA_HPP_

#include <memory>
#include <string>
#include <vector>

#include "autonomy/localization/atlas/common/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {

/**
 * @class autonomy::localization::atlas::sensor::GeometricCamera
 * @brief Abstract base for camera projection / unprojection.
 *
 * Supported models: pinhole, radtan, kannala-brandt, fov, ucm, eucm,
 * double-sphere, equirectangular, radial-division.
 *
 * **Conventions**
 * - `Project`: camera-frame 3D point → pixel (u,v);
 * - `Unproject`: pixel → camera-frame ray or 3D point at given depth;
 * - Pixel origin at image top-left; u right, v down.
 *
 * Not thread-safe: instances are usually shared read-only; synchronize writers
 * of `set_image_size` / `id`.
 *
 * @code{.cpp}
 * auto cam = camera::CameraFactory::Create(intrinsics);
 * Vec2 uv = cam->Project(point_c);
 * Vec3 ray = cam->Unproject(uv, 1.0);
 * @endcode
 */
class GeometricCamera {
public:
    /**
     * @enum Type
     * @brief Concrete projection model enum.
     */
    enum class Type {
        kPinhole = 0,          ///< Ideal pinhole.
        kRadTan = 1,           ///< Brown–Conrady (OpenCV / Kalibr pinhole-radtan).
        kKannalaBrandt = 2,    ///< Equidistant fisheye (OpenCV fisheye / ORB KB8).
        kFov = 3,              ///< Devernay–Faugeras FOV.
        kUcm = 4,              ///< Mei unified / omni.
        kEucm = 5,             ///< Extended Unified (Kalibr/Basalt).
        kDoubleSphere = 6,     ///< Usenko double sphere.
        kEquirectangular = 7,  ///< 360° spherical cylindrical unwrap.
        kRadialDivision = 8,   ///< Fitzgibbon division model.
    };

    /**
     * @brief Virtual destructor for safe polymorphic deletion.
     */
    virtual ~GeometricCamera() = default;

    /**
     * @brief Project a camera-frame 3D point to pixels.
     * @param point_camera Point in the camera frame (meters); usually in FOV.
     * @return Pixel coordinates (u, v).
     */
    virtual Vec2 Project(const Vec3& point_camera) const = 0;

    /**
     * @brief Unproject a pixel to a camera-frame point / ray.
     * @param pixel Pixel (u, v).
     * @param depth Depth along the ray (meters); default 1.0 ≈ unit direction.
     * @return Camera-frame 3D point.
     */
    virtual Vec3 Unproject(const Vec2& pixel,
                                         double depth = 1.0) const = 0;

    /**
     * @brief Model type enum.
     * @return `Type` value.
     */
    virtual Type type() const = 0;

    /**
     * @brief Model name string (factory / logging).
     * @return Static C string such as `"pinhole"` or `"kannala_brandt"`.
     */
    virtual const char* type_name() const = 0;

    /**
     * @brief Focal length fx (pixels).
     */
    virtual double fx() const = 0;
    /**
     * @brief Focal length fy (pixels).
     */
    virtual double fy() const = 0;
    /**
     * @brief Principal point cx (pixels).
     */
    virtual double cx() const = 0;
    /**
     * @brief Principal point cy (pixels).
     */
    virtual double cy() const = 0;

    /**
     * @brief Image width; 0 means unknown.
     * @return Width in pixels.
     */
    virtual int width() const { return width_; }

    /**
     * @brief Image height; 0 means unknown.
     * @return Height in pixels.
     */
    virtual int height() const { return height_; }

    /**
     * @brief Set image size (equirectangular / bounds checks, etc.).
     * @param width Width (pixels).
     * @param height Height (pixels).
     */
    void set_image_size(int width, int height) {
        width_ = width;
        height_ = height;
    }

    /**
     * @brief Whether a camera-frame point lies in the model's valid FOV.
     * @param point_camera Camera-frame point.
     * @return true if valid. Default: pinhole-like requires z>1e-6; wide/omni relaxed.
     */
    virtual bool IsInValidRange(const Vec3& point_camera) const {
        return point_camera.z() > 1e-6 || type() == Type::kEquirectangular ||
               type() == Type::kUcm || type() == Type::kEucm ||
               type() == Type::kDoubleSphere || type() == Type::kKannalaBrandt ||
               type() == Type::kFov;
    }

    long unsigned int id = 0;  ///< Instance ID; usually from `next_id++` on construct.
    static long unsigned int next_id;  ///< Global auto-increment ID seed.

protected:
    int width_ = 0;   ///< Image width; 0=unknown.
    int height_ = 0;  ///< Image height; 0=unknown.
};

/**
 * @brief Convert `GeometricCamera::Type` to a stable English name.
 * @param type Model enum.
 * @return e.g. `"pinhole"`; `"unknown"` if unrecognized.
 */
const char* GeometricCameraTypeName(GeometricCamera::Type type);

}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SENSOR_GEOMETRIC_CAMERA_HPP_
