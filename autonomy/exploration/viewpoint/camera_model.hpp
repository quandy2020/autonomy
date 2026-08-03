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

#pragma once

#include <string>
#include <vector>

#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud.pb.h>
#include "autonomy/exploration/proto/exploration_options.pb.h"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace exploration {

class PlanningEnv;

/**
 * @file camera_model.hpp
 * @brief Pinhole RGBD camera model with FoV limits for exploration gain.
 */

/**
 * @class CameraModel
 * @brief Pinhole camera with FoV tests; map→camera pose comes from the TF tree.
 */
class CameraModel
{
public:
    /**
     * @brief Constructor with exploration options.
     * @param options Exploration options containing camera FoV parameters
     */
    explicit CameraModel(const proto::ExplorationOptions& options);

    /**
     * @brief Update camera parameters from exploration options.
     * @param options Exploration options
     */
    void SetOptions(const proto::ExplorationOptions& options);

    /**
     * @brief Bind the process-local TF buffer used for camera pose lookup.
     * @param tf_buffer Non-owning pointer to transform::Buffer (may be nullptr)
     */
    void SetTfBuffer(transform::Buffer* tf_buffer);

    /**
     * @brief Set map and camera frame ids used in lookupTransform.
     * @param map_frame Source / world frame (e.g. "map")
     * @param camera_frame Target optical / body frame (e.g. "camera_link")
     */
    void SetFrames(const std::string& map_frame,
                   const std::string& camera_frame);

    /**
     * @brief Update intrinsics from CameraInfo.
     * @param info Camera intrinsic / distortion message
     */
    void SetFromCameraInfo(const automsgs::msgs::sensor_msgs::CameraInfo& info);

    /**
     * @brief Check whether a map-frame point is visible in the TF camera FoV.
     * @param wx Map-frame x [m]
     * @param wy Map-frame y [m]
     * @param wz Map-frame z [m]
     * @param env Optional planning env for 2D LOS occlusion
     * @return true if TF lookup succeeds and the point lies inside FoV (+ LOS)
     */
    bool IsVisible(double wx, double wy, double wz,
                   const PlanningEnv* env = nullptr) const;

    /**
     * @brief Visibility test with an explicit map_T_camera extrinsic.
     *
     * Used for hypothetical viewpoints that are not published on the TF tree.
     * @param map_t_camera Extrinsic transform of the camera in the map frame
     * @param wx Map-frame x [m]
     * @param wy Map-frame y [m]
     * @param wz Map-frame z [m]
     * @param env Optional planning env for 2D LOS occlusion
     * @return true if the point lies inside FoV (+ LOS)
     */
    bool IsVisible(const automsgs::msgs::geometry_msgs::Transform& map_t_camera,
                   double wx, double wy, double wz,
                   const PlanningEnv* env = nullptr) const;

    /**
     * @brief Coverage gain for uncovered targets from an explicit camera pose.
     * @param map_t_camera Extrinsic of the candidate camera in the map frame
     * @param targets Candidate coverage target points
     * @param uncovered Per-target uncovered flags (same size as targets)
     * @param env Optional planning env for 2D LOS occlusion
     * @return Scalar coverage gain score
     */
    double ComputeGain(
        const automsgs::msgs::geometry_msgs::Transform& map_t_camera,
        const std::vector<automsgs::msgs::geometry_msgs::Point>& targets,
        const std::vector<bool>& uncovered,
        const PlanningEnv* env = nullptr) const;

    /**
     * @brief Get horizontal FoV in radians.
     * @return Horizontal FoV [rad]
     */
    double hfov_rad() const { return hfov_rad_; }

    /**
     * @brief Get vertical FoV in radians.
     * @return Vertical FoV [rad]
     */
    double vfov_rad() const { return vfov_rad_; }

    /**
     * @brief Get near clipping distance.
     * @return Near range [m]
     */
    double z_near() const { return z_near_; }

    /**
     * @brief Get far clipping distance.
     * @return Far range [m]
     */
    double z_far() const { return z_far_; }

    /**
     * @brief Get map frame id.
     * @return Map frame id
     */
    const std::string& map_frame() const { return map_frame_; }

    /**
     * @brief Get camera frame id.
     * @return Camera frame id
     */
    const std::string& camera_frame() const { return camera_frame_; }

private:
    /**
     * @brief Look up camera_frame ← map_frame from the TF buffer.
     * @param camera_t_map Output transform that maps map points into camera
     * @return true on successful lookup
     */
    bool LookupCameraFromMap(
        automsgs::msgs::geometry_msgs::Transform* camera_t_map) const;

    /**
     * @brief Apply a rigid transform to a 3D point.
     * @param transform Transform to apply
     * @param x Input x
     * @param y Input y
     * @param z Input z
     * @param out_x Output x
     * @param out_y Output y
     * @param out_z Output z
     */
    static void TransformPoint(const automsgs::msgs::geometry_msgs::Transform& transform,
                               double x, double y, double z, double* out_x,
                               double* out_y, double* out_z);

    /**
     * @brief Invert map_T_camera to get camera_T_map.
     * @param map_t_camera Extrinsic of camera in map
     * @return camera_T_map
     */
    static automsgs::msgs::geometry_msgs::Transform InvertTransform(
        const automsgs::msgs::geometry_msgs::Transform& map_t_camera);

    /**
     * @brief Project a camera-frame point and test FoV membership.
     * @param cx Camera-frame x
     * @param cy Camera-frame y
     * @param cz Camera-frame z
     * @param dist Euclidean distance from camera origin
     * @return true if the point projects inside the FoV
     */
    bool ProjectVisible(double cx, double cy, double cz, double dist) const;

    /**
     * @brief Optional 2D LOS test from camera origin to a map point.
     * @param ox Camera origin x [m]
     * @param oy Camera origin y [m]
     * @param wx Target x [m]
     * @param wy Target y [m]
     * @param env Planning environment (may be nullptr)
     * @return true if occlusion is disabled or LOS is clear
     */
    bool PassesOcclusion(double ox, double oy, double wx, double wy,
                         const PlanningEnv* env) const;

    transform::Buffer* tf_buffer_{nullptr};  //!< @brief non-owning TF buffer
    std::string map_frame_{"map"};           //!< @brief map / world frame
    std::string camera_frame_{"camera_optical_frame"};  //!< @brief camera frame
    float tf_timeout_sec_{0.0f};             //!< @brief lookup timeout [s] (0=latest)

    double fx_{525.0};                    //!< @brief focal length x [px]
    double fy_{525.0};                    //!< @brief focal length y [px]
    double cx_{320.0};                    //!< @brief principal point x [px]
    double cy_{240.0};                    //!< @brief principal point y [px]
    double hfov_rad_{1.518};              //!< @brief horizontal FoV [rad]
    double vfov_rad_{1.012};              //!< @brief vertical FoV [rad]
    double z_near_{0.3};                  //!< @brief near clip [m]
    double z_far_{5.0};                   //!< @brief far clip [m]
    bool use_intrinsics_fov_{false};      //!< @brief use fx/fy for FoV test
    bool occlusion_enabled_{true};        //!< @brief apply 2D LOS in gain/visibility
    bool los_stop_at_unknown_{false};     //!< @brief unknown cells block LOS
};

}  // namespace exploration
}  // namespace autonomy
