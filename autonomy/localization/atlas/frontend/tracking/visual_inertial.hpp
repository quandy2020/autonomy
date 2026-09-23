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
 * @file visual_inertial.hpp
 * @brief Visual-inertial frontend wrapper: FrontendBase delegates to
 *        tracking::Tracker (default IMU+RGB-D).
 *
 * Registry name `"vio"`. Init selects mono / stereo / RGB-D inertial from
 * `AtlasConfig::camera_sensor` (`camera.sensor` in YAML).
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_TRACKING_VISUAL_INERTIAL_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_TRACKING_VISUAL_INERTIAL_HPP_

#include "autonomy/localization/atlas/frontend/frontend_base.hpp"
#include "autonomy/localization/atlas/frontend/tracking/tracker.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @class autonomy::localization::atlas::VisualInertial
 * @brief Visual-inertial odometry frontend (FrontendBase → Tracker, with IMU).
 *
 * @note IMU enqueue may be decoupled from the image thread (GrabImuData);
 *       image Process still consumes preintegration on the Tracking thread.
 */
class VisualInertial : public FrontendBase {
public:
    /**
     * @brief Initialize Tracker. Sensor is IMU plus `config.camera_sensor`.
     * @param config Atlas config (rig, IMU noise, extrinsics).
     * @return Return value of Tracker::Init.
     */
    bool Init(const AtlasConfig& config) override;

    /**
     * @brief Forward SensorData (may include IMU sequence) to Tracker.
     * @param data Sensor packet.
     * @return Whether processing succeeded.
     */
    bool Process(const SensorData& data) override;

    /**
     * @brief Read Tracker's latest odometry result.
     * @param[out] out Output result.
     * @return Whether a valid result exists.
     */
    bool GetResult(OdometryResult* out) const override;

    /**
     * @brief Reset the internal Tracker.
     */
    void Reset() override;

    /**
     * @brief Return FrontendMode::kVio.
     */
    FrontendMode Mode() const override { return FrontendMode::kVio; }

    /**
     * @brief Pop a pending system keyframe from Tracker.
     * @param[out] out Keyframe output.
     * @return Whether a keyframe was popped.
     */
    bool ConsumePendingKeyframe(Keyframe* out) override;

private:
    tracking::Tracker tracker_;  ///< Underlying tracking engine (with IMU preintegration)
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_TRACKING_VISUAL_INERTIAL_HPP_
