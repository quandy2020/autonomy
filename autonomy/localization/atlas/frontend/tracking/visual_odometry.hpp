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
 * @file visual_odometry.hpp
 * @brief Pure-vision frontend wrapper: FrontendBase delegates to tracking::Tracker
 *        (mono / stereo / RGB-D from `camera.sensor`).
 *
 * Registry name `"vo"`. Init selects the rig from `AtlasConfig::camera_sensor`.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_TRACKING_VISUAL_ODOMETRY_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_TRACKING_VISUAL_ODOMETRY_HPP_

#include "autonomy/localization/atlas/frontend/frontend_base.hpp"
#include "autonomy/localization/atlas/frontend/tracking/tracker.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @class autonomy::localization::atlas::VisualOdometry
 * @brief Visual odometry frontend (FrontendBase → Tracker, no IMU fusion).
 *
 * @note Process / tracking run on the Tracking thread; shares Tracker with VisualInertial.
 */
class VisualOdometry : public FrontendBase {
public:
    /**
     * @brief Initialize Tracker for `config.camera_sensor` without IMU.
     * @param config Atlas configuration.
     * @return Return value of Tracker::Init.
     */
    bool Init(const AtlasConfig& config) override;

    /**
     * @brief Forward SensorData to Tracker::Process.
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
     * @brief Return FrontendMode::kVo.
     */
    FrontendMode Mode() const override { return FrontendMode::kVo; }

    /**
     * @brief Pop a pending system keyframe from Tracker.
     * @param[out] out Keyframe output.
     * @return Whether a keyframe was popped.
     */
    bool ConsumePendingKeyframe(Keyframe* out) override;

private:
    tracking::Tracker tracker_;  ///< Underlying tracking engine
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_TRACKING_VISUAL_ODOMETRY_HPP_
