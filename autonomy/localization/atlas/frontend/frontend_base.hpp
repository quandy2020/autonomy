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
 * @file frontend_base.hpp
 * @brief Atlas frontend abstract interface: init, per-frame process, odometry,
 *        and keyframe handoff.
 *
 * Concrete types include VisualOdometry / VisualInertial (delegate to
 * tracking::Tracker). Instantiated via CreateFrontend / FrontendRegistry.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_FRONTEND_BASE_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_FRONTEND_BASE_HPP_

#include "autonomy/localization/atlas/common/config.hpp"
#include "autonomy/localization/atlas/common/types.hpp"
#include "autonomy/localization/atlas/sensor/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @class autonomy::localization::atlas::FrontendBase
 * @brief Unified frontend base for VO / VIO (and future LIO).
 *
 * Typical usage: `Init` → loop `Process(SensorData)` → `GetResult` for pose;
 * mapping side pops pending system keyframes via `ConsumePendingKeyframe`.
 *
 * @note Calling thread is set by the upper scheduler; default impl is not
 *       thread-safe — use a single thread or lock externally.
 */
class FrontendBase {
public:
    virtual ~FrontendBase() = default;

    /**
     * @brief Initialize the frontend from Atlas config (camera, ORB, map, …).
     * @param config Global Atlas config (calibration, vocabulary path, sensor mode).
     * @return true on success; false on failure (missing calib/vocab, etc.).
     */
    virtual bool Init(const AtlasConfig& config) = 0;

    /**
     * @brief Process one multi-sensor packet (image / depth / IMU, …).
     * @param data SensorData at the current timestamp.
     * @return true if tracking advanced; false on invalid input or internal error.
     * @note Usually called synchronously on the Tracking thread.
     */
    virtual bool Process(const SensorData& data) = 0;

    /**
     * @brief Read the latest valid odometry result.
     * @param[out] out If non-null, writes pose, timestamp, and validity.
     * @return true if a result is available; false if none yet.
     */
    virtual bool GetResult(OdometryResult* out) const = 0;

    /**
     * @brief Reset frontend internal state (map/tracking semantics are impl-specific).
     */
    virtual void Reset() = 0;

    /**
     * @brief Return the frontend run mode (VO / VIO, …).
     * @return FrontendMode enum value.
     */
    virtual FrontendMode Mode() const = 0;

    /**
     * @brief Pop one pending system keyframe for MapManager, if any.
     * @param[out] out If non-null, writes Keyframe; empty-queue semantics are impl-defined.
     * @return true if a keyframe was popped; base default returns false when empty.
     * @note Default is a no-op; VO/VIO wrappers forward to Tracker.
     */
    virtual bool ConsumePendingKeyframe(Keyframe* out) {
        (void)out;
        return false;
    }
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_FRONTEND_BASE_HPP_
