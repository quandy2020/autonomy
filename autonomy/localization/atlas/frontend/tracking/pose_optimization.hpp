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
 * Lightweight PoseOptimization stand-in for ORB-SLAM3 Optimizer::PoseOptimization
 * (OpenCV PnP). Full g2o BA lands later.
 */

/**
 * @file pose_optimization.hpp
 * @brief Frame pose refinement entry: optimize Frame T_cw from matched MapPoints.
 *
 * Corresponds to ORB-SLAM3 Optimizer::PoseOptimization; current implementation
 * forwards to backend::Optimizer::PoseOptimization (Ceres).
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_TRACKING_POSE_OPTIMIZATION_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_TRACKING_POSE_OPTIMIZATION_HPP_

#include "autonomy/localization/atlas/frontend/tracking/frame.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace tracking {

/**
 * @brief Refine camera pose from MapPoints associated with the current frame;
 *        mark outliers.
 * @param frame Frame to optimize; must have initial pose and map_points / outliers.
 * @return Number of inlier matches; may return 0 on failure or insufficient obs.
 *
 * @note Called on the Tracking thread after TrackLocalMap / motion-model tracking;
 *       updates @p frame T_cw and outlier flags in place.
 */
int PoseOptimization(Frame* frame);

}  // namespace tracking
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_TRACKING_POSE_OPTIMIZATION_HPP_
