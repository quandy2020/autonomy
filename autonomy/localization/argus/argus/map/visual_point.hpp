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
 * @file visual_point.hpp
 * @brief Visual map point with patch observations (FAST-LIVO2 VisualPoint).
 *
 * Related type: `visual_feature.hpp` (`Feature`).
 */

#ifndef ARGUS_MAP_VISUAL_POINT_HPP_
#define ARGUS_MAP_VISUAL_POINT_HPP_

#include <list>

#include <Eigen/Core>

#include "argus/map/visual_feature.hpp"

namespace argus {
namespace map {

/** @brief 3D visual map point with patch observations. */
class VisualPoint
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d pos_ =
        Eigen::Vector3d::Zero(); /**< @brief 3D position in world frame. */
    Eigen::Vector3d normal_ =
        Eigen::Vector3d::Zero(); /**< @brief Surface normal. */
    Eigen::Matrix3d normal_information_ =
        Eigen::Matrix3d::Zero(); /**< @brief Normal uncertainty. */
    Eigen::Vector3d previous_normal_ =
        Eigen::Vector3d::Zero(); /**< @brief Previous normal estimate. */
    std::list<Feature*> obs_;    /**< @brief Patch observations. */
    Eigen::Matrix3d covariance_ =
        Eigen::Matrix3d::Zero(); /**< @brief Position covariance. */
    bool is_converged_ = false;  /**< @brief True if position has converged. */
    bool is_normal_initialized_ =
        false;                    /**< @brief True if normal is initialized. */
    bool has_ref_patch_ = false;  /**< @brief True if @c ref_patch is set. */
    Feature* ref_patch = nullptr; /**< @brief Reference patch for matching. */

    /** @param pos Initial world-frame position (m). */
    explicit VisualPoint(const Eigen::Vector3d& pos);
    ~VisualPoint();

    VisualPoint(const VisualPoint&) = delete;
    VisualPoint& operator=(const VisualPoint&) = delete;

    /**
     * @brief Pick observation with minimum score from @p framepos viewpoint.
     * @param framepos Camera position in world frame.
     * @param ftr Output feature pointer (unchanged if none).
     */
    void FindMinScoreFeature(const Eigen::Vector3d& framepos,
                             Feature*& ftr) const;

    /** @brief Delete all observations except the reference patch. */
    void DeleteNonRefPatchFeatures();

    /**
     * @brief Remove one observation and update reference if needed.
     * @param ftr Feature to remove (must belong to this point).
     */
    void DeleteFeatureRef(Feature* ftr);

    /**
     * @brief Register a new frame observation.
     * @param ftr Feature owned by this point's @c obs_ list.
     */
    void AddFrameRef(Feature* ftr);

    /**
     * @brief Find observation with viewing direction closest to @p pos.
     * @param pos Query camera position (world).
     * @param obs Output best feature (may be null).
     * @param cur_px Current pixel (for parallax checks).
     * @return True if a suitable observation was found.
     */
    bool GetCloseViewObs(const Eigen::Vector3d& pos, Feature*& obs,
                         const Eigen::Vector2d& cur_px) const;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_VISUAL_POINT_HPP_
