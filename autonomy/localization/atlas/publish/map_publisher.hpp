/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_PUBLISH_MAP_PUBLISHER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_PUBLISH_MAP_PUBLISHER_HPP_

#include "autonomy/localization/atlas/type.hpp"

#include <mutex>
#include <memory>

namespace autonomy::localization::atlas {

class config;

namespace data {
class keyframe;
class landmark;
class landmark_plane;
class landmark_line;
class map_database;
class marker;
} // namespace data

namespace publish {

class map_publisher {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Constructor
     * @param cfg
     * @param map_db
     */
    map_publisher(const std::shared_ptr<config>& cfg, data::map_database* map_db);

    /**
     * Destructor
     */
    virtual ~map_publisher();

    /**
     * Set current camera pose
     * NOTE: should be accessed from tracker thread
     * @param cam_pose_cw
     */
    void set_current_cam_pose(const Mat44_t& cam_pose_cw);

    /**
     * Get current camera pose
     * NOTE: should be accessed from viewer thread
     * @return
     */
    Mat44_t get_current_cam_pose();

    /**
     * Get all keyframes
     * @param all_keyfrms
     * @return number of keyframes in map
     */
    unsigned int get_keyframes(std::vector<std::shared_ptr<data::keyframe>>& all_keyfrms);

    /**
     * Get all landmarks and local landmarks
     * @param all_landmarks
     * @param local_landmarks
     * @return number of landmarks in map
     */
    unsigned int get_landmarks(std::vector<std::shared_ptr<data::landmark>>& all_landmarks,
                               std::set<std::shared_ptr<data::landmark>>& local_landmarks);

    unsigned int get_markers(std::vector<std::shared_ptr<data::marker>>& all_markers);

    unsigned int get_landmark_planes(std::vector<std::shared_ptr<data::landmark_plane>>& all_planes) const;

    unsigned int get_landmark_lines(std::vector<std::shared_ptr<data::landmark_line>>& all_lines) const;

private:
    //! config
    std::shared_ptr<config> cfg_;
    //! map database
    data::map_database* map_db_;

    // -------------------------------------------
    //! mutex to access camera pose
    std::mutex mtx_cam_pose_;
    Mat44_t cam_pose_cw_ = Mat44_t::Identity();
    Mat44_t cam_pose_wc_ = Mat44_t::Identity();
};

} // namespace publish
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_PUBLISH_MAP_PUBLISHER_HPP_
