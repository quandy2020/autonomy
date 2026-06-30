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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_DATA_FRAME_STATISTICS_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_DATA_FRAME_STATISTICS_HPP_

#include "autonomy/localization/atlas/type.hpp"

#include <vector>
#include <unordered_map>
#include <memory>

namespace autonomy::localization::atlas {
namespace data {

class frame;
class keyframe;

class frame_statistics {
public:
    /**
     * Constructor
     */
    frame_statistics() = default;

    /**
     * Destructor
     */
    virtual ~frame_statistics() = default;

    /**
     * Update frame statistics
     * @param frm
     * @param is_lost
     */
    void update_frame_statistics(const data::frame& frm, const bool is_lost);

    /**
     * Replace a keyframe which will be erased in frame statistics
     * @param old_keyfrm
     * @param new_keyfrm
     */
    void replace_reference_keyframe(const std::shared_ptr<data::keyframe>& old_keyfrm, const std::shared_ptr<data::keyframe>& new_keyfrm);

    /**
     * Get frame IDs of each of the reference keyframes
     * @return
     */
    std::unordered_map<std::shared_ptr<data::keyframe>, std::vector<unsigned int>> get_frame_id_of_reference_keyframes() const;

    /**
     * Get the number of the contained valid frames
     * @return
     */
    unsigned int get_num_valid_frames() const;

    /**
     * Get reference keyframes of each of the frames
     * @return
     */
    std::map<unsigned int, std::shared_ptr<data::keyframe>> get_reference_keyframes() const;

    /**
     * Get relative camera poses from the corresponding reference keyframes
     * @return
     */
    eigen_alloc_map<unsigned int, Mat44_t> get_relative_cam_poses() const;

    /**
     * Get timestamps
     * @return
     */
    std::map<unsigned int, double> get_timestamps() const;

    /**
     * Get lost frame flags
     * @return
     */
    std::map<unsigned int, bool> get_lost_frames() const;

    /**
     * Clear frame statistics
     */
    void clear();

private:
    //! Reference keyframe, frame ID associated with the keyframe
    std::unordered_map<std::shared_ptr<data::keyframe>, std::vector<unsigned int>> frm_ids_of_ref_keyfrms_;

    //! Number of valid frames
    unsigned int num_valid_frms_ = 0;

    // Size of all the following variables is the number of frames
    //! Reference keyframes for each frame
    std::unordered_map<unsigned int, std::shared_ptr<data::keyframe>> ref_keyfrms_;
    //! Relative pose against reference keyframe for each frame
    eigen_alloc_unord_map<unsigned int, Mat44_t> rel_cam_poses_from_ref_keyfrms_;
    //! Timestamp for each frame
    std::unordered_map<unsigned int, double> timestamps_;
    //! Flag whether each frame is lost or not
    std::unordered_map<unsigned int, bool> is_lost_frms_;
};

} // namespace data
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_DATA_FRAME_STATISTICS_HPP_
