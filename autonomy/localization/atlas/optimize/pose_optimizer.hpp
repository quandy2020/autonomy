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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_POSE_OPTIMIZER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_POSE_OPTIMIZER_HPP_

#include "autonomy/localization/atlas/type.hpp"

namespace autonomy::localization::atlas {

namespace data {
class frame;
struct frame_observation;
class keyframe;
} // namespace data

namespace camera {
class base;
} // namespace camera

namespace feature {
struct orb_params;
} // namespace feature

namespace optimize {

class pose_optimizer {
public:
    /**
     * Perform pose optimization
     * @param frm
     * @return
     */
    virtual unsigned int optimize(const data::frame& frm, Mat44_t& optimized_pose, std::vector<bool>& outlier_flags) const = 0;
    virtual unsigned int optimize(const data::keyframe* keyfrm, Mat44_t& optimized_pose, std::vector<bool>& outlier_flags) const = 0;

    virtual unsigned int optimize(const Mat44_t& cam_pose_cw, const data::frame_observation& frm_obs,
                                  const feature::orb_params* orb_params,
                                  const camera::base* camera,
                                  const std::vector<std::shared_ptr<data::landmark>>& landmarks,
                                  Mat44_t& optimized_pose,
                                  std::vector<bool>& outlier_flags) const = 0;
};

} // namespace optimize
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_POSE_OPTIMIZER_HPP_
