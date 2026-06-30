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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_TRANSFORM_OPTIMIZER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_TRANSFORM_OPTIMIZER_HPP_

#include <g2o/types/sim3/types_seven_dof_expmap.h>

#include <vector>
#include <memory>

namespace autonomy::localization::atlas {

namespace data {
class keyframe;
class landmark;
} // namespace data

namespace optimize {

class transform_optimizer {
public:
    /**
     * Constructor
     * @param fix_scale
     * @param num_iter
     */
    explicit transform_optimizer(const bool fix_scale, const unsigned int num_iter = 10);

    /**
     * Destructor
     */
    virtual ~transform_optimizer() = default;

    /**
     * Perform optimization
     * @param keyfrm_1
     * @param keyfrm_2
     * @param matched_lms_in_keyfrm_2
     * @param g2o_Sim3_12
     * @param chi_sq
     * @return
     */
    unsigned int optimize(const std::shared_ptr<data::keyframe>& keyfrm_1, const std::shared_ptr<data::keyframe>& keyfrm_2,
                          std::vector<std::shared_ptr<data::landmark>>& matched_lms_in_keyfrm_2,
                          g2o::Sim3& g2o_Sim3_12, const float chi_sq) const;

private:
    //! transform is Sim3 or SE3
    const bool fix_scale_;

    //! number of iterations of optimization
    const unsigned int num_iter_;
};

} // namespace optimize
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_TRANSFORM_OPTIMIZER_HPP_
