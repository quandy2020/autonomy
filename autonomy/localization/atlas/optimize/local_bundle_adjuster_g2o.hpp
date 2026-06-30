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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_G2O_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_G2O_HPP_

#include "autonomy/localization/atlas/optimize/local_bundle_adjuster.hpp"

#include <memory>

namespace autonomy::localization::atlas {

namespace data {
class keyframe;
class map_database;
} // namespace data

namespace optimize {

class local_bundle_adjuster_g2o : public local_bundle_adjuster {
public:
    /**
     * Constructor
     * @param yaml_node
     * @param num_first_iter
     * @param num_second_iter
     */
    explicit local_bundle_adjuster_g2o(const YAML::Node& yaml_node,
                                       const unsigned int num_first_iter = 5,
                                       const unsigned int num_second_iter = 10);

    /**
     * Destructor
     */
    virtual ~local_bundle_adjuster_g2o() = default;

    /**
     * Perform optimization
     * @param map_db
     * @param curr_keyfrm
     * @param force_stop_flag
     */
    void optimize(data::map_database* map_db, const std::shared_ptr<data::keyframe>& curr_keyfrm, bool* const force_stop_flag) const override;

private:
    //! number of iterations of first optimization
    const unsigned int num_first_iter_;
    //! number of iterations of second optimization
    const unsigned int num_second_iter_;
    //!
    const unsigned int use_additional_keyframes_for_monocular_ = false;
};

} // namespace optimize
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_G2O_HPP_
