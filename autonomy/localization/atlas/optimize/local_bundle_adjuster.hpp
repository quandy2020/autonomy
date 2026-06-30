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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_HPP_

#include <memory>

namespace autonomy::localization::atlas {

namespace data {
class keyframe;
class map_database;
} // namespace data

namespace optimize {

class local_bundle_adjuster {
public:
    /**
     * Perform optimization
     * @param map_db
     * @param curr_keyfrm
     * @param force_stop_flag
     */
    virtual void optimize(data::map_database* map_db, const std::shared_ptr<data::keyframe>& curr_keyfrm, bool* const force_stop_flag) const = 0;
};

} // namespace optimize
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_HPP_
