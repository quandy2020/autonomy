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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_GRAPH_OPTIMIZER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_GRAPH_OPTIMIZER_HPP_

#include "autonomy/localization/atlas/module/type.hpp"

#include <map>
#include <set>
#include <memory>

namespace autonomy::localization::atlas {

namespace data {
class keyframe;
class map_database;
} // namespace data

namespace optimize {

class graph_optimizer {
public:
    /**
     * Constructor
     * @param yaml_node
     * @param fix_scale
     */
    explicit graph_optimizer(const YAML::Node& yaml_node, data::map_database* map_db, bool fix_scale);

    /**
     * Destructor
     */
    virtual ~graph_optimizer() = default;

    /**
     * Perform pose graph optimization
     * @param loop_keyfrm
     * @param curr_keyfrm
     * @param non_corrected_Sim3s
     * @param pre_corrected_Sim3s
     * @param loop_connections
     */
    void optimize(const std::shared_ptr<data::keyframe>& loop_keyfrm, const std::shared_ptr<data::keyframe>& curr_keyfrm,
                  const module::keyframe_Sim3_pairs_t& non_corrected_Sim3s,
                  const module::keyframe_Sim3_pairs_t& pre_corrected_Sim3s,
                  const std::map<std::shared_ptr<data::keyframe>, std::set<std::shared_ptr<data::keyframe>>>& loop_connections,
                  std::unordered_map<unsigned int, unsigned int>& found_lm_to_ref_keyfrm_id) const;

private:
    data::map_database* map_db_ = nullptr;
    const bool fix_scale_;

    unsigned int min_num_shared_lms_ = 100;
};

} // namespace optimize
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_GRAPH_OPTIMIZER_HPP_
