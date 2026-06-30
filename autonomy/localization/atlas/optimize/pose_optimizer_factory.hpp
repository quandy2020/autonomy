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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_POSE_OPTIMIZER_FACTORY_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_POSE_OPTIMIZER_FACTORY_HPP_

#include "autonomy/localization/atlas/optimize/pose_optimizer_g2o.hpp"
#include "autonomy/localization/atlas/util/yaml.hpp"

#include <memory>
#include <stdexcept>
#include <string>

namespace autonomy::localization::atlas {

namespace optimize {

class pose_optimizer_factory {
public:
    static std::unique_ptr<pose_optimizer> create(const YAML::Node& yaml_node) {
        const auto& backend = yaml_node["backend"].as<std::string>("g2o");
        if (backend != "g2o") {
            throw std::runtime_error("Invalid backend: only g2o is supported");
        }
        YAML::Node g2o_node = util::yaml_optional_ref(yaml_node, "g2o");
        return std::unique_ptr<pose_optimizer>(new pose_optimizer_g2o(
            g2o_node["num_trials_robust"].as<unsigned int>(2),
            g2o_node["num_trials"].as<unsigned int>(2),
            g2o_node["num_each_iter"].as<unsigned int>(10)));
    }
};

} // namespace optimize
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_POSE_OPTIMIZER_FACTORY_HPP_
