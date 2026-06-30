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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_FEATURE_ORB_PARAMS_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_FEATURE_ORB_PARAMS_HPP_

#include <nlohmann/json_fwd.hpp>
#include "yaml-cpp/yaml.h"
#include <cmath>

namespace autonomy::localization::atlas {
namespace feature {

struct orb_params {
    orb_params() = delete;

    //! Constructor
    orb_params(const std::string& name, const float scale_factor, const unsigned int num_levels,
               const unsigned int ini_fast_thr, const unsigned int min_fast_thr);
    orb_params(const std::string& name);

    //! Constructor
    explicit orb_params(const YAML::Node& yaml_node);

    //! Destructor
    virtual ~orb_params() = default;

    nlohmann::json to_json() const;

    //! name (id for saving)
    const std::string name_;

    const float scale_factor_ = 1.2;
    const float log_scale_factor_ = std::log(1.2);
    const unsigned int num_levels_ = 8;
    const unsigned int ini_fast_thr_ = 20;
    const unsigned int min_fast_thr_ = 7;

    //! A list of the scale factor of each pyramid layer
    std::vector<float> scale_factors_;
    std::vector<float> inv_scale_factors_;
    //! A list of the sigma of each pyramid layer
    std::vector<float> level_sigma_sq_;
    std::vector<float> inv_level_sigma_sq_;

    //! Calculate scale factors
    static std::vector<float> calc_scale_factors(const unsigned int num_scale_levels, const float scale_factor);

    //! Calculate inverses of scale factors
    static std::vector<float> calc_inv_scale_factors(const unsigned int num_scale_levels, const float scale_factor);

    //! Calculate squared sigmas at all levels
    static std::vector<float> calc_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor);

    //! Calculate inverses of squared sigmas at all levels
    static std::vector<float> calc_inv_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor);
};

std::ostream& operator<<(std::ostream& os, const orb_params& oparam);

} // namespace feature
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_FEATURE_ORB_PARAMS_HPP_
