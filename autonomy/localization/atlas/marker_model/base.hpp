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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MARKER_MODEL_BASE_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MARKER_MODEL_BASE_HPP_

#include "autonomy/localization/atlas/type.hpp"

#include <string>
#include <limits>

#include "yaml-cpp/yaml.h"
#include <nlohmann/json_fwd.hpp>

namespace autonomy::localization::atlas {
namespace marker_model {

class base {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Constructor
    explicit base(double width);

    //! Destructor
    virtual ~base();

    //! marker geometry
    const double width_;
    eigen_alloc_vector<Vec3_t> corners_pos_;

    //! Encode marker_model information as JSON
    virtual nlohmann::json to_json() const;
};

std::ostream& operator<<(std::ostream& os, const base& params);

} // namespace marker_model
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MARKER_MODEL_BASE_HPP_
