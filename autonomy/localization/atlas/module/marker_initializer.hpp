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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MODULE_MARKER_INITIALIZER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MODULE_MARKER_INITIALIZER_HPP_

#include "autonomy/localization/atlas/type.hpp"

namespace autonomy::localization::atlas {

namespace data {
class marker;
} // namespace data

namespace module {

class marker_initializer {
public:
    static void check_marker_initialization(data::marker& mkr, size_t needed_observations_for_initialization);

private:
    const size_t required_keyframes_for_marker_initialization_ = 3;
};

} // namespace module
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MODULE_MARKER_INITIALIZER_HPP_
