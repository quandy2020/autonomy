/*
 * Copyright 2026 The Openbot Authors
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

#pragma once

namespace autonomy::localization::atlas {
namespace map {

//! Per-cell stats inside a SubGrid (2.5D occupancy).
struct GridData {
    explicit GridData(unsigned int occupy_sum = 0, unsigned int visit_sum = 0)
        : hit_cnt_(occupy_sum), visit_cnt_(visit_sum) {}

    unsigned int hit_cnt_ = 0;
    unsigned int visit_cnt_ = 0;
    //! Lowest height relative to floor; passable from above, not from below.
    float height_ = 10000.f;
};

}  // namespace map
}  // namespace autonomy::localization::atlas
