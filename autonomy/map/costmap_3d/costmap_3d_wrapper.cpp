/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/map/costmap_3d/costmap_3d_wrapper.hpp"

namespace autonomy {
namespace map {
namespace costmap_3d {

Costmap3DWrapper::Costmap3DWrapper(const proto::Costmap3DOptions& options)
    : options_{options}
{

}

Costmap3DWrapper::~Costmap3DWrapper()
{

}

}  // namespace costmap_3d
}  // namespace map
}  // namespace autonomy