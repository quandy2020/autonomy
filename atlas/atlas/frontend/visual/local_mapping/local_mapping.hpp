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

/**
 * @file local_mapping.hpp
 * @brief Visual local mapping. Reprojection BA stays here and emits pose
 * factors.
 */

#ifndef ATLAS_FRONTEND_VISUAL_LOCAL_MAPPING_HPP_
#define ATLAS_FRONTEND_VISUAL_LOCAL_MAPPING_HPP_

#include "backend/pose_graph/pose_graph.hpp"

namespace atlas {
namespace frontend {
namespace visual {

class LocalMapping {
 public:
  LocalMapping() = default;
};

}  // namespace visual
}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_VISUAL_LOCAL_MAPPING_HPP_
