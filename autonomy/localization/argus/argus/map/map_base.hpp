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
 * @file map_base.hpp
 * @brief Common map interface for Argus geometric / visual maps.
 */

#ifndef ARGUS_MAP_MAP_BASE_HPP_
#define ARGUS_MAP_MAP_BASE_HPP_

#include <cstddef>

namespace argus {
namespace map {

/**
 * @brief Abstract interface shared by voxel, iKD-tree, and visual maps.
 */
class MapBase
{
public:
    virtual ~MapBase() = default;

    /** @brief Remove all stored map elements. */
    virtual void Clear() = 0;

    /**
     * @brief Number of stored elements (map-specific semantics).
     * @return Element count.
     */
    virtual std::size_t size() const = 0;

    /**
     * @brief Whether the map holds no elements.
     * @return True if empty.
     */
    virtual bool empty() const = 0;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_MAP_BASE_HPP_
