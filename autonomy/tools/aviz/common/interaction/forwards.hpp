/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef AVIZ_COMMON__INTERACTION__FORWARDS_HPP_
#define AVIZ_COMMON__INTERACTION__FORWARDS_HPP_

#include <QColor>  // NOLINT: cpplint is unable to handle the include order here
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

#include "autonomy/tools/aviz/common/logging.hpp"

namespace aviz {
namespace common {
namespace interaction {

using CollObjectHandle = uint32_t;
using V_CollObject = std::vector<CollObjectHandle>;
using VV_CollObject = std::vector<V_CollObject>;
using S_CollObject = std::set<CollObjectHandle>;

using S_uint64 = std::set<uint64_t>;
using V_uint64 = std::vector<uint64_t>;

struct Picked {
    explicit Picked(CollObjectHandle _handle = 0)
        : handle(_handle), pixel_count(1) {}

    CollObjectHandle handle;
    int pixel_count;
    S_uint64 extra_handles;
};

using M_Picked = std::unordered_map<CollObjectHandle, Picked>;

/// Convert QColor to handle (for selection picking)
inline uint32_t colorToHandle(const QColor& color) {
    return (static_cast<int>(color.red()) << 16) |
           (static_cast<int>(color.green()) << 8) |
           static_cast<int>(color.blue());
}

/// Convert handle to QColor (for selection picking)
inline QColor handleToColor(CollObjectHandle handle) {
    return QColor((handle >> 16) & 0xFF, (handle >> 8) & 0xFF, handle & 0xFF);
}

}  // namespace interaction
}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__INTERACTION__FORWARDS_HPP_
