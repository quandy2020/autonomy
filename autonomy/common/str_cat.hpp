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

#pragma once

#include <sstream>
#include <string>
#include <utility>

namespace autonomy {
namespace common {

template <typename T>
void StrAppend(std::string* out, const T& value) {
    std::ostringstream stream;
    stream << value;
    out->append(stream.str());
}

template <typename... Args>
std::string StrCat(Args&&... args) {
    std::string out;
    out.reserve(64);
    (StrAppend(&out, std::forward<Args>(args)), ...);
    return out;
}

template <typename Range, typename Formatter>
std::string StrJoin(const Range& range, const std::string& delimiter,
                    Formatter formatter) {
    std::string out;
    bool first = true;
    for (const auto& item : range) {
        if (!first) {
            out.append(delimiter);
        }
        formatter(&out, item);
        first = false;
    }
    return out;
}

}  // namespace common
}  // namespace autonomy
