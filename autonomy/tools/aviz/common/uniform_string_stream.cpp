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

#include "autonomy/tools/aviz/common/uniform_string_stream.hpp"

#include <locale>
#include <string>

namespace aviz {
namespace common {

UniformStringStream::UniformStringStream() {
    imbue(std::locale("C"));
}

UniformStringStream::UniformStringStream(const std::string& str)
    : std::stringstream(str) {
    imbue(std::locale("C"));
}

void UniformStringStream::parseFloat(float& f) {
    std::string float_string;
    *this >> float_string;
    size_t comma_index = float_string.find(',');
    if (comma_index != std::string::npos) {
        float_string[comma_index] = '.';
    }
    UniformStringStream float_reader(float_string);
    float_reader >> f;
    if (float_reader.fail()) {
        this->setstate(std::ios::failbit);
    }
}

}  // namespace common
}  // namespace aviz
