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

#pragma once

#include <sstream>
#include <string>

namespace aviz {
namespace common {

/// std::stringstream subclass which defaults to the "C" locale.
/**
 * This useful so that the serialization of numbers is uniform across locales.
 *
 * For reading floats in, use parseFloat() instead of operator>>,
 * because operator>> is the one from std::stringstream which only
 * handles "C" style floats.
 * parseFloat() handles "C" and also European-style floats which use the ",",
 * like "1,2" parses to 1.2f
 */
class UniformStringStream : public std::stringstream
{
public:
    UniformStringStream();
    explicit UniformStringStream(const std::string& str);

    /// Parse a float, supporting both period- and comma- style floats (1,2
    /// and 1.2).
    /**
     * Uses operator>>(std::string&) internally, so consumes up to next
     * whitespace from the stream.
     */
    void parseFloat(float& f);
};

}  // namespace common
}  // namespace aviz
