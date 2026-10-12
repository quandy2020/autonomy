/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/**
 * @file display.hpp
 * @brief Loops over the elements in a container and creates a delimited string.
 */


#include <sstream>
#include <string>

namespace bolza {

/** Loops over the elements in a container and creates a delimited string */
template <typename Container>
/**
 * @brief To Delimited String.
 * @param [in] container: Container.
 * @param [in] delimiter: Delimiter.
 * @return Result value.
 */
std::string ToDelimitedString(const Container& container, const std::string& delimiter = ", ") {
  std::stringstream ss;

  bool addDelimiter = false;
  for (size_t i = 0; /**
  * @brief Size.
  * @return Result value.
  */
 i < container.size(); i++) {
    if (addDelimiter) {
      ss << delimiter;
    }
    ss << container[i];
    addDelimiter = true;
  }

  /**
   * @brief Str.
   * @return Result value.
   */
  return ss.str();
}

}  // namespace bolza
